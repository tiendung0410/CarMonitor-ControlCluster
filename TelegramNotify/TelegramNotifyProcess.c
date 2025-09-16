// TelegramNotifyProcess.c
// Build:  gcc -O2 TelegramNotifyProcess.c -o tg_notify -lcurl
// Run  :  ./tg_notify
// Yêu cầu: Gateway sendto() DataTransfer_t tới /tmp/telegram_notify_socket (SOCK_DGRAM)

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <curl/curl.h>

//========================= CONFIG =========================
#define TELEGRAM_SOCK_PATH  "/tmp/telegram_notify_socket"

// Thay bằng token & chat của bạn
#define TG_BOT_TOKEN  "8489087580:AAGKgzw73xUiK0kf7x0k-8OufFID_mgrgT8"
#define TG_CHAT_ID    "8396115032"      

// Debounce / Rate-limit
#define HOLD_OVERSPEED_SEC   2.0    // vượt tốc phải giữ >= 2s
#define RL_OVERSPEED_SEC     60.0   // mỗi 60s mới báo lại

#define HOLD_TIREOFF_SEC     3.0    // lốp lỗi khi máy tắt giữ >= 3s
#define RL_TIREOFF_SEC       120.0  // mỗi 120s mới báo lại

#define DOOR_AFTER_OFF_SEC   60.0   // cửa mở liên tục 60s sau engine OFF

//===================== STRUCT (match gateway) =====================
typedef struct {
    uint8_t engine_status;       // 1=ON, 0=OFF
    uint8_t light_status;
    uint8_t tire_pressure;       // GIẢ ĐỊNH: 1=Đạt, 0=Không đạt
    uint8_t door_status;         // 1=ĐÓNG, 0=MỞ
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;               // km/h
    uint8_t arrived_distance;
    uint8_t total_distance;      // km (8-bit) - demo, KHÔNG xử lý tràn theo yêu cầu
    uint8_t drived_time;
    uint8_t transmission_gear;
    uint8_t speech_enable;
    uint8_t temp_or_limit_changed;
    float   gps_lat;
    float   gps_lon;
} __attribute__((packed)) VehicleStatus;

typedef struct  {
    uint8_t air_condition_temperature;
    uint8_t speed_limit;         // km/h
    uint8_t light_touch_control;
} controlData_t;

typedef struct {
    VehicleStatus vehicle_status;
    controlData_t control_data;
} __attribute__((packed)) DataTransfer_t;

//===================== TIME & TELEGRAM =====================
static double now_monotonic_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int tg_send_text(const char* text) {
    int ok = 0;
    CURL *curl = curl_easy_init();
    if (!curl) { fprintf(stderr, "[tg] curl init failed\n"); return 0; }

    char url[512];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage", TG_BOT_TOKEN);
    curl_easy_setopt(curl, CURLOPT_URL, url);

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    char *esc = curl_easy_escape(curl, text, (int)strlen(text));
    if (!esc) { curl_slist_free_all(headers); curl_easy_cleanup(curl); return 0; }

    char body[4096];
    snprintf(body, sizeof(body), "chat_id=%s&text=%s", TG_CHAT_ID, esc);
    curl_free(esc);

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 8L);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    if (res != CURLE_OK) fprintf(stderr, "[tg] curl: %s\n", curl_easy_strerror(res));
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    ok = (res == CURLE_OK && http_code == 200);
    if (!ok) fprintf(stderr, "[tg] HTTP %ld (expect 200)\n", http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return ok;
}

//============================= MAIN =============================
int main(void) {
    // Bind socket nhận từ Gateway
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, TELEGRAM_SOCK_PATH, sizeof(addr.sun_path)-1);
    unlink(TELEGRAM_SOCK_PATH); // xóa path cũ nếu còn
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }

    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Trạng thái cho 3 luật đầu
    double last_send_over  = 0.0;
    double last_send_tire  = 0.0;
    bool   os_latch = false;  double os_since = 0.0;  // overspeed latch
    bool   tf_latch = false;  double tf_since = 0.0;  // tire-off latch

    // Cửa mở sau OFF 1 phút
    uint8_t prev_engine = 0xFF; // chưa biết
    bool    engine_off_active = false; double engine_off_ts = 0.0;
    bool    door_alert_sent_after_off = false;

    // Nhắc bảo dưỡng mỗi 10 km (chỉ dùng tổng hiện tại, không xử lý tràn)
    int last_maint_total_sent = -1; // lưu total_distance của mốc 10km đã gửi gần nhất

    for (;;) {
        DataTransfer_t dt;
        ssize_t n = recvfrom(fd, &dt, sizeof(dt), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom"); break;
        }
        if ((size_t)n < sizeof(dt)) {
            fprintf(stderr, "[warn] short packet: %zd\n", n);
            continue;
        }

        double tnow = now_monotonic_s();
        int eng   = (int)dt.vehicle_status.engine_status;   // 1=ON, 0=OFF
        int speed = (int)dt.vehicle_status.speed;           // km/h
        int limit = (int)dt.control_data.speed_limit;       // km/h
        int tire_ok = (int)dt.vehicle_status.tire_pressure; // 1=Đạt, 0=Không đạt
        uint8_t door = dt.vehicle_status.door_status;       // 1=ĐÓNG, 0=MỞ
        uint8_t total = dt.vehicle_status.total_distance;   // km, demo ngắn, không xử lý tràn

        // --------- 1) Vượt giới hạn tốc độ (giữ >=2s, RL 60s) ---------
        if (limit > 0 && speed > limit) {
            if (!os_latch) { os_latch = true; os_since = tnow; }
            else if ((tnow - os_since) >= HOLD_OVERSPEED_SEC) {
                if ((tnow - last_send_over) >= RL_OVERSPEED_SEC) {
                    char msg[128];
                    snprintf(msg, sizeof(msg), "⚠️ VƯỢT TỐC ĐỘ GIỚI HẠN: %d km/h > %d km/h", speed, limit);
                    tg_send_text(msg);
                    last_send_over = tnow;
                }
            }
        } else {
            os_latch = false;
        }

        // --------- 2) Lốp lỗi khi động cơ TẮT (giữ >=3s, RL 120s) ------
        if (eng == 0 && tire_ok == 0) {
            if (!tf_latch) { tf_latch = true; tf_since = tnow; }
            else if ((tnow - tf_since) >= HOLD_TIREOFF_SEC) {
                if ((tnow - last_send_tire) >= RL_TIREOFF_SEC) {
                    tg_send_text("🚨 ÁP SUẤT LỐP KHÔNG ĐẠT. Vui lòng kiểm tra.");
                    last_send_tire = tnow;
                }
            }
        } else {
            tf_latch = false;
        }

        // --------- 3) Cửa vẫn mở sau khi OFF >= 60s --------------------
        if (prev_engine == 1 && eng == 0) {
            engine_off_active = true;
            engine_off_ts = tnow;
            door_alert_sent_after_off = false;
        }
        prev_engine = eng;

        if (engine_off_active && !door_alert_sent_after_off) {
            if ((tnow - engine_off_ts) >= DOOR_AFTER_OFF_SEC && door == 0) {
                tg_send_text("🚨 Cảnh báo: CỬA VẪN ĐANG MỞ .");
                door_alert_sent_after_off = true;
            }
        }
        if (eng == 1) {
            engine_off_active = false;
            door_alert_sent_after_off = false;
        }

        // --------- 4) Nhắc mỗi 10 km dựa trên total_distance ----------
        // Chỉ cần: chia hết cho 10 và khác mốc đã gửi gần nhất -> gửi, không xử lý tràn
        if (total % 10 == 0 && total > 0) {
            if (last_maint_total_sent != (int)total) {
                char msg[160];
                snprintf(msg, sizeof(msg),
                         "🛠️ Nhắc bảo dưỡng: đã đạt mốc %u km .",
                         (unsigned)total);
                tg_send_text(msg);
                last_maint_total_sent = (int)total;
            }
        }
    }

    curl_global_cleanup();
    close(fd);
    return 0;
}
