#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>

#define SOCKET_PATH "/tmp/warning_socket"
#define MQ_NAME "/warning_mq"


// ----------- Định nghĩa VehicleStatus và code cảnh báo ----------
typedef struct {
    uint8_t engine_status;
    uint8_t light_status;
    uint8_t tire_pressure;
    uint8_t door_status;
    uint8_t seat_belt_status;
    uint8_t battery_level;
    uint8_t speed;
    uint8_t arrived_distance;
    uint8_t remain_distance;
    uint8_t drived_time; 
    uint8_t transmission_gear;
    uint8_t reserve;
    uint8_t temp_or_limit_changed; // 1: limit_inc 2: limit_dec 3:temp_inc 4:temp_dec 5:cloud fixed
    float gps_lat;
    float gps_lon;
} __attribute__((packed)) VehicleStatus;

typedef struct  {
    uint8_t air_condition_temperature;
    uint8_t speed_limit;
    uint8_t light_touch_control;
}controlData_t;

typedef struct {
    VehicleStatus vehicle_status;
    controlData_t control_data;
}DataTransfer_t;

typedef enum {
    WARN_NONE = 0,
    WARN_DOOR,
    WARN_SEATBELT,
    WARN_TIRE_PRESSURE,
    WARN_SPEED_EXCEED,
    WARN_TEMP_INC,
    WARN_TEMP_DEC,
    WARN_SPEEDLIMIT_INC,
    WARN_SPEEDLIMIT_DEC,
    WARN_FIXED_CHANGE
} warning_code_t;

typedef struct {
    warning_code_t code;
} warning_msg_t;

// ----------- Đường dẫn file .wav và priority từng loại -----------
#define WAV_DOOR             "/root/WarningSound/Door.wav"
#define WAV_SEATBELT         "/root/WarningSound/SeatBelt.wav"
#define WAV_TEMP_INC         "/root/WarningSound/Temp_Inc.wav"
#define WAV_TEMP_DEC         "/root/WarningSound/Temp_Dec.wav"
#define WAV_TIRE_PRESSURE    "/root/WarningSound/TirePressure.wav"
#define WAV_SPEED_EXCEED     "/root/WarningSound/speed_exceed.wav"
#define WAV_SPEEDLIMIT_INC   "/root/WarningSound/speedlimit_inc.wav"
#define WAV_SPEEDLIMIT_DEC   "/root/WarningSound/speedlimit_dec.wav"
#define WAV_FIXED_CHANGE     "/root/WarningSound/Fixed_change.wav"

#define DOOR_PRIO               3
#define SEATBELT_PRIO           3
#define TIRE_PRESSURE_PRIO      3
#define SPEED_EXCEED_PRIO       2
#define TEMP_INC_PRIO           1
#define TEMP_DEC_PRIO           1
#define SPEEDLIMIT_INC_PRIO     1
#define SPEEDLIMIT_DEC_PRIO     1
#define FIXED_CHANGE_PRIO       1

typedef struct {
    warning_code_t code;
    const char* wav_file;
    unsigned int prio;
    unsigned int pending;
} warning_info_t;

warning_info_t warning_table[] = {
    {WARN_DOOR,           WAV_DOOR,           DOOR_PRIO,                    0},
    {WARN_SEATBELT,       WAV_SEATBELT,       SEATBELT_PRIO,                0},
    {WARN_TIRE_PRESSURE,  WAV_TIRE_PRESSURE,  TIRE_PRESSURE_PRIO,           0},
    {WARN_SPEED_EXCEED,   WAV_SPEED_EXCEED,   SPEED_EXCEED_PRIO,            0},
    {WARN_TEMP_INC,       WAV_TEMP_INC,       TEMP_INC_PRIO,                0},
    {WARN_TEMP_DEC,       WAV_TEMP_DEC,       TEMP_DEC_PRIO,                0},
    {WARN_SPEEDLIMIT_INC, WAV_SPEEDLIMIT_INC, SPEEDLIMIT_INC_PRIO,          0},
    {WARN_SPEEDLIMIT_DEC, WAV_SPEEDLIMIT_DEC, SPEEDLIMIT_DEC_PRIO,          0},
    {WARN_FIXED_CHANGE,   WAV_FIXED_CHANGE,   FIXED_CHANGE_PRIO,            0}
};
#define WARNING_TABLE_SIZE (sizeof(warning_table)/sizeof(warning_table[0]))


const char* get_wav_by_code(warning_code_t code) {
    for (size_t i = 0; i < WARNING_TABLE_SIZE; ++i)
        if (warning_table[i].code == code)
            return warning_table[i].wav_file;
    return NULL;
}
unsigned int get_prio_by_code(warning_code_t code) {
    for (size_t i = 0; i < WARNING_TABLE_SIZE; ++i)
        if (warning_table[i].code == code)
            return warning_table[i].prio;
    return -1;
}

unsigned int get_pending_state_by_code(warning_code_t code){
    for (size_t i = 0; i < WARNING_TABLE_SIZE; ++i)
        if (warning_table[i].code == code)
            return warning_table[i].pending;
    return -1;
}

// ----------- Enqueue warning code vào mq, ưu tiên -----------
void warn_enqueue(mqd_t mq, warning_code_t code) {
    warning_msg_t msg = {.code = code};
    unsigned int prio = get_prio_by_code(code);
    if (mq_send(mq, (const char*)&msg, sizeof(msg), prio) < 0) {
        perror("mq_send");
    }
}

// ----------- Logic phát hiện cảnh báo từ trạng thái xe -----------
void check_and_warn(mqd_t mq, const DataTransfer_t* vehicle_data) {
    if(vehicle_data->vehicle_status.engine_status == 1)
    {
        if(vehicle_data->vehicle_status.transmission_gear== 1 || vehicle_data->vehicle_status.transmission_gear== 3) //1 for R, 3 for D
        {
            // Quá tốc độ
            if (vehicle_data->vehicle_status.speed > vehicle_data->control_data.speed_limit && !get_pending_state_by_code(WARN_SPEED_EXCEED)) {
                warn_enqueue(mq, WARN_SPEED_EXCEED);
                warning_table[WARN_SPEED_EXCEED-1].pending=1;
            } 
            // Cửa mở
            if (vehicle_data->vehicle_status.door_status == 0 && !get_pending_state_by_code(WARN_DOOR)) {
                warn_enqueue(mq, WARN_DOOR);
                warning_table[WARN_DOOR-1].pending=1;
            } 
            // Dây an toàn chưa cài
            if (vehicle_data->vehicle_status.seat_belt_status == 0 && !get_pending_state_by_code(WARN_SEATBELT)) {
                warn_enqueue(mq, WARN_SEATBELT);
                warning_table[WARN_SEATBELT-1].pending=1;
            } 
            //  Áp suất lốp có vấn đề
            if (vehicle_data->vehicle_status.tire_pressure == 0 && !get_pending_state_by_code(WARN_TIRE_PRESSURE)) {
                warn_enqueue(mq, WARN_TIRE_PRESSURE);
                warning_table[WARN_TIRE_PRESSURE-1].pending=1;
            } 
        }
        // Control Change
        if(vehicle_data->vehicle_status.temp_or_limit_changed!=0)
        {
            if(vehicle_data->vehicle_status.temp_or_limit_changed == 1)
            {
                warn_enqueue(mq, WARN_SPEEDLIMIT_INC);
            }
            else if(vehicle_data->vehicle_status.temp_or_limit_changed == 2)
            {
                warn_enqueue(mq, WARN_SPEEDLIMIT_DEC);
            }
            else if(vehicle_data->vehicle_status.temp_or_limit_changed == 3)
            {
                warn_enqueue(mq, WARN_TEMP_INC);
            }
            else if(vehicle_data->vehicle_status.temp_or_limit_changed == 4)
            {
                warn_enqueue(mq, WARN_TEMP_DEC);
            }
            else if(vehicle_data->vehicle_status.temp_or_limit_changed == 5)
            {
                warn_enqueue(mq, WARN_FIXED_CHANGE);
            }

        }
    }

}

// ----------- Thread worker lấy từ mq, phát file .wav -----------
void* warning_worker(void* arg) {
    mqd_t mq = *((mqd_t*)arg);
    warning_msg_t msg;
    unsigned int prio;
    while (1) {
        ssize_t n = mq_receive(mq, (char*)&msg, sizeof(msg), &prio);
        if (n == sizeof(msg)) {
            const char* wav_file = get_wav_by_code(msg.code);
            if (wav_file) {
                printf("[WarningWorker] Play: %s (prio=%u)\n", wav_file, prio);
                char cmd[256];
                snprintf(cmd, sizeof(cmd), "aplay -D hw:0,0 '%s'", wav_file);
                system(cmd); // block đúng bằng thời lượng file wav (~5s)
            }
            warning_table[msg.code-1].pending=0;
        } else {
            if (errno != EINTR)
                perror("mq_receive");
        }
    }
    return NULL;
}

// ---------------------- Main process ----------------------
int main() {
    // Khởi tạo MQ
    struct mq_attr attr = {
        .mq_flags = 0,
        .mq_maxmsg = 32,
        .mq_msgsize = sizeof(warning_msg_t),
        .mq_curmsgs = 0
    };
    mq_unlink(MQ_NAME); // Xóa cũ nếu có
    mqd_t mq = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0644, &attr);
    if (mq == (mqd_t)-1) {
        perror("mq_open");
        exit(1);
    }

    // Setup socket nhận VehicleStatus
    int sockfd;
    struct sockaddr_un addr;
    sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); exit(1); }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    unlink(SOCKET_PATH);
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(1);
    }
 
    pthread_t worker_tid;
    pthread_create(&worker_tid, NULL, warning_worker, &mq);

    while (1) {
        DataTransfer_t vehicle_data = {0};
        ssize_t n = recvfrom(sockfd, &vehicle_data, sizeof(vehicle_data), 0, NULL, NULL);
        if (n == sizeof(vehicle_data)) {
            check_and_warn(mq, &vehicle_data);
        } else {
            usleep(10000);
        }
    }

    mq_close(mq);
    mq_unlink(MQ_NAME);
    close(sockfd);
    return 0;
}
