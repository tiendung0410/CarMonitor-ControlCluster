#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pocketsphinx.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/err.h>
#include <alsa/asoundlib.h>
#include <stdint.h>
#include <limits.h>
#include <sys/un.h>
#include <sys/socket.h>

/* ==== ĐƯỜNG DẪN MẶC ĐỊNH (sửa nếu máy bạn khác) ==== */
#define DEFAULT_HMM   "/usr/share/pocketsphinx/model/en-us/en-us"
#define DEFAULT_DICT  "/usr/share/pocketsphinx/model/en-us/cmudict-en-us.dict"
#define DEFAULT_KWS   "/root/SpeechRecognitionProcess/keywords.txt"

#define SR_CMD_PATH "/tmp/sr_gateway"  // Gateway sẽ bind vào đây

#define CMD_ENGINE_ON             0x01
#define CMD_ENGINE_OFF            0x02
#define CMD_LOW_BEAM_ON           0x03
#define CMD_HIGH_BEAM_ON          0x04
#define CMD_MIST_BEAM_ON          0x05
#define CMD_LIGHT_OFF             0x06
#define CMD_DOOR_OPEN             0x07
#define CMD_DOOR_CLOSE            0x08
#define CMD_AC_INCREASE           0x09
#define CMD_AC_DECREASE           0x0A

static int sr_sock = -1;
static struct sockaddr_un sr_dst;

static int sr_init_sender(void) {
    sr_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sr_sock < 0) { perror("sr socket"); return -1; }
    memset(&sr_dst, 0, sizeof(sr_dst));
    sr_dst.sun_family = AF_UNIX;
    strncpy(sr_dst.sun_path, SR_CMD_PATH, sizeof(sr_dst.sun_path)-1);
    return 0;
}

static void sr_send_cmd(uint8_t code) {
    (void)sendto(sr_sock, &code, 1, 0, (struct sockaddr*)&sr_dst, sizeof(sr_dst));
}

/* Map keyword -> 1 byte command (tùy bạn khớp với keywords.txt)
   0x00 = không hợp lệ, bỏ qua */
static uint8_t map_cmd(const char* w) {
    if (!w) return 0x00;
    /* ví dụ: các câu tiếng Anh/Việt khớp với keywords.txt của bạn */
    if (!strcasecmp(w, "engine on"))              return CMD_ENGINE_ON;
    if (!strcasecmp(w, "engine off"))             return CMD_ENGINE_OFF;

    if (!strcasecmp(w, "low beam on"))            return CMD_LOW_BEAM_ON;
    if (!strcasecmp(w, "high beam on"))           return CMD_HIGH_BEAM_ON;
    if (!strcasecmp(w, "mist beam on"))           return CMD_MIST_BEAM_ON;
    if (!strcasecmp(w, "light off"))              return CMD_LIGHT_OFF;

    if (!strcasecmp(w, "open door"))              return CMD_DOOR_OPEN;
    if (!strcasecmp(w, "close door"))             return CMD_DOOR_CLOSE;

    if (!strcasecmp(w, "increase air condition")) return CMD_AC_INCREASE;
    if (!strcasecmp(w, "reduce air condition"))   return CMD_AC_DECREASE;
    return 0x00;
}


/* Thứ tự ưu tiên ALSA device thử ở 16 kHz (dùng wrapper ad_open_dev) */
static const char* kAlsa16k[] = { "usb_mic_16k", "plughw:1,0", "default", "hw:1,0" };
static const int   kNum16k    = 4;

static volatile int g_keep = 1;
static void on_sigint(int _) { (void)_; g_keep = 0; }

/* ==== Software mic gain, chống clipping ==== */
static void apply_gain_s16(int16_t *buf, size_t n, float gain) {
    if (gain <= 1.0001f) return;
    for (size_t i = 0; i < n; ++i) {
        float s = (float)buf[i] * gain;
        if (s > 32767.0f) s = 32767.0f;
        else if (s < -32768.0f) s = -32768.0f;
        buf[i] = (int16_t)s;
    }
}

/* ==== Fallback: mở ALSA trực tiếp ở 48 kHz (hw) ==== */
typedef struct {
    snd_pcm_t *pcm;
    unsigned rate;
    int channels;
    snd_pcm_format_t fmt;
} alsa48_t;

static int alsa48_open(alsa48_t *a, const char *dev) {
    int rc;
    snd_pcm_hw_params_t *params;
    a->pcm = NULL;
    a->rate = 48000;
    a->channels = 1;
    a->fmt = SND_PCM_FORMAT_S16_LE;

    if ((rc = snd_pcm_open(&a->pcm, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) return rc;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(a->pcm, params);
    snd_pcm_hw_params_set_access(a->pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(a->pcm, params, a->fmt);
    snd_pcm_hw_params_set_channels(a->pcm, params, a->channels);
    unsigned r = a->rate; int dir = 0;
    snd_pcm_hw_params_set_rate_near(a->pcm, params, &r, &dir);
    if (r != a->rate) { snd_pcm_close(a->pcm); a->pcm = NULL; return -EINVAL; }
    /* period ~10ms (480 frames) */
    snd_pcm_hw_params_set_period_size_near(a->pcm, params, (snd_pcm_uframes_t[]){480}, &dir);
    if ((rc = snd_pcm_hw_params(a->pcm, params)) < 0) {
        snd_pcm_close(a->pcm); a->pcm = NULL; return rc;
    }
    snd_pcm_prepare(a->pcm);
    return 0;
}
static void alsa48_close(alsa48_t *a) {
    if (a->pcm) { snd_pcm_drain(a->pcm); snd_pcm_close(a->pcm); a->pcm = NULL; }
}

/* ==== FIR low-pass 7-tap rồi decimate by 3 (48k -> 16k) ==== */
static size_t downsample48to16(const int16_t *in, size_t n_in, int16_t *out) {
    static const int k[7] = {1,2,3,4,3,2,1};
    const int norm = 16;
    if (n_in < 7) return 0;
    size_t produced = 0;
    for (size_t i = 3; i + 3 < n_in; i += 3) {
        int32_t acc = 0;
        acc += k[0] * in[i-3];
        acc += k[1] * in[i-2];
        acc += k[2] * in[i-1];
        acc += k[3] * in[i  ];
        acc += k[4] * in[i+1];
        acc += k[5] * in[i+2];
        acc += k[6] * in[i+3];
        int32_t y = acc / norm;
        if (y > 32767) y = 32767; else if (y < -32768) y = -32768;
        out[produced++] = (int16_t)y;
    }
    return produced;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [-k <kws_file>] [-d <alsa_dev>] [-t <kws_threshold>] [-g <gain>] [--hmm PATH] [--dict PATH]\n"
        "Defaults:\n"
        "  -hmm  %s\n"
        "  -dict %s\n"
        "  -k    %s\n"
        "  -d    (auto: usb_mic_16k, plughw:1,0, default, hw:1,0)\n"
        "  -t    1e-20\n"
        "  -g    1.0\n", prog, DEFAULT_HMM, DEFAULT_DICT, DEFAULT_KWS);
}

/* millisecond timestamp */
static long long now_ms(void) {
    struct timeval tv; gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);
}

/* ==== HÀM CHỌN LỆNH TỐT NHẤT (C thuần, top-level) ==== */
static void pick_best_command(ps_decoder_t *ps, char *out, size_t outsz, int32 *best_prob) {
    out[0] = '\0';
    if (best_prob) *best_prob = INT_MIN;
    for (ps_seg_t *seg = ps_seg_iter(ps); seg; seg = ps_seg_next(seg)) {
        const char *w = ps_seg_word(seg);
        int32 prob = ps_seg_prob(seg, NULL, NULL, NULL); /* log-prob (âm, càng lớn càng tốt) */
        if (w && *w) {
            if (!out[0] || prob > (best_prob ? *best_prob : INT_MIN)) {
                strncpy(out, w, outsz - 1);
                out[outsz - 1] = '\0';
                if (best_prob) *best_prob = prob;
            }
        }
    }
}

int main(int argc, char **argv) {

    const char *hmm = DEFAULT_HMM, *dict = DEFAULT_DICT, *kws = DEFAULT_KWS;
    const char *force_dev = NULL, *kws_th = "1e-20";
    float in_gain = 1.0f;

    int opt;
    while ((opt = getopt(argc, argv, "k:d:t:g:")) != -1) {
        if (opt == 'k') kws = optarg;
        else if (opt == 'd') force_dev = optarg;
        else if (opt == 't') kws_th = optarg;
        else if (opt == 'g') in_gain = strtof(optarg, NULL);
        else { print_usage(argv[0]); return 1; }
    }
    /* Parse --hmm / --dict (option kiểu dài) */
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--hmm")  && i+1 < argc) { hmm  = argv[++i]; }
        else if (!strcmp(argv[i], "--dict") && i+1 < argc) { dict = argv[++i]; }
    }

    /* Khoi tao unix socket*/
    if(sr_init_sender() <0)\
    {
        printf("Failed to init unix socket!\n");
    }

    /* 1) CẤU HÌNH POCKETSPHINX */
    cmd_ln_t *config = cmd_ln_init(NULL, ps_args(), TRUE,
        "-hmm", hmm,
        "-dict", dict,
        "-kws",  kws,
        "-kws_threshold", kws_th,
        "-samprate", "16000",
        /* Xử lý tín hiệu để ổn định */
        "-cmn", "live",
        "-agc", "noise",
        "-remove_noise", "yes",
        "-remove_silence", "yes",
        /* VAD khắt khe hơn để cắt câu rõ hơn */
        "-vad_prespeech",  "20",
        "-vad_postspeech", "50",
        "-vad_threshold",  "3.0",
        NULL);
    if (!config) { fprintf(stderr,"Failed to create config\n"); return 1; }

    printf("HMM : %s\nDICT: %s\nKWS : %s\nTH  : %s\nGAIN: %.2f\n", hmm, dict, kws, kws_th, in_gain);

    ps_decoder_t *ps = ps_init(config);
    if (!ps) { fprintf(stderr,"ps_init failed\n"); cmd_ln_free_r(config); return 1; }

    /* 2) THỬ MỞ AUDIO 16 kHz BẰNG WRAPPER SPHINX */
    ad_rec_t *ad = NULL;
    if (force_dev) {
        printf("Trying ALSA (forced) %s @16k\n", force_dev);
        ad = ad_open_dev(force_dev, 16000);
        if (!ad) fprintf(stderr,"Failed to open forced device: %s\n", force_dev);
    }
    for (int i = 0; !ad && i < kNum16k; ++i) {
        printf("Trying ALSA %s @16k\n", kAlsa16k[i]);
        ad = ad_open_dev(kAlsa16k[i], 16000);
    }

    /* 3) FALLBACK: 48 kHz + FIR + decimate -> 16 kHz */
    int use_fallback48 = 0;
    alsa48_t a48 = {0};
    if (!ad) {
        printf("Falling back to ALSA hw:1,0 @48k with FIR downsample -> 16k\n");
        if (alsa48_open(&a48, "hw:1,0") == 0) use_fallback48 = 1;
        else { fprintf(stderr,"ERROR: Cannot open hw:1,0 @48k\n"); ps_free(ps); cmd_ln_free_r(config); return 1; }
    }

    signal(SIGINT, on_sigint);

    if (ps_start_utt(ps) < 0) {
        fprintf(stderr,"ps_start_utt failed\n");
        if (use_fallback48) alsa48_close(&a48);
        ps_free(ps); cmd_ln_free_r(config); return 1;
    }
    printf("Listening... (Ctrl+C to quit)\n");

    int32 in_speech = 0, was_in_speech = 0;

    /* Debounce để không spam cùng 1 lệnh quá nhanh */
    static char last_cmd[128] = {0};
    long long last_emit_ms = 0;
    const long long debounce_ms = 700; /* 0.7s */

    if (!use_fallback48) {
        if (ad_start_rec(ad) < 0) { fprintf(stderr,"start_rec failed\n"); ad_close(ad); ps_free(ps); cmd_ln_free_r(config); return 1; }
        int16 buf[2048];

        while (g_keep) {
            int32 k = ad_read(ad, buf, 2048);
            if (k <= 0) { usleep(10000); continue; }
            apply_gain_s16(buf, (size_t)k, in_gain);
            ps_process_raw(ps, buf, k, FALSE, FALSE);

            in_speech = ps_get_in_speech(ps);
            if (!in_speech && was_in_speech) {
                if (ps_end_utt(ps) == 0) {
                    char best_cmd[128]; int32 best_prob;
                    pick_best_command(ps, best_cmd, sizeof(best_cmd), &best_prob);

                    if (best_cmd[0]) {
                        long long t = now_ms();
                        if (!(strcmp(best_cmd, last_cmd) == 0 && (t - last_emit_ms) < debounce_ms)) {
                            printf("[DETECTED] \"%s\" (prob=%d)\n", best_cmd, best_prob);
                            printf("{\"cmd\":\"%s\",\"prob\":%d}\n", best_cmd, best_prob);
                            fflush(stdout);
                            /* >>> GỬI 1 BYTE COMMAND SANG GATEWAY <<< */
                            uint8_t code = map_cmd(best_cmd);
                            if (code != 0x00) sr_send_cmd(code);
                            strncpy(last_cmd, best_cmd, sizeof(last_cmd)-1);
                            last_cmd[sizeof(last_cmd)-1] = '\0';
                            last_emit_ms = t;
                        }
                    }
                    ps_start_utt(ps);
                }
            }
            was_in_speech = in_speech;
            usleep(10000);
        }
        ad_stop_rec(ad); ad_close(ad);

    } else {
        const size_t N48 = 4800; /* ~100ms */
        int16_t *buf48 = (int16_t*)malloc(N48 * sizeof(int16_t));
        int16_t *buf16 = (int16_t*)malloc((N48/3 + 8) * sizeof(int16_t));
        if (!buf48 || !buf16) {
            fprintf(stderr,"OOM\n");
            if (buf48) free(buf48);
            if (buf16) free(buf16);
            alsa48_close(&a48); ps_free(ps); cmd_ln_free_r(config); return 1;
        }

        while (g_keep) {
            snd_pcm_sframes_t got = snd_pcm_readi(a48.pcm, buf48, N48);
            if (got < 0) { snd_pcm_prepare(a48.pcm); continue; }

            size_t n16 = downsample48to16(buf48, (size_t)got, buf16);
            if (n16 == 0) continue;

            apply_gain_s16(buf16, n16, in_gain);
            ps_process_raw(ps, buf16, (int)n16, FALSE, FALSE);

            in_speech = ps_get_in_speech(ps);
            if (!in_speech && was_in_speech) {
                if (ps_end_utt(ps) == 0) {
                    char best_cmd[128]; int32 best_prob;
                    pick_best_command(ps, best_cmd, sizeof(best_cmd), &best_prob);

                    if (best_cmd[0]) {
                        long long t = now_ms();
                        if (!(strcmp(best_cmd, last_cmd) == 0 && (t - last_emit_ms) < debounce_ms)) {
                            printf("[DETECTED] \"%s\" (prob=%d)\n", best_cmd, best_prob);
                            printf("{\"cmd\":\"%s\",\"prob\":%d}\n", best_cmd, best_prob);
                            fflush(stdout);
                            /* >>> GỬI 1 BYTE COMMAND SANG GATEWAY <<< */
                            uint8_t code = map_cmd(best_cmd);
                            if (code != 0x00) sr_send_cmd(code);
                            strncpy(last_cmd, best_cmd, sizeof(last_cmd)-1);
                            last_cmd[sizeof(last_cmd)-1] = '\0';
                            last_emit_ms = t;
                        }
                    }
                    ps_start_utt(ps);
                }
            }
            was_in_speech = in_speech;
            usleep(10000);
        }
        free(buf48); free(buf16);
        alsa48_close(&a48);
    }

    ps_free(ps);
    cmd_ln_free_r(config);
    printf("Bye.\n");
    return 0;
}
