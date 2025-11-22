#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pocketsphinx/pocketsphinx.h>
#include <sphinxbase/err.h>
#include <alsa/asoundlib.h>
#include <stdint.h>
#include <limits.h>
#include <sys/un.h>
#include <sys/socket.h>

/* ==== ĐƯỜNG DẪN MẶC ĐỊNH ==== */
#define DEFAULT_HMM   "/usr/share/pocketsphinx/model/en-us/en-us"
#define DEFAULT_DICT  "/usr/share/pocketsphinx/model/en-us/cmudict-en-us.dict"
#define DEFAULT_KWS   "/root/SpeechRecognitionProcess/keywords.txt"

#define SR_CMD_PATH "/tmp/sr_gateway"

/* ==== COMMAND ==== */
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

/* ==== UNIX SOCKET ==== */
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
    (void)sendto(sr_sock, &code, 1, 0,
        (struct sockaddr*)&sr_dst, sizeof(sr_dst));
}

/* ==== MAP KEYWORD → CMD ==== */
static uint8_t map_cmd(const char* w) {
    if (!w) return 0;
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
    return 0;
}

/* ==== Software mic gain ==== */
static void apply_gain_s16(int16_t *buf, size_t n, float gain) {
    if (gain <= 1.0001f) return;
    for (size_t i = 0; i < n; ++i) {
        float s = buf[i] * gain;
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        buf[i] = (int16_t)s;
    }
}

/* ==== ALSA 48k CAPTURE ==== */
typedef struct {
    snd_pcm_t *pcm;
    unsigned rate;
    int channels;
} alsa48_t;

static int alsa48_open(alsa48_t *a, const char *dev) {
    int rc;
    snd_pcm_hw_params_t *params;

    a->rate = 48000;
    a->channels = 1;

    if ((rc = snd_pcm_open(&a->pcm, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0)
        return rc;

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(a->pcm, params);
    snd_pcm_hw_params_set_access(a->pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(a->pcm, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(a->pcm, params, a->channels);

    unsigned r = a->rate; int dir = 0;
    snd_pcm_hw_params_set_rate_near(a->pcm, params, &r, &dir);

    snd_pcm_hw_params_set_period_size_near(a->pcm, params,
        (snd_pcm_uframes_t[]){480}, &dir);

    if ((rc = snd_pcm_hw_params(a->pcm, params)) < 0) {
        snd_pcm_close(a->pcm);
        a->pcm = NULL;
        return rc;
    }

    snd_pcm_prepare(a->pcm);
    return 0;
}

static void alsa48_close(alsa48_t *a) {
    if (a->pcm) {
        snd_pcm_drain(a->pcm);
        snd_pcm_close(a->pcm);
        a->pcm = NULL;
    }
}

/* ==== FIR 7 tap → decimate by 3 ==== */
static size_t downsample48to16(const int16_t *in, size_t n_in, int16_t *out) {
    static const int k[7] = {1,2,3,4,3,2,1};
    const int norm = 16;

    if (n_in < 7) return 0;
    size_t produced = 0;

    for (size_t i = 3; i + 3 < n_in; i += 3) {
        int32_t acc =
            k[0]*in[i-3] + k[1]*in[i-2] + k[2]*in[i-1] +
            k[3]*in[i]   +
            k[4]*in[i+1] + k[5]*in[i+2] + k[6]*in[i+3];

        int32_t y = acc / norm;
        if (y > 32767) y = 32767;
        if (y < -32768) y = -32768;
        out[produced++] = (int16_t)y;
    }
    return produced;
}

/* ==== timestamp ==== */
static long long now_ms(void) {
    struct timeval tv; gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}

/* ==== Pick best hypothesis ==== */
static void pick_best_command(ps_decoder_t *ps, char *out, size_t outsz, int32 *best_prob) {
    out[0] = 0;
    if (best_prob) *best_prob = INT_MIN;

    for (ps_seg_t *seg = ps_seg_iter(ps); seg; seg = ps_seg_next(seg)) {
        const char *w = ps_seg_word(seg);
        int32 prob = ps_seg_prob(seg, NULL, NULL, NULL);

        if (w && *w && (prob > *best_prob)) {
            strncpy(out, w, outsz-1);
            out[outsz-1] = 0;
            *best_prob = prob;
        }
    }
}

/* ==== Main ==== */

static volatile int g_keep = 1;
static void on_sigint(int _) { (void)_; g_keep = 0; }

int main(int argc, char **argv) {

    const char *hmm = DEFAULT_HMM, *dict = DEFAULT_DICT, *kws = DEFAULT_KWS;
    const char *alsa_dev = "hw:0,0";
    const char *kws_th = "1e-20";
    float gain = 1.0f;

    int opt;
    while ((opt = getopt(argc, argv, "k:d:t:g:")) != -1) {
        if (opt == 'k') kws = optarg;
        else if (opt == 'd') alsa_dev = optarg;
        else if (opt == 't') kws_th = optarg;
        else if (opt == 'g') gain = strtof(optarg, NULL);
    }

    if (sr_init_sender() < 0)
        printf("WARNING: cannot init UNIX socket\n");

    /* ==== POCKETSPHINX ==== */
    cmd_ln_t *config = cmd_ln_init(NULL, ps_args(), TRUE,
        "-hmm", hmm,
        "-dict", dict,
        "-kws",  kws,
        "-kws_threshold", kws_th,
        "-samprate", "16000",
        "-cmn", "live",
        "-agc", "noise",
        "-remove_noise", "yes",
        "-remove_silence", "yes",
        NULL);

    ps_decoder_t *ps = ps_init(config);
    if (!ps) return 1;

    /* ==== ALSA INIT ==== */
    alsa48_t a48;
    if (alsa48_open(&a48, alsa_dev) != 0) {
        fprintf(stderr,"Failed to open ALSA device %s\n", alsa_dev);
        return 1;
    }

    signal(SIGINT, on_sigint);
    ps_start_utt(ps);

    printf("Listening on ALSA %s @48k -> 16k...\n", alsa_dev);

    const size_t N48 = 4800;
    int16_t *buf48 = malloc(N48 * sizeof(int16_t));
    int16_t *buf16 = malloc((N48/3 + 16) * sizeof(int16_t));

    int32 in_speech = 0, was_in_speech = 0;
    char last_cmd[128] = {0};
    long long last_emit_ms = 0;

    while (g_keep) {

        snd_pcm_sframes_t got = snd_pcm_readi(a48.pcm, buf48, N48);
        if (got < 0) { snd_pcm_prepare(a48.pcm); continue; }

        size_t n16 = downsample48to16(buf48, (size_t)got, buf16);
        if (n16 == 0) continue;

        apply_gain_s16(buf16, n16, gain);
        ps_process_raw(ps, buf16, n16, FALSE, FALSE);

        in_speech = ps_get_in_speech(ps);

        if (!in_speech && was_in_speech) {
            if (ps_end_utt(ps) == 0) {

                char best[128];
                int32 best_prob;
                pick_best_command(ps, best, sizeof(best), &best_prob);

                if (best[0]) {
                    long long t = now_ms();
                    if (!(strcmp(best, last_cmd)==0 &&
                          (t - last_emit_ms) < 700)) {

                        printf("[DETECTED] %s (prob=%d)\n", best, best_prob);

                        uint8_t code = map_cmd(best);
                        if (code != 0) sr_send_cmd(code);

                        strncpy(last_cmd, best, sizeof(last_cmd)-1);
                        last_emit_ms = t;
                    }
                }

                ps_start_utt(ps);
            }
        }

        was_in_speech = in_speech;
        usleep(10000);
    }

    alsa48_close(&a48);
    free(buf48); free(buf16);
    ps_free(ps); cmd_ln_free_r(config);
    printf("Bye.\n");
    return 0;
}
