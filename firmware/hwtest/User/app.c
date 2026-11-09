/* 命令行与人机任务。每条命令回复以 OK / ERR / 数据行开头，'#' 开头为注释行 */
#include "app.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "audio.h"
#include "bsp.h"
#include "bt_uart.h"

#define AMP_DEFAULT 0.10f /* PAM8403 约 24 dB 增益，DAC 大幅度会削波 */
#define AMP_MAX     1.0f
#define F_MIN       20.0f
#define F_MAX       (0.45f * FS_HZ)
#define FS_SINE_RMS (2047.0f / 1.41421356f)
#define MAX_ARGS    6

static const char *const sig_name[] = {"OFF", "TONE", "NOISE", "SWEEP"};
static const char *const ch_name[AUDIO_NCH] = {"ref", "err"};
static volatile uint32_t stream_ms;

/* 不依赖 printf 浮点支持的定点格式化 */
static const char *fx(char *buf, float v, int dec)
{
    static const int32_t p10[] = {1, 10, 100, 1000};
    int32_t s = p10[dec];
    int32_t iv = (int32_t)lrintf(fabsf(v) * s);
    sprintf(buf, "%s%ld.%0*ld", (v < 0 && iv) ? "-" : "", (long)(iv / s), dec, (long)(iv % s));
    return buf;
}

static bool parse_f(const char *s, float *out)
{
    float v = 0.0f, scale = 1.0f;
    bool neg = false, digits = false, frac = false;

    if (*s == '-' || *s == '+')
        neg = *s++ == '-';
    for (; *s; s++) {
        if (*s == '.' && !frac) {
            frac = true;
        } else if (isdigit((unsigned char)*s)) {
            digits = true;
            if (frac)
                v += (*s - '0') * (scale *= 0.1f);
            else
                v = v * 10.0f + (*s - '0');
        } else {
            return false;
        }
    }
    *out = neg ? -v : v;
    return digits;
}

static bool arg_f(int argc, char **argv, int i, float def, float lo, float hi, float *out)
{
    if (i >= argc) {
        *out = def;
        return true;
    }
    return parse_f(argv[i], out) && *out >= lo && *out <= hi;
}

static void print_level(void)
{
    level_t l;
    char a[16], b[16], c[16], d[16];

    audio_get_level(&l);
    if (!l.n) {
        bt_printf("ERR no data yet\r\n");
        return;
    }
    bt_lock();
    bt_printf("LVL t=%lu n=%lu", (unsigned long)l.tick, (unsigned long)l.n);
    for (int ch = 0; ch < AUDIO_NCH; ch++) {
        double dc = (double)l.sum[ch] / l.n;
        double var = (double)l.sumsq[ch] / l.n - dc * dc;
        float rms = var > 0 ? (float)sqrt(var) : 0.0f;
        float dbfs = rms > 0.01f ? 20.0f * log10f(rms / FS_SINE_RMS) : -99.0f;
        bt_printf(" | %s dc=%s rms=%s mv=%s dbfs=%s min=%u max=%u clip=%lu", ch_name[ch],
                  fx(a, (float)dc, 1), fx(b, rms, 2), fx(c, rms * 3300.0f / 4095.0f, 2),
                  fx(d, dbfs, 1), l.min[ch], l.max[ch], (unsigned long)l.clip[ch]);
    }
    bt_printf("\r\n");
    bt_unlock();
}

static void print_signal(const char *prefix)
{
    sig_cfg_t s;
    char a[16], b[16], c[16];

    audio_get_signal(&s);
    bt_printf("%s out=%s f1=%s f2=%s amp=%s", prefix, sig_name[s.mode], fx(a, s.f1, 1),
              fx(b, s.f2, 1), fx(c, s.amp, 3));
}

static void set_signal(sig_mode_t mode, float f1, float f2, float sec, float amp)
{
    sig_cfg_t s = {.mode = mode, .amp = amp, .f1 = f1, .f2 = f2, .sweep_s = sec};
    audio_set_signal(&s);
    bt_lock();
    print_signal("OK");
    bt_printf("\r\n");
    bt_unlock();
}

static void cmd_help(int argc, char **argv);

static void cmd_ping(int argc, char **argv)
{
    bt_printf("PONG %s %lu\r\n", argc > 1 ? argv[1] : "-", (unsigned long)xTaskGetTickCount());
}

static void cmd_info(int argc, char **argv)
{
    audio_diag_t dg;
    char up[16];
    (void)argc;
    (void)argv;

    audio_get_diag(&dg);
    bt_lock();
    bt_printf("INFO fw=%s sysclk=%lu clk_ok=%d fs=%d baud=%d up=%s", FW_NAME,
              (unsigned long)SystemCoreClock, bsp_clock_ok(), FS_HZ, BT_BAUD,
              fx(up, xTaskGetTickCount() / 1000.0f, 1));
    print_signal("");
    bt_printf(" active=%d blocks=%lu late=%lu ovr=%lu udr=%lu rxovf=%lu heap=%u\r\n",
              audio_output_active(), (unsigned long)dg.blocks, (unsigned long)dg.late_blocks,
              (unsigned long)dg.adc_ovr, (unsigned long)dg.dac_udr,
              (unsigned long)bt_rx_overflow(), (unsigned)xPortGetFreeHeapSize());
    bt_unlock();
}

static void cmd_tone(int argc, char **argv)
{
    float f, amp;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, F_MIN, F_MAX, &f) ||
        !arg_f(argc, argv, 2, AMP_DEFAULT, 0, AMP_MAX, &amp)) {
        bt_printf("ERR usage: TONE <f_hz 20..%d> [amp 0..1]\r\n", (int)F_MAX);
        return;
    }
    set_signal(SIG_TONE, f, f, 0, amp);
}

static void cmd_noise(int argc, char **argv)
{
    float amp;
    if (!arg_f(argc, argv, 1, AMP_DEFAULT, 0, AMP_MAX, &amp)) {
        bt_printf("ERR usage: NOISE [amp 0..1]\r\n");
        return;
    }
    set_signal(SIG_NOISE, 0, 0, 0, amp);
}

static void cmd_sweep(int argc, char **argv)
{
    float f1, f2, sec, amp;
    if (argc < 4 || !arg_f(argc, argv, 1, 0, F_MIN, F_MAX, &f1) ||
        !arg_f(argc, argv, 2, 0, F_MIN, F_MAX, &f2) || f2 <= f1 ||
        !arg_f(argc, argv, 3, 0, 0.2f, 60.0f, &sec) ||
        !arg_f(argc, argv, 4, AMP_DEFAULT, 0, AMP_MAX, &amp)) {
        bt_printf("ERR usage: SWEEP <f1> <f2> <sec 0.2..60> [amp 0..1], f1<f2\r\n");
        return;
    }
    set_signal(SIG_SWEEP, f1, f2, sec, amp);
}

static void cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    set_signal(SIG_OFF, 0, 0, 0, 0);
}

static void cmd_level(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    print_level();
}

static void cmd_stream(int argc, char **argv)
{
    float ms;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, 0, 60000, &ms) || (ms > 0 && ms < 200)) {
        bt_printf("ERR usage: STREAM <ms 200..60000 | 0=off>\r\n");
        return;
    }
    stream_ms = (uint32_t)ms;
    bt_printf("OK stream=%lu\r\n", (unsigned long)stream_ms);
}

static void cmd_cap(int argc, char **argv)
{
    float f;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, 1, CAP_MAX, &f)) {
        bt_printf("ERR usage: CAP <frames 1..%d>\r\n", CAP_MAX);
        return;
    }
    uint32_t n = (uint32_t)f;
    const uint16_t *p = audio_capture(n, n * 1000u / FS_HZ + 1000u);
    if (!p) {
        bt_printf("ERR capture timeout\r\n");
        return;
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n * AUDIO_NCH; i++)
        sum += p[i];
    /* 二进制段：n 帧 × 2 通道 × uint16 小端，交织 ref,err */
    bt_lock();
    bt_printf("#CAP %lu %d %d\r\n", (unsigned long)n, FS_HZ, AUDIO_NCH);
    bt_write(p, n * AUDIO_NCH * sizeof(uint16_t));
    bt_printf("#END %08lX\r\n", (unsigned long)sum);
    bt_unlock();
}

static void cmd_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bt_printf("OK reset\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    NVIC_SystemReset();
}

typedef struct {
    const char *name;
    void (*fn)(int argc, char **argv);
    const char *help;
} cmd_t;

static const cmd_t cmds[] = {
    {"HELP", cmd_help, "this list"},
    {"PING", cmd_ping, "[token]  -> PONG token tick_ms"},
    {"INFO", cmd_info, "firmware, clock, output and error counters"},
    {"TONE", cmd_tone, "<f_hz> [amp]  sine to speaker"},
    {"NOISE", cmd_noise, "[amp]  white noise to speaker"},
    {"SWEEP", cmd_sweep, "<f1> <f2> <sec> [amp]  log sweep, repeats"},
    {"STOP", cmd_stop, "speaker off (DAC mid, 20 ms fade)"},
    {"LEVEL", cmd_level, "last 0.5 s mic stats: dc, rms, mV, dBFS, min, max, clip"},
    {"STREAM", cmd_stream, "<ms|0>  print LEVEL periodically"},
    {"CAP", cmd_cap, "<frames>  capture both mics, binary dump"},
    {"RESET", cmd_reset, "reboot MCU"},
};

static void cmd_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bt_lock();
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++)
        bt_printf("# %-6s %s\r\n", cmds[i].name, cmds[i].help);
    bt_printf("OK\r\n");
    bt_unlock();
}

static void handle_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;

    for (char *p = strtok(line, " \t"); p && argc < MAX_ARGS; p = strtok(NULL, " \t"))
        argv[argc++] = p;
    if (!argc)
        return;
    for (char *p = argv[0]; *p; p++)
        *p = (char)toupper((unsigned char)*p);
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        if (!strcmp(argv[0], cmds[i].name)) {
            cmds[i].fn(argc, argv);
            return;
        }
    }
    bt_printf("ERR unknown command '%s', type HELP\r\n", argv[0]);
}

static void cli_task(void *arg)
{
    static char line[96];
    (void)arg;

    bt_printf("\r\n# %s  STM32F407VET6  sysclk=%luHz fs=%d baud=%d\r\n", FW_NAME,
              (unsigned long)SystemCoreClock, FS_HZ, BT_BAUD);
    if (!bsp_clock_ok())
        bt_printf("# WARNING: HSE/PLL not running, check 8 MHz crystal\r\n");
    bt_printf("# type HELP\r\n");
    for (;;) {
        if (bt_getline(line, sizeof line, BT_WAIT_FOREVER) > 0)
            handle_line(line);
    }
}

static void toggle_signal(sig_mode_t mode, const char *key)
{
    sig_cfg_t s;
    audio_get_signal(&s);
    bt_printf("KEY %s\r\n", key);
    if (s.mode == mode)
        set_signal(SIG_OFF, 0, 0, 0, 0);
    else
        set_signal(mode, 200.0f, 200.0f, 0, AMP_DEFAULT);
}

static void house_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t t = 0, stream_t = 0;
    const uint32_t blink = bsp_clock_ok() ? 50 : 8; /* 500 ms 正常，80 ms 表示时钟异常 */
    (void)arg;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
        if (++t % blink == 0)
            led_toggle(LED_D2);
        led_set(LED_D3, audio_output_active());

        switch (key_poll()) {
        case KEY_K0: toggle_signal(SIG_TONE, "K0 tone 200 Hz"); break;
        case KEY_K1: bt_printf("KEY K1 stop\r\n"); set_signal(SIG_OFF, 0, 0, 0, 0); break;
        case KEY_UP: toggle_signal(SIG_NOISE, "K_UP noise"); break;
        default: break;
        }

        stream_t += 10;
        if (stream_ms && stream_t >= stream_ms) {
            stream_t = 0;
            print_level();
        }
    }
}

void app_start(void)
{
    BaseType_t ok = xTaskCreate(cli_task, "cli", 768, NULL, 3, NULL);
    ok &= xTaskCreate(house_task, "house", 512, NULL, 2, NULL);
    configASSERT(ok == pdPASS);
}
