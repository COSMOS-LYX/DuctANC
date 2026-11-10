/* 命令行与人机任务。每条命令回复以 OK / ERR / 数据行开头，'#' 开头为注释行 */
#include "app.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "anc.h"
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
    static const int32_t p10[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
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
    if (!audio_set_signal(&s)) {
        bt_printf(mode == SIG_OFF ? "OK out=OFF anc=1\r\n" : "ERR anc on\r\n");
        return;
    }
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
    bt_printf("INFO fw=%s sysclk=%lu clk_ok=%d fs=%d baud=%lu bt=%s up=%s", FW_NAME,
              (unsigned long)SystemCoreClock, bsp_clock_ok(), FS_HZ, (unsigned long)bt_baud(),
              bt_jdy_status(), fx(up, xTaskGetTickCount() / 1000.0f, 1));
    print_signal("");
    bt_printf(" active=%d blocks=%lu late=%lu ovr=%lu udr=%lu slip=%lu rxovf=%lu heap=%u\r\n",
              audio_output_active(), (unsigned long)dg.blocks, (unsigned long)dg.late_blocks,
              (unsigned long)dg.adc_ovr, (unsigned long)dg.dac_udr, (unsigned long)dg.slip,
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

static void dump_cap(const uint16_t *p, uint32_t n, uint32_t nch);

static void cmd_cap(int argc, char **argv)
{
    float f;
    bool dac = argc > 2 && toupper((unsigned char)argv[2][0]) == 'D' && !argv[2][1];
    if (argc < 2 || argc > 3 || (argc == 3 && !dac) ||
        !arg_f(argc, argv, 1, 0, 1, dac ? CAP_MAX_DAC : CAP_MAX, &f)) {
        bt_printf("ERR usage: CAP <frames 1..%d> | CAP <frames 1..%d> D\r\n", CAP_MAX, CAP_MAX_DAC);
        return;
    }
    uint32_t n = (uint32_t)f, nch = AUDIO_NCH + (dac ? 1 : 0);
    const uint16_t *p = audio_capture(n, dac, n * 1000u / FS_HZ + 1000u);
    if (!p) {
        bt_printf("ERR capture timeout\r\n");
        return;
    }
    dump_cap(p, n, nch);
}

static void dump_cap(const uint16_t *p, uint32_t n, uint32_t nch)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n * nch; i++)
        sum += p[i];
    /* 二进制段：n 帧 × nch 通道 × uint16 小端，交织 ref,err[,dac] */
    bt_lock();
    bt_printf("#CAP %lu %d %lu\r\n", (unsigned long)n, FS_HZ, (unsigned long)nch);
    bt_write(p, n * nch * sizeof(uint16_t));
    bt_printf("#END %08lX\r\n", (unsigned long)sum);
    bt_unlock();
}

static void cmd_carm(int argc, char **argv)
{
    float f;
    bool dac = argc > 2 && toupper((unsigned char)argv[2][0]) == 'D' && !argv[2][1];
    if (argc < 2 || argc > 3 || (argc == 3 && !dac) ||
        !arg_f(argc, argv, 1, 0, 1, dac ? CAP_MAX_DAC : CAP_MAX, &f)) {
        bt_printf("ERR usage: CARM <frames> [D]\r\n");
        return;
    }
    if (!audio_capture_arm((uint32_t)f, dac)) {
        bt_printf("ERR carm busy\r\n");
        return;
    }
    bt_printf("OK carm %d\r\n", (int)f);
}

static void cmd_cget(int argc, char **argv)
{
    uint32_t n = 0;
    bool dac = false;
    const uint16_t *p;
    (void)argc;
    (void)argv;
    p = audio_capture_take(&n, &dac);
    if (!p) {
        bt_printf("ERR cap busy\r\n");
        return;
    }
    dump_cap(p, n, AUDIO_NCH + (dac ? 1 : 0));
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

/* hex 是小端字节序的连续 8*n 个字符，和 Python float32.tobytes().hex() 一致。 */
static int parse_hex_f32(const char *hex, float *out, int n)
{
    if (strlen(hex) != (size_t)n * 8)
        return 0;
    for (int i = 0; i < n; i++) {
        uint8_t raw[4];
        for (int b = 0; b < 4; b++) {
            int hi = hex_nibble(hex[i * 8 + b * 2]);
            int lo = hex_nibble(hex[i * 8 + b * 2 + 1]);
            if (hi < 0 || lo < 0)
                return 0;
            raw[b] = (uint8_t)((hi << 4) | lo);
        }
        memcpy(&out[i], raw, sizeof out[i]);
    }
    return 1;
}

static void print_anc(const char *prefix, const audio_anc_stat_t *s)
{
    char a[16], b[16], c[16], d[16], e[16], f[16], g[16], h[16], p[16], q[16];
    bt_printf("%s on=%d prot=%d shat=%d settle=%d mu=%s gamma=%s clamp=%s "
              "wnorm=%s yrms=%s err_rms=%s e200=%s r200=%s baseline=%s "
              "clamp_n=%lu cpu=%s lag_us=%lu slip=%lu late=%lu ovr=%lu udr=%lu rxovf=%lu samples=%lu\r\n",
              prefix, s->on, s->prot, s->shat, s->settle, fx(a, s->mu, 4), fx(b, s->gamma, 6),
              fx(c, s->clamp, 0), fx(d, s->wnorm, 4), fx(e, s->y_rms, 2), fx(f, s->err_rms, 2),
              fx(g, s->e200, 2), fx(h, s->r200, 2), fx(p, s->baseline, 2),
              (unsigned long)s->clamp_n, fx(q, s->cpu_pct, 1), (unsigned long)s->lag_us,
              (unsigned long)s->slip, (unsigned long)s->late, (unsigned long)s->ovr,
              (unsigned long)s->udr, (unsigned long)bt_rx_overflow(), (unsigned long)s->samples);
}

static void cmd_shat(int argc, char **argv)
{
    if (argc >= 2) {
        for (char *p = argv[1]; *p; p++)
            *p = (char)toupper((unsigned char)*p);
    }
    if (argc == 2 && !strcmp(argv[1], "RST")) {
        if (!audio_anc_shat_reset()) {
            bt_printf("ERR shat anc on\r\n");
            return;
        }
        bt_printf("OK shat rst\r\n");
        return;
    }
    if (argc == 2 && !strcmp(argv[1], "CRC")) {
        bt_printf("OK shat crc=%08lX ready=%d\r\n", (unsigned long)audio_anc_shat_crc(),
                  anc_shat_ready(anc_state()));
        return;
    }
    float idx = 0.0f, v[8];
    if (argc != 3 || !parse_f(argv[1], &idx) || !parse_hex_f32(argv[2], v, 8)) {
        bt_printf("ERR usage: SHAT RST | SHAT CRC | SHAT <index> <64hex>\r\n");
        return;
    }
    if (!audio_anc_shat_chunk((int)idx, v, 8)) {
        bt_printf("ERR shat rejected\r\n");
        return;
    }
    bt_printf("OK shat index=%d ready=%d crc=%08lX\r\n", (int)idx, anc_shat_ready(anc_state()),
              (unsigned long)audio_anc_shat_crc());
}

static void cmd_anc(int argc, char **argv)
{
    char sub[12] = {0};
    if (argc < 2) {
        bt_printf("ERR usage: ANC ON | ANC OFF | ANC STATUS\r\n");
        return;
    }
    for (int i = 0; argv[1][i] && i < (int)sizeof sub - 1; i++)
        sub[i] = (char)toupper((unsigned char)argv[1][i]);
    if (!strcmp(sub, "ON")) {
        const char *why = NULL;
        if (!audio_anc_on(&why)) {
            bt_printf("ERR anc %s\r\n", why ? why : "fail");
            return;
        }
        audio_anc_stat_t s;
        audio_anc_stat(&s);
        bt_lock();
        print_anc("OK", &s);
        bt_unlock();
        return;
    }
    if (!strcmp(sub, "OFF")) {
        audio_anc_off();
        audio_anc_stat_t s;
        audio_anc_stat(&s);
        bt_lock();
        print_anc("OK", &s);
        bt_unlock();
        return;
    }
    if (!strcmp(sub, "STATUS")) {
        audio_anc_stat_t s;
        audio_anc_stat(&s);
        bt_lock();
        print_anc("ANC", &s);
        bt_unlock();
        return;
    }
    bt_printf("ERR usage: ANC ON | ANC OFF | ANC STATUS\r\n");
}

static void cmd_mu(int argc, char **argv)
{
    float v;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, 0.0f, 5.0f, &v)) {
        bt_printf("ERR usage: MU <0..5>\r\n");
        return;
    }
    audio_anc_set_mu(v);
    bt_printf("OK mu set\r\n");
}

static void cmd_gamma(int argc, char **argv)
{
    float v;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, 0.0f, 0.01f, &v)) {
        bt_printf("ERR usage: GAMMA <0..0.01>\r\n");
        return;
    }
    audio_anc_set_gamma(v);
    bt_printf("OK gamma set\r\n");
}

static void cmd_clamp(int argc, char **argv)
{
    float v;
    if (argc < 2 || !arg_f(argc, argv, 1, 0, 0.0f, 2047.0f, &v)) {
        bt_printf("ERR usage: CLAMP <codes 0..2047>\r\n");
        return;
    }
    audio_anc_set_clamp(v);
    bt_printf("OK clamp set\r\n");
}

static void cmd_xchk(int argc, char **argv)
{
    float nf;
    static uint16_t in[400];
    static float xf[400];

    if (argc != 2 || !parse_f(argv[1], &nf) || nf < 1.0f || nf > 400.0f) {
        bt_printf("ERR usage: XCHK <n 1..400>\r\n");
        return;
    }
    if (audio_anc_is_on()) {
        bt_printf("ERR xchk anc on\r\n");
        return;
    }
    if (!anc_shat_ready(anc_state())) {
        bt_printf("ERR xchk shat\r\n");
        return;
    }
    int n = (int)nf;
    bt_printf("OK xchk %d\r\n", n);
    if (bt_read(in, (size_t)n * sizeof(uint16_t), 3000) < 0) {
        bt_printf("ERR xchk timeout\r\n");
        return;
    }
    if (!audio_xchk(in, xf, n)) {
        bt_printf("ERR xchk busy\r\n");
        return;
    }
    uint32_t sum = 0;
    const uint8_t *p = (const uint8_t *)xf;
    for (size_t i = 0; i < (size_t)n * sizeof(float); i++)
        sum += p[i];
    bt_lock();
    bt_printf("#XCHK %d\r\n", n);
    bt_write(xf, (size_t)n * sizeof(float));
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
    {"CAP", cmd_cap, "<frames> [D]  capture both mics (+DAC output with D), binary dump"},
    {"CARM", cmd_carm, "<frames> [D]  start a capture and return at once"},
    {"CGET", cmd_cget, "dump the armed capture after it fills"},
    {"SHAT", cmd_shat, "RST | CRC | <index> <64hex>  load 8 float32 of S_eff"},
    {"ANC", cmd_anc, "ON | OFF | STATUS  FxNLMS, default off"},
    {"MU", cmd_mu, "<0..5>  NLMS step, default 0.002"},
    {"GAMMA", cmd_gamma, "<0..0.01>  leakage, default 0"},
    {"CLAMP", cmd_clamp, "<codes 0..2047>  output limit, default 40; 0 keeps DAC at mid"},
    {"XCHK", cmd_xchk, "<n>  replay n ref codes, return x' floats; DAC stays put"},
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
    static char line[160];
    (void)arg;

    const char *bt = bt_jdy_setup();
    bt_printf("\r\n# %s  STM32F407VET6  sysclk=%luHz fs=%d baud=%lu bt=%s\r\n", FW_NAME,
              (unsigned long)SystemCoreClock, FS_HZ, (unsigned long)bt_baud(), bt);
    if (!strcmp(bt, "noreply"))
        bt_printf("# JDY-31 no AT reply (PC already connected at boot?), assuming %d baud\r\n", BT_BAUD);
    else if (!strcmp(bt, "set_fail"))
        bt_printf("# WARNING: JDY-31 baud change not confirmed\r\n");
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
