/* 音频链路：TIM2 同时触发 ADC1（PC0/PC1）和 DAC1（PA4），双缓冲 DMA，块处理在 audio 任务里 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#define AUDIO_BLK   1                 /* 半缓冲 1 帧：逐点。8 kHz 时一拍 125 µs */
#define AUDIO_NCH   2                 /* 0 = 参考麦 PC0，1 = 误差麦 PC1 */
#define CAP_SAMPLES 32000                          /* CCM 64 KB 能放的 uint16 个数 */
#define CAP_MAX     (CAP_SAMPLES / AUDIO_NCH)       /* 只采麦克风：16000 帧 */
#define CAP_MAX_DAC (CAP_SAMPLES / (AUDIO_NCH + 1)) /* 加 DAC 通道：10666 帧 */
#define ADC_CLIP_LO 8
#define ADC_CLIP_HI 4087

typedef enum { SIG_OFF = 0, SIG_TONE, SIG_NOISE, SIG_SWEEP } sig_mode_t;

typedef struct {
    sig_mode_t mode;
    float amp;      /* 0..1，相对 DAC 满幅 ±2047 */
    float f1, f2;   /* 单音用 f1；扫频 f1→f2 */
    float sweep_s;  /* 扫频一轮时长 */
} sig_cfg_t;

typedef struct {
    uint32_t n;                 /* 窗内帧数 */
    int64_t sum[AUDIO_NCH];
    uint64_t sumsq[AUDIO_NCH];
    uint16_t min[AUDIO_NCH], max[AUDIO_NCH];
    uint32_t clip[AUDIO_NCH];
    uint32_t tick;              /* 窗结束时刻 ms */
} level_t;

typedef struct {
    uint32_t adc_ovr, dac_udr, late_blocks, blocks;
    uint32_t slip;       /* 任务晚于两个采样周期才去写 DAC 槽 */
    uint32_t lag_us_max; /* 距本次查询以来，中断到写槽的最大延迟 */
    uint32_t cpu_cyc;    /* 最近一拍 anc_step 的周期数 */
} audio_diag_t;

typedef struct {
    int on, prot, shat, settle;
    float mu, gamma, clamp, wnorm, y_rms, err_rms, e200, r200, baseline;
    uint32_t clamp_n, lag_us, slip, late, ovr, udr, samples;
    float cpu_pct;
} audio_anc_stat_t;

void audio_init(void);
void audio_start_task(void);
/* ANC 开着时拒绝，调用方要保持信号发生器不动。成功返回 1。 */
bool audio_set_signal(const sig_cfg_t *cfg);
void audio_get_signal(sig_cfg_t *cfg);
bool audio_output_active(void);
void audio_get_level(level_t *out);
void audio_get_diag(audio_diag_t *out);
/*
 * 采集 frames 帧到 CCM，阻塞到完成；返回缓冲指针。
 * 交织 ref,err；with_dac 时为 ref,err,dac，dac 是该帧 ADC 触发时刻起 DAC 正在输出的码值。
 */
const uint16_t *audio_capture(uint32_t frames, bool with_dac, uint32_t timeout_ms);
/* 只开采集、立刻返回，命令行还能处理 STATUS / ANC OFF。完成前再 arm 会失败。 */
int audio_capture_arm(uint32_t frames, bool with_dac);
/* 采满则交出缓冲并结束这次采集；还在采就返回 NULL。 */
const uint16_t *audio_capture_take(uint32_t *frames, bool *with_dac);

/* 下面这些都要求在 audio 任务已启动之后调用。失败时 err 写成短原因（可为 NULL）。 */
bool audio_anc_is_on(void);
bool audio_anc_on(const char **err);
void audio_anc_off(void);
void audio_anc_set_mu(float mu);
void audio_anc_set_gamma(float gamma);
void audio_anc_set_clamp(float clamp);
void audio_anc_stat(audio_anc_stat_t *out); /* 读完把 lag 峰值清掉，方便夹住一次 CAP */
int audio_anc_shat_chunk(int index, const float *v, int n);
int audio_anc_shat_reset(void); /* ANC 开着时拒绝 */
uint32_t audio_anc_shat_crc(void);
/* ANC 必须关着。把 ref 码重放进同一只高通和 Ŝ_eff，写出 x'。n 上限 400。 */
int audio_xchk(const uint16_t *ref, float *xf, int n);

#endif
