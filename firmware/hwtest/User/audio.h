/* 音频链路：TIM2 同时触发 ADC1（PC0/PC1）和 DAC1（PA4），双缓冲 DMA，块处理在 audio 任务里 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#define AUDIO_BLK   128               /* 每半缓冲帧数，8 kHz 时 16 ms */
#define AUDIO_NCH   2                 /* 0 = 参考麦 PC0，1 = 误差麦 PC1 */
#define CAP_MAX     16000             /* 采集上限帧数（CCM 64 KB） */
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
} audio_diag_t;

void audio_init(void);
void audio_start_task(void);
void audio_set_signal(const sig_cfg_t *cfg);
void audio_get_signal(sig_cfg_t *cfg);
bool audio_output_active(void);
void audio_get_level(level_t *out);
void audio_get_diag(audio_diag_t *out);
/* 采集 frames 帧到 CCM，阻塞到完成；返回缓冲指针（交织 ref,err） */
const uint16_t *audio_capture(uint32_t frames, uint32_t timeout_ms);

#endif
