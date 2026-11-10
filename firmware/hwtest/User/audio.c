#include "audio.h"

#include <math.h>
#include <string.h>

#include "anc.h"

#include "FreeRTOS.h"
#include "task.h"

#define EV_ADC_H0 (1u << 0)
#define EV_ADC_H1 (1u << 1)
#define EV_DAC_H0 (1u << 2)
#define EV_DAC_H1 (1u << 3)

#define ADC_LEN      (2 * AUDIO_BLK * AUDIO_NCH)
#define DAC_LEN      (2 * AUDIO_BLK)
#define LEVEL_BLOCKS ((FS_HZ / 2 + AUDIO_BLK / 2) / AUDIO_BLK) /* 约 0.5 s 一窗 */
#define RAMP_STEP    (1.0f / (0.02f * FS_HZ))                  /* 20 ms 淡入淡出，防爆音 */
#define TWO_PI       6.28318530718f
#define DAC_MID      2048

static uint16_t adc_buf[ADC_LEN];
static uint16_t dac_buf[DAC_LEN];
static uint16_t cap_buf[CAP_SAMPLES] __attribute__((section(".ccmram")));
/*
 * DAC 半缓冲被 DMA 读完、重新填写之前的快照。
 * DAC 带 DMA 时，第 j 个触发把 DHR 送到输出再由 DMA 装入 dac_buf[j]，所以 dac_buf[j] 从第 j+1 个触发起输出；
 * ADC 第 k 帧在第 k 个触发采样，两个环形缓冲都是 2*AUDIO_BLK 帧、同一个 TIM2 触发，
 * 因此第 k 帧对应的 DAC 输出是 dac_buf[(k-1) mod 2*AUDIO_BLK]。DAC 半缓冲中断比同半的 ADC 中断早几 µs，
 * audio 任务又先处理 DAC 事件，处理 ADC 半缓冲 h 时 dac_hist[h] 和上一轮的 dac_hist[h^1] 都是对的。
 * 前提是 ADC/DAC DMA 没有重启过（INFO 里 ovr=0、udr=0）。
 */
static uint16_t dac_hist[2][AUDIO_BLK];

static TaskHandle_t audio_task_h;
static volatile TaskHandle_t cap_waiter;
static volatile uint32_t cap_need, cap_pos;
static volatile bool cap_dac;

/* 命令侧写请求，audio 任务在增益淡到 0 后切换 */
static sig_cfg_t sig_req;
static uint32_t sig_req_seq;
static sig_cfg_t sig_cur;
static uint32_t sig_cur_seq;
static float gain, phase, fcur, fk;
static uint32_t rng = 0x12345678u;
static volatile bool out_active;

static level_t level_acc, level_pub;
static uint32_t level_blocks;
static audio_diag_t diag;
static volatile uint32_t dac_isr_cyc;
static volatile int anc_on, anc_prot;
static float anc_y;
static float baseline, err_rms_pub, y_rms_pub, e200, r200;
static float err_ss, y_ss;
static uint32_t err_n, y_n, sample_n, clamp_n, lag_cyc_max, cpu_cyc_max;
static int baseline_init, over_windows;
static anc_goertzel_t g_ref, g_err;

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void level_reset(level_t *l)
{
    memset(l, 0, sizeof *l);
    for (int c = 0; c < AUDIO_NCH; c++)
        l->min[c] = 0xFFFF;
}

static void dma_restart(DMA_Stream_TypeDef *s, uint32_t len)
{
    DMA_Cmd(s, DISABLE);
    while (s->CR & DMA_SxCR_EN) {
    }
    if (s == DMA2_Stream0)
        DMA_ClearFlag(s, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);
    else
        DMA_ClearFlag(s, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
    DMA_SetCurrDataCounter(s, len);
    DMA_Cmd(s, ENABLE);
}

static void dma_setup(DMA_Stream_TypeDef *s, uint32_t ch, uint32_t periph, void *mem,
                      uint32_t len, uint32_t dir, uint8_t irq)
{
    DMA_InitTypeDef d;
    NVIC_InitTypeDef n;

    DMA_DeInit(s);
    DMA_StructInit(&d);
    d.DMA_Channel = ch;
    d.DMA_PeripheralBaseAddr = periph;
    d.DMA_Memory0BaseAddr = (uint32_t)mem;
    d.DMA_DIR = dir;
    d.DMA_BufferSize = len;
    d.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    d.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    d.DMA_Mode = DMA_Mode_Circular;
    d.DMA_Priority = DMA_Priority_High;
    d.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_Init(s, &d);
    DMA_ITConfig(s, DMA_IT_HT | DMA_IT_TC, ENABLE);

    n.NVIC_IRQChannel = irq;
    n.NVIC_IRQChannelPreemptionPriority = IRQ_PRIO_DMA_AUDIO;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
    DMA_Cmd(s, ENABLE);
}

void audio_init(void)
{
    GPIO_InitTypeDef g;
    TIM_TimeBaseInitTypeDef t;
    ADC_CommonInitTypeDef ac;
    ADC_InitTypeDef a;
    DAC_InitTypeDef da;
    NVIC_InitTypeDef n;
    RCC_ClocksTypeDef clk;

    level_reset(&level_acc);
    level_reset(&level_pub);
    dwt_enable();
    anc_init(anc_state());
    for (int i = 0; i < DAC_LEN; i++)
        dac_buf[i] = DAC_MID;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC |
                           RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_DAC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    g.GPIO_Mode = GPIO_Mode_AN;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOC, &g);
    g.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &g);

    /* TIM2：更新事件作为 TRGO，频率 FS_HZ */
    RCC_GetClocksFreq(&clk);
    uint32_t tim_clk = clk.PCLK1_Frequency * (clk.PCLK1_Frequency == clk.HCLK_Frequency ? 1 : 2);
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler = 0;
    t.TIM_Period = tim_clk / FS_HZ - 1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &t);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

    /* ADC1：扫描 IN10 (PC0)、IN11 (PC1)，TIM2_TRGO 上升沿触发，DMA2 Stream0 Ch0 */
    dma_setup(DMA2_Stream0, DMA_Channel_0, (uint32_t)&ADC1->DR, adc_buf, ADC_LEN,
              DMA_DIR_PeripheralToMemory, DMA2_Stream0_IRQn);
    ac.ADC_Mode = ADC_Mode_Independent;
    ac.ADC_Prescaler = ADC_Prescaler_Div4; /* 84/4 = 21 MHz */
    ac.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ac.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ac);
    a.ADC_Resolution = ADC_Resolution_12b;
    a.ADC_ScanConvMode = ENABLE;
    a.ADC_ContinuousConvMode = DISABLE;
    a.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    a.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    a.ADC_DataAlign = ADC_DataAlign_Right;
    a.ADC_NbrOfConversion = AUDIO_NCH;
    ADC_Init(ADC1, &a);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_56Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_56Cycles);
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_ITConfig(ADC1, ADC_IT_OVR, ENABLE);
    n.NVIC_IRQChannel = ADC_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = IRQ_PRIO_ADC;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    /* DAC1：TIM2_TRGO 触发，输出缓冲开，DMA1 Stream5 Ch7 */
    dma_setup(DMA1_Stream5, DMA_Channel_7, (uint32_t)&DAC->DHR12R1, dac_buf, DAC_LEN,
              DMA_DIR_MemoryToPeripheral, DMA1_Stream5_IRQn);
    DAC_StructInit(&da);
    da.DAC_Trigger = DAC_Trigger_T2_TRGO;
    da.DAC_WaveGeneration = DAC_WaveGeneration_None;
    da.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &da);
    DAC_SetChannel1Data(DAC_Align_12b_R, DAC_MID);
    DAC_ITConfig(DAC_Channel_1, DAC_IT_DMAUDR, ENABLE);
    n.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_Init(&n);
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_DMACmd(DAC_Channel_1, ENABLE);
}

static void fill_dac(uint16_t *dst)
{
    sig_cfg_t req;
    uint32_t seq;

    taskENTER_CRITICAL();
    req = sig_req;
    seq = sig_req_seq;
    taskEXIT_CRITICAL();

    for (int i = 0; i < AUDIO_BLK; i++) {
        if (seq != sig_cur_seq && gain <= 0.0f) {
            sig_cur = req;
            sig_cur_seq = seq;
            phase = 0.0f;
            fcur = req.f1;
            fk = req.mode == SIG_SWEEP ? powf(req.f2 / req.f1, 1.0f / (req.sweep_s * FS_HZ)) : 1.0f;
        }
        float target = (seq != sig_cur_seq || sig_cur.mode == SIG_OFF) ? 0.0f : sig_cur.amp;
        if (gain < target)
            gain = fminf(gain + RAMP_STEP, target);
        else if (gain > target)
            gain = fmaxf(gain - RAMP_STEP, target);

        float s = 0.0f;
        switch (sig_cur.mode) {
        case SIG_TONE:
        case SIG_SWEEP:
            s = sinf(phase);
            phase += TWO_PI * fcur / FS_HZ;
            if (phase >= TWO_PI)
                phase -= TWO_PI;
            if (sig_cur.mode == SIG_SWEEP && (fcur *= fk) >= sig_cur.f2)
                fcur = sig_cur.f1;
            break;
        case SIG_NOISE:
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            s = (float)(int32_t)rng * (1.0f / 2147483648.0f);
            break;
        default:
            break;
        }
        int32_t v = DAC_MID + (int32_t)lrintf(gain * s * 2047.0f);
        dst[i] = (uint16_t)(v < 0 ? 0 : v > 4095 ? 4095 : v);
    }
    out_active = gain > 0.0f || (seq == sig_cur_seq && sig_cur.mode != SIG_OFF);
}

static void process_adc(const uint16_t *src, int half)
{
    for (int i = 0; i < AUDIO_BLK; i++) {
        for (int c = 0; c < AUDIO_NCH; c++) {
            uint32_t v = src[i * AUDIO_NCH + c];
            level_acc.sum[c] += v;
            level_acc.sumsq[c] += v * v;
            if (v < level_acc.min[c]) level_acc.min[c] = (uint16_t)v;
            if (v > level_acc.max[c]) level_acc.max[c] = (uint16_t)v;
            if (v <= ADC_CLIP_LO || v >= ADC_CLIP_HI) level_acc.clip[c]++;
        }
    }
    level_acc.n += AUDIO_BLK;

    anc_t *st = anc_state();
    for (int i = 0; i < AUDIO_BLK; i++) {
        uint32_t t0 = DWT->CYCCNT;
        int adapt = anc_on && !anc_prot && anc_shat_ready(st);
        anc_sample_t o;
        anc_step(st, src[i * AUDIO_NCH], src[i * AUDIO_NCH + 1], adapt, &o);
        uint32_t dt = DWT->CYCCNT - t0;
        diag.cpu_cyc = dt;
        if (dt > cpu_cyc_max)
            cpu_cyc_max = dt;
        if (anc_goertzel_step(&g_ref, o.x_mv))
            r200 = g_ref.rms;
        if (anc_goertzel_step(&g_err, o.e_mv))
            e200 = g_err.rms;
        err_ss += o.e_mv * o.e_mv;
        y_ss += o.y * o.y;
        err_n++;
        y_n++;
        if (o.clamped && adapt)
            clamp_n++;
        anc_y = adapt ? o.y : 0.0f;
        if (err_n >= (FS_HZ / 20)) { /* 50 ms */
            float rms = sqrtf(err_ss / (float)err_n);
            err_rms_pub = rms;
            y_rms_pub = sqrtf(y_ss / (float)y_n);
            err_ss = y_ss = 0.0f;
            err_n = y_n = 0;
            if (!anc_on) {
                baseline = baseline_init ? baseline + 0.05f * (rms - baseline) : rms;
                baseline_init = 1;
                over_windows = 0;
            } else if (!anc_prot && baseline_init && rms > 2.0f * baseline) {
                if (++over_windows >= 4) {
                    anc_prot = 1;
                    anc_reset_w(st);
                    anc_y = 0.0f;
                }
            } else {
                over_windows = 0;
            }
        }
        sample_n++;
    }

    if (cap_pos < cap_need) {
        uint32_t k = cap_need - cap_pos;
        if (k > AUDIO_BLK)
            k = AUDIO_BLK;
        if (cap_dac) {
            uint16_t *dst = &cap_buf[cap_pos * (AUDIO_NCH + 1)];
            for (uint32_t i = 0; i < k; i++, dst += AUDIO_NCH + 1) {
                for (int c = 0; c < AUDIO_NCH; c++)
                    dst[c] = src[i * AUDIO_NCH + c];
                dst[AUDIO_NCH] = i ? dac_hist[half][i - 1] : dac_hist[half ^ 1][AUDIO_BLK - 1];
            }
        } else {
            memcpy(&cap_buf[cap_pos * AUDIO_NCH], src, k * AUDIO_NCH * sizeof(uint16_t));
        }
        cap_pos += k;
        if (cap_pos >= cap_need && cap_waiter)
            xTaskNotifyGive(cap_waiter);
    }

    if (++level_blocks >= LEVEL_BLOCKS) {
        level_acc.tick = xTaskGetTickCount();
        taskENTER_CRITICAL();
        level_pub = level_acc;
        taskEXIT_CRITICAL();
        level_reset(&level_acc);
        level_blocks = 0;
    }
}

/* 双缓冲只有两格。槽被 DMA 读走后，要再过一格才会被读第二次，所以期限是两个采样周期。
 * 超过这个时间再写，DMA 已经把旧码装进 DHR，late/udr 都不会加。 */
static void write_dac_half(uint16_t *dst, int half)
{
    uint32_t isr_cyc = dac_isr_cyc;
    uint32_t dt = DWT->CYCCNT - isr_cyc;
    if (isr_cyc != 0) {
        if (dt > lag_cyc_max)
            lag_cyc_max = dt;
        if (dt > 2u * (SystemCoreClock / FS_HZ))
            diag.slip++;
    }
    memcpy(dac_hist[half], dst, sizeof dac_hist[half]);
    if (anc_on) {
        for (int i = 0; i < AUDIO_BLK; i++) {
            int32_t v = DAC_MID;
            if (!anc_prot)
                v += (int32_t)anc_y;
            if (v < 0)
                v = 0;
            else if (v > 4095)
                v = 4095;
            dst[i] = (uint16_t)v;
        }
        out_active = !anc_prot && anc_y != 0.0f;
    } else {
        fill_dac(dst);
    }
}

static void audio_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t ev;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &ev, portMAX_DELAY);
        if (ev & EV_DAC_H0)
            write_dac_half(&dac_buf[0], 0);
        if (ev & EV_DAC_H1)
            write_dac_half(&dac_buf[AUDIO_BLK], 1);
        if (ev & EV_ADC_H0) process_adc(&adc_buf[0], 0);
        if (ev & EV_ADC_H1) process_adc(&adc_buf[AUDIO_BLK * AUDIO_NCH], 1);
    }
}

void audio_start_task(void)
{
    xTaskCreate(audio_task, "audio", 512, NULL, configMAX_PRIORITIES - 1, &audio_task_h);
    configASSERT(audio_task_h);
    TIM_Cmd(TIM2, ENABLE);
}

bool audio_set_signal(const sig_cfg_t *cfg)
{
    if (anc_on)
        return false;
    taskENTER_CRITICAL();
    sig_req = *cfg;
    sig_req_seq++;
    taskEXIT_CRITICAL();
    return true;
}

void audio_get_signal(sig_cfg_t *cfg)
{
    taskENTER_CRITICAL();
    *cfg = sig_req;
    taskEXIT_CRITICAL();
}

bool audio_output_active(void)
{
    return out_active;
}

void audio_get_level(level_t *out)
{
    taskENTER_CRITICAL();
    *out = level_pub;
    taskEXIT_CRITICAL();
}

void audio_get_diag(audio_diag_t *out)
{
    taskENTER_CRITICAL();
    diag.lag_us_max = (SystemCoreClock >= 1000000u) ? lag_cyc_max / (SystemCoreClock / 1000000u) : 0;
    *out = diag;
    taskEXIT_CRITICAL();
}

static void force_silence(void)
{
    sig_cfg_t off = {0};
    sig_req = off;
    sig_cur = off;
    sig_req_seq++;
    sig_cur_seq = sig_req_seq;
    gain = 0.0f;
    out_active = false;
}

bool audio_anc_is_on(void)
{
    return anc_on;
}

bool audio_anc_on(const char **err)
{
    if (!anc_shat_ready(anc_state())) {
        if (err) *err = "shat";
        return false;
    }
    if (anc_prot) {
        if (err) *err = "protected";
        return false;
    }
    if (sample_n < (FS_HZ / 2)) {
        if (err) *err = "settling";
        return false;
    }
    taskENTER_CRITICAL();
    force_silence();
    anc_reset_w(anc_state());
    clamp_n = 0;
    over_windows = 0;
    anc_y = 0.0f;
    diag.slip = 0;
    lag_cyc_max = 0;
    cpu_cyc_max = 0;
    anc_on = 1;
    taskEXIT_CRITICAL();
    return true;
}

void audio_anc_off(void)
{
    taskENTER_CRITICAL();
    anc_on = 0;
    anc_prot = 0;
    anc_y = 0.0f;
    over_windows = 0;
    anc_reset_w(anc_state());
    force_silence();
    taskEXIT_CRITICAL();
}

void audio_anc_set_mu(float mu) { anc_set_mu(anc_state(), mu); }
void audio_anc_set_gamma(float gamma) { anc_set_gamma(anc_state(), gamma); }
void audio_anc_set_clamp(float clamp) { anc_set_clamp(anc_state(), clamp); }

void audio_anc_stat(audio_anc_stat_t *out)
{
    anc_t *st = anc_state();
    uint32_t period = SystemCoreClock / FS_HZ;
    taskENTER_CRITICAL();
    out->on = anc_on;
    out->prot = anc_prot;
    out->shat = anc_shat_ready(st);
    out->settle = sample_n >= (FS_HZ / 2);
    out->mu = anc_mu(st);
    out->gamma = anc_gamma(st);
    out->clamp = anc_clamp(st);
    out->wnorm = anc_wnorm(st);
    out->y_rms = y_rms_pub;
    out->err_rms = err_rms_pub;
    out->e200 = e200;
    out->r200 = r200;
    out->baseline = baseline;
    out->clamp_n = clamp_n;
    out->lag_us = (SystemCoreClock >= 1000000u) ? lag_cyc_max / (SystemCoreClock / 1000000u) : 0;
    out->slip = diag.slip;
    out->late = diag.late_blocks;
    out->ovr = diag.adc_ovr;
    out->udr = diag.dac_udr;
    out->samples = sample_n;
    out->cpu_pct = period ? 100.0f * (float)cpu_cyc_max / (float)period : 0.0f;
    lag_cyc_max = 0;
    cpu_cyc_max = 0;
    taskEXIT_CRITICAL();
}

int audio_anc_shat_chunk(int index, const float *v, int n)
{
    if (anc_on)
        return 0;
    return anc_shat_chunk(anc_state(), index, v, n);
}

int audio_anc_shat_reset(void)
{
    if (anc_on)
        return 0;
    anc_shat_clear(anc_state());
    return 1;
}

uint32_t audio_anc_shat_crc(void)
{
    return anc_shat_crc(anc_state());
}

int audio_xchk(const uint16_t *ref, float *xf, int n)
{
    if (anc_on || n <= 0 || n > 400)
        return 0;
    return anc_replay_xf(anc_state(), ref, xf, n);
}

int audio_capture_arm(uint32_t frames, bool with_dac)
{
    if (frames == 0 || frames > (with_dac ? CAP_MAX_DAC : CAP_MAX))
        return 0;
    taskENTER_CRITICAL();
    if (cap_need != 0 && cap_pos < cap_need) {
        taskEXIT_CRITICAL();
        return 0;
    }
    cap_waiter = NULL;
    cap_dac = with_dac;
    cap_pos = 0;
    cap_need = frames;
    taskEXIT_CRITICAL();
    return 1;
}

const uint16_t *audio_capture_take(uint32_t *frames, bool *with_dac)
{
    taskENTER_CRITICAL();
    if (cap_need == 0 || cap_pos < cap_need) {
        taskEXIT_CRITICAL();
        return NULL;
    }
    if (frames)
        *frames = cap_need;
    if (with_dac)
        *with_dac = cap_dac;
    cap_need = 0;
    cap_waiter = NULL;
    taskEXIT_CRITICAL();
    return cap_buf;
}

const uint16_t *audio_capture(uint32_t frames, bool with_dac, uint32_t timeout_ms)
{
    bool ok;

    ulTaskNotifyTake(pdTRUE, 0);
    if (!audio_capture_arm(frames, with_dac))
        return NULL;
    taskENTER_CRITICAL();
    cap_waiter = xTaskGetCurrentTaskHandle();
    if (cap_pos >= cap_need)
        xTaskNotifyGive(cap_waiter);
    taskEXIT_CRITICAL();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    taskENTER_CRITICAL();
    ok = cap_need != 0 && cap_pos >= cap_need;
    cap_need = 0;
    cap_waiter = NULL;
    taskEXIT_CRITICAL();
    return ok ? cap_buf : NULL;
}

static void notify_audio(uint32_t ev, BaseType_t *woken)
{
    uint32_t prev = 0;
    if (!ev || !audio_task_h)
        return;
    xTaskNotifyAndQueryFromISR(audio_task_h, ev, eSetBits, &prev, woken);
    if (prev & ev)
        diag.late_blocks++; /* 上一块还没处理完 */
}

void DMA2_Stream0_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint32_t ev = 0;

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0)) {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
        ev |= EV_ADC_H0;
    }
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        ev |= EV_ADC_H1;
    }
    diag.blocks++;
    notify_audio(ev, &woken);
    portYIELD_FROM_ISR(woken);
}

void DMA1_Stream5_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint32_t ev = 0;

    /* HT：DMA 正在读后半，回填前半；TC 反之 */
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5)) {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
        ev |= EV_DAC_H0;
    }
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        ev |= EV_DAC_H1;
    }
    if (ev)
        dac_isr_cyc = DWT->CYCCNT;
    notify_audio(ev, &woken);
    portYIELD_FROM_ISR(woken);
}

void ADC_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_OVR)) {
        diag.adc_ovr++;
        ADC_DMACmd(ADC1, DISABLE);
        ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);
        dma_restart(DMA2_Stream0, ADC_LEN);
        ADC_DMACmd(ADC1, ENABLE);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if (DAC_GetITStatus(DAC_Channel_1, DAC_IT_DMAUDR)) {
        diag.dac_udr++;
        DAC_DMACmd(DAC_Channel_1, DISABLE);
        DAC_ClearITPendingBit(DAC_Channel_1, DAC_IT_DMAUDR);
        dma_restart(DMA1_Stream5, DAC_LEN);
        DAC_DMACmd(DAC_Channel_1, ENABLE);
    }
}
