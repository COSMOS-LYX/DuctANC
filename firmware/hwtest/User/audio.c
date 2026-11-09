#include "audio.h"

#include <math.h>
#include <string.h>

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
static uint16_t cap_buf[CAP_MAX * AUDIO_NCH] __attribute__((section(".ccmram")));

static TaskHandle_t audio_task_h;
static volatile TaskHandle_t cap_waiter;
static volatile uint32_t cap_need, cap_pos;

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

static void process_adc(const uint16_t *src)
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

    if (cap_pos < cap_need) {
        uint32_t k = cap_need - cap_pos;
        if (k > AUDIO_BLK)
            k = AUDIO_BLK;
        memcpy(&cap_buf[cap_pos * AUDIO_NCH], src, k * AUDIO_NCH * sizeof(uint16_t));
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

static void audio_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t ev;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &ev, portMAX_DELAY);
        if (ev & EV_DAC_H0) fill_dac(&dac_buf[0]);
        if (ev & EV_DAC_H1) fill_dac(&dac_buf[AUDIO_BLK]);
        if (ev & EV_ADC_H0) process_adc(&adc_buf[0]);
        if (ev & EV_ADC_H1) process_adc(&adc_buf[AUDIO_BLK * AUDIO_NCH]);
    }
}

void audio_start_task(void)
{
    xTaskCreate(audio_task, "audio", 512, NULL, configMAX_PRIORITIES - 1, &audio_task_h);
    configASSERT(audio_task_h);
    TIM_Cmd(TIM2, ENABLE);
}

void audio_set_signal(const sig_cfg_t *cfg)
{
    taskENTER_CRITICAL();
    sig_req = *cfg;
    sig_req_seq++;
    taskEXIT_CRITICAL();
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
    *out = diag;
    taskEXIT_CRITICAL();
}

const uint16_t *audio_capture(uint32_t frames, uint32_t timeout_ms)
{
    bool ok;

    if (frames == 0 || frames > CAP_MAX)
        return NULL;
    ulTaskNotifyTake(pdTRUE, 0);
    taskENTER_CRITICAL();
    cap_waiter = xTaskGetCurrentTaskHandle();
    cap_pos = 0;
    cap_need = frames;
    taskEXIT_CRITICAL();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    taskENTER_CRITICAL();
    ok = cap_pos >= cap_need;
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
