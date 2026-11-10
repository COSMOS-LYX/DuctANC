/* 200 Hz FxNLMS。系数和步进与 P3 同一套：减号更新、ε=1、二阶 80 Hz 高通。 */
#ifndef ANC_H
#define ANC_H

#include <stdint.h>

#define ANC_N    16
#define ANC_SHAT 384
#define ANC_FS   8000

typedef struct anc anc_t;

anc_t *anc_state(void);

void anc_init(anc_t *s);
void anc_reset_runtime(anc_t *s); /* 清延迟线、W、高通状态，不动 Ŝ */
void anc_reset_w(anc_t *s);

/* 从序号 index 起写入 n 个 float。n 必须是 8，index 是 8 的倍数。 */
int anc_shat_chunk(anc_t *s, int index, const float *v, int n);
int anc_shat_ready(const anc_t *s);
uint32_t anc_shat_crc(const anc_t *s);
void anc_shat_clear(anc_t *s);
const float *anc_shat(const anc_t *s);

void anc_set_mu(anc_t *s, float mu);
void anc_set_gamma(anc_t *s, float gamma);
void anc_set_clamp(anc_t *s, float clamp);
float anc_mu(const anc_t *s);
float anc_gamma(const anc_t *s);
float anc_clamp(const anc_t *s);
float anc_wnorm(const anc_t *s);

typedef struct {
    float x_mv;   /* 高通后的参考，mV */
    float e_mv;   /* 高通后的误差，mV */
    float xf;     /* Ŝ_eff * x */
    float y;      /* 量化并限幅后的码，相对中点 */
    int clamped;
    float wnorm;
} anc_sample_t;

/* adapt=0 时只算滤波和 y，不改 W。高通和延迟线总会往前走。 */
void anc_step(anc_t *s, uint16_t ref_code, uint16_t err_code, int adapt, anc_sample_t *out);

/* 用已提交的 Ŝ_eff，从零状态重算参考路的 x'。不改 s 里正在跑的延迟线。 */
int anc_replay_xf(const anc_t *s, const uint16_t *ref, float *xf, int n);

/* 0.25 s Goertzel，输入是高通后的 mV。满窗后 rms 更新，返回 1。 */
typedef struct {
    float q1, q2, rms;
    uint32_t n;
} anc_goertzel_t;

void anc_goertzel_reset(anc_goertzel_t *g);
int anc_goertzel_step(anc_goertzel_t *g, float x);

#endif
