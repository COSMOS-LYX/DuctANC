#include "anc.h"

#include <math.h>
#include <string.h>

/* 热循环按 O2 编译。Debug 预设整体是 -Og，384 点 FIR 用 -Og 会把节拍占满。 */
#pragma GCC optimize("O2")
#pragma GCC optimize("fp-contract=off")

#define ANC_EPS  1.0f
#define CODE_MID 2048

/* float32 位型与 host/anc_coeff.py 里的常量逐位相同。二阶 Butterworth，80 Hz，8 kHz。 */
static float bits_to_f(uint32_t u)
{
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}

static const uint32_t HP_B_BITS[3] = {0x3F74E004u, 0xBFF4E004u, 0x3F74E004u};
static const uint32_t HP_A_BITS[2] = {0xBFF4A21Bu, 0x3F6A3BDBu}; /* a1 a2，a0 = 1 */
static const uint32_t MV_PER_CODE_BITS = 0x3F4E4CE5u;            /* 3300/4095 */
/* 2*cos(2π*200/8000)，Goertzel 系数 */
static const uint32_t GOERTZEL_BITS = 0x3FFCD925u;

struct anc {
    float b0, b1, b2, a1, a2;
    float mv_per_code;
    float z_ref[2], z_err[2];
    float shat[ANC_SHAT];
    float x_hist[ANC_SHAT];
    float xf_hist[ANC_N];
    float w[ANC_N];
    int x_pos, xf_pos;
    uint64_t shat_mask; /* 48 块，bit i 表示第 i 块已到 */
    float mu, gamma, clamp;
    float g_coeff;
};

static anc_t g_anc;

anc_t *anc_state(void)
{
    return &g_anc;
}

static float biquad(float x, float b0, float b1, float b2, float a1, float a2, float z[2])
{
    float y = b0 * x + z[0];
    z[0] = b1 * x - a1 * y + z[1];
    z[1] = b2 * x - a2 * y;
    return y;
}

static void hist_push(float *hist, int n, int *pos, float x)
{
    int p = *pos + 1;
    if (p == n)
        p = 0;
    hist[p] = x;
    *pos = p;
}

static float hist_at(const float *hist, int n, int pos, int k)
{
    int i = pos - k;
    if (i < 0)
        i += n;
    return hist[i];
}

void anc_init(anc_t *s)
{
    memset(s, 0, sizeof *s);
    s->b0 = bits_to_f(HP_B_BITS[0]);
    s->b1 = bits_to_f(HP_B_BITS[1]);
    s->b2 = bits_to_f(HP_B_BITS[2]);
    s->a1 = bits_to_f(HP_A_BITS[0]);
    s->a2 = bits_to_f(HP_A_BITS[1]);
    s->mv_per_code = bits_to_f(MV_PER_CODE_BITS);
    s->g_coeff = bits_to_f(GOERTZEL_BITS);
    s->mu = 0.002f;
    s->gamma = 0.0f;
    s->clamp = 40.0f;
    s->x_pos = ANC_SHAT - 1;
    s->xf_pos = ANC_N - 1;
}

void anc_reset_runtime(anc_t *s)
{
    memset(s->z_ref, 0, sizeof s->z_ref);
    memset(s->z_err, 0, sizeof s->z_err);
    memset(s->x_hist, 0, sizeof s->x_hist);
    memset(s->xf_hist, 0, sizeof s->xf_hist);
    memset(s->w, 0, sizeof s->w);
    s->x_pos = ANC_SHAT - 1;
    s->xf_pos = ANC_N - 1;
}

void anc_reset_w(anc_t *s)
{
    memset(s->w, 0, sizeof s->w);
}

void anc_shat_clear(anc_t *s)
{
    memset(s->shat, 0, sizeof s->shat);
    s->shat_mask = 0;
}

int anc_shat_chunk(anc_t *s, int index, const float *v, int n)
{
    if (n != 8 || index < 0 || (index % 8) != 0 || index + n > ANC_SHAT)
        return 0;
    memcpy(&s->shat[index], v, (size_t)n * sizeof(float));
    s->shat_mask |= 1ull << (index / 8);
    return 1;
}

int anc_shat_ready(const anc_t *s)
{
    return s->shat_mask == ((1ull << (ANC_SHAT / 8)) - 1ull);
}

static uint32_t crc32_bytes(const void *data, size_t n)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

uint32_t anc_shat_crc(const anc_t *s)
{
    return crc32_bytes(s->shat, sizeof s->shat);
}

const float *anc_shat(const anc_t *s)
{
    return s->shat;
}

void anc_set_mu(anc_t *s, float mu) { s->mu = mu; }
void anc_set_gamma(anc_t *s, float gamma) { s->gamma = gamma; }
void anc_set_clamp(anc_t *s, float clamp) { s->clamp = clamp; }
float anc_mu(const anc_t *s) { return s->mu; }
float anc_gamma(const anc_t *s) { return s->gamma; }
float anc_clamp(const anc_t *s) { return s->clamp; }

float anc_wnorm(const anc_t *s)
{
    float wn = 0.0f;
    for (int k = 0; k < ANC_N; k++)
        wn += s->w[k] * s->w[k];
    return sqrtf(wn);
}

static float code_to_mv(const anc_t *s, uint16_t code)
{
    return ((int)code - CODE_MID) * s->mv_per_code;
}

static float fir_shat(const anc_t *s)
{
    float acc = 0.0f;
    for (int k = 0; k < ANC_SHAT; k++)
        acc += s->shat[k] * hist_at(s->x_hist, ANC_SHAT, s->x_pos, k);
    return acc;
}

void anc_step(anc_t *s, uint16_t ref_code, uint16_t err_code, int adapt, anc_sample_t *out)
{
    float x = biquad(code_to_mv(s, ref_code), s->b0, s->b1, s->b2, s->a1, s->a2, s->z_ref);
    float e = biquad(code_to_mv(s, err_code), s->b0, s->b1, s->b2, s->a1, s->a2, s->z_err);
    hist_push(s->x_hist, ANC_SHAT, &s->x_pos, x);
    float xf = fir_shat(s);
    hist_push(s->xf_hist, ANC_N, &s->xf_pos, xf);

    float y = 0.0f;
    float pwr = ANC_EPS;
    for (int k = 0; k < ANC_N; k++) {
        float xs = hist_at(s->x_hist, ANC_SHAT, s->x_pos, k);
        float rs = hist_at(s->xf_hist, ANC_N, s->xf_pos, k);
        y += s->w[k] * xs;
        pwr += rs * rs;
    }
    float yq = rintf(y);
    int clamped = 0;
    if (yq > s->clamp) {
        yq = s->clamp;
        clamped = 1;
    } else if (yq < -s->clamp) {
        yq = -s->clamp;
        clamped = 1;
    }
    if (adapt) {
        float scale = s->mu * e / pwr;
        float leak = 1.0f - s->gamma;
        for (int k = 0; k < ANC_N; k++) {
            float rs = hist_at(s->xf_hist, ANC_N, s->xf_pos, k);
            s->w[k] = leak * s->w[k] - scale * rs;
        }
    }
    float wn = 0.0f;
    for (int k = 0; k < ANC_N; k++)
        wn += s->w[k] * s->w[k];
    out->x_mv = x;
    out->e_mv = e;
    out->xf = xf;
    out->y = yq;
    out->clamped = clamped;
    out->wnorm = sqrtf(wn);
}

int anc_replay_xf(const anc_t *s, const uint16_t *ref, float *xf, int n)
{
    if (!anc_shat_ready(s) || n <= 0)
        return 0;
    /* 放 BSS。anc_t 大约 3 KB，CLI 栈只有 2 KB，不能放栈上。 */
    static anc_t tmp;
    anc_init(&tmp);
    memcpy(tmp.shat, s->shat, sizeof tmp.shat);
    tmp.shat_mask = s->shat_mask;
    for (int i = 0; i < n; i++) {
        anc_sample_t o;
        /* 误差麦填中点：x' 只用参考路。adapt=0，W 保持 0，y 保持 0。 */
        anc_step(&tmp, ref[i], CODE_MID, 0, &o);
        xf[i] = o.xf;
    }
    return 1;
}

void anc_goertzel_reset(anc_goertzel_t *g)
{
    g->q1 = g->q2 = 0.0f;
    g->n = 0;
}

int anc_goertzel_step(anc_goertzel_t *g, float x)
{
    float coeff = bits_to_f(GOERTZEL_BITS);
    float q0 = x + coeff * g->q1 - g->q2;
    g->q2 = g->q1;
    g->q1 = q0;
    g->n++;
    if (g->n < (ANC_FS / 4))
        return 0;
    float cw = coeff * 0.5f;
    float sw = bits_to_f(0x3E20305Bu); /* sin(2π*200/8000) */
    float real = g->q1 - g->q2 * cw;
    float imag = g->q2 * sw;
    float mag = sqrtf(real * real + imag * imag);
    g->rms = mag * 1.41421356f / (float)g->n;
    g->q1 = g->q2 = 0.0f;
    g->n = 0;
    return 1;
}
