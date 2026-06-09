#include "romlib_runtime.h"
#include "ci_fft.h"

static riscv_rfft_fast_instance_f32 s_spk_rfft;
static int s_spk_fft_inited = 0;

int ci_software_fft_w512_s256_init(void)
{
    if (s_spk_fft_inited) return 0;
    int ret = MASK_ROM_LIB_FUNC->fftfunc.riscv_rfft_fast_init_f32_p(&s_spk_rfft, 512);
    if (ret == 0) s_spk_fft_inited = 1;
    return ret;
}

int ci_software_fft_w512_s256(const short *audio_data, const float *window_data, float *result)
{
    float f32_buf[512];
    for (int i = 0; i < 512; i++) {
        f32_buf[i] = (float)audio_data[i] * window_data[i];
    }
    MASK_ROM_LIB_FUNC->fftfunc.riscv_rfft_fast_f32_p(&s_spk_rfft, f32_buf, result, 0);
    result[1] = 0.0f;  /* CMSIS-DSP packs Nyquist here; interleaved wants Im(DC)=0 */
    return 0;
}

/* Provide mprintf as a function symbol for precompiled libraries
 * (libnewlib_port.a __wrap_cosf/__wrap_sinf etc.) that call it directly.
 * All source code uses the mprintf macro from ci_log.h instead. */
int mprintf(const char *fmt, ...) { return 0; }
