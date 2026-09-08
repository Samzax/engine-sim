#include "../include/convolution_filter.h"

#include <assert.h>
#include <string.h>
#if defined(_M_X64) || defined(__SSE2__)
#include <emmintrin.h>
#endif

namespace {
float dot(const float *a, const float *b, int count) {
    int i = 0;
    float result = 0;
#if defined(_M_X64) || defined(__SSE2__)
    // SSE2 is part of the x64 baseline; unaligned loads also handle the ring's
    // wrapped segments. Other platforms retain the scalar implementation.
    __m128 sum = _mm_setzero_ps();
    for (; i + 4 <= count; i += 4) {
        sum = _mm_add_ps(sum, _mm_mul_ps(_mm_loadu_ps(a + i), _mm_loadu_ps(b + i)));
    }
    float lanes[4];
    _mm_storeu_ps(lanes, sum);
    result = (lanes[0] + lanes[1]) + (lanes[2] + lanes[3]);
#endif
    for (; i < count; ++i) result += a[i] * b[i];
    return result;
}
}

ConvolutionFilter::ConvolutionFilter() {
    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;

    m_shiftOffset = 0;
    m_sampleCount = 0;
}

ConvolutionFilter::~ConvolutionFilter() {
    destroy();
}

void ConvolutionFilter::initialize(int samples) {
    destroy();
    // Empty/missing responses use the dry signal, including before initialization.
    if (samples <= 0) return;
    m_sampleCount = samples;
    m_shiftOffset = 0;
    m_shiftRegister = new float[samples];
    m_impulseResponse = new float[samples];

    memset(m_shiftRegister, 0, sizeof(float) * samples);
    memset(m_impulseResponse, 0, sizeof(float) * samples);
}

void ConvolutionFilter::destroy() {
    delete[] m_shiftRegister;
    delete[] m_impulseResponse;

    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;
    m_sampleCount = 0;
    m_shiftOffset = 0;
}

float ConvolutionFilter::f(float sample) {
    if (m_sampleCount == 0) return sample;
    m_shiftRegister[m_shiftOffset] = sample;

    const int first = m_sampleCount - m_shiftOffset;
    const float result = dot(m_impulseResponse, m_shiftRegister + m_shiftOffset, first)
        + dot(m_impulseResponse + first, m_shiftRegister, m_shiftOffset);

    m_shiftOffset = (m_shiftOffset - 1 + m_sampleCount) % m_sampleCount;

    return result;
}
