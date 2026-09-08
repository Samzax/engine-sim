#include "../include/synthesizer.h"

#include "../include/utilities.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

#undef min
#undef max

Synthesizer::Synthesizer() {
    m_inputChannels = nullptr;
    m_inputChannelCount = 0;
    m_inputBufferSize = 0;
    m_inputWriteOffset = 0.0;
    m_pendingInputSamples = 0;
    m_latency = 0;
    m_processed = true;

    m_audioBufferSize = 0;

    m_inputSampleRate = 0.0;
    m_audioSampleRate = 0.0;

    m_lastInputSampleOffset = 0.0;

    m_run = true;
    m_thread = nullptr;
    m_filters = nullptr;
}

Synthesizer::~Synthesizer() {
    destroy();
}

void Synthesizer::initialize(const Parameters &p) {
    if (p.inputChannelCount <= 0 || p.inputBufferSize <= 0 || p.audioBufferSize <= 0
        || !std::isfinite(p.inputSampleRate) || p.inputSampleRate <= 0
        || !std::isfinite(p.audioSampleRate) || p.audioSampleRate <= 0) {
        throw std::invalid_argument("Invalid synthesizer channels, capacity or sample rate");
    }
    destroy();
    m_run = true;
    m_inputChannelCount = p.inputChannelCount;
    m_inputBufferSize = p.inputBufferSize;
    m_inputWriteOffset = p.inputBufferSize;
    m_audioBufferSize = p.audioBufferSize;
    m_inputSampleRate = p.inputSampleRate;
    m_audioSampleRate = p.audioSampleRate;
    m_audioParameters = p.initialAudioParameters;

    m_pendingInputSamples = 0;
    m_latency = 0;
    m_lastInputSampleOffset = 0;
    m_rendering = false;
    m_levelerGain = 1.0;
    m_levelingFilter = LevelingFilter();
    m_antialiasing.reset();

    m_inputWriteOffset = 0;
    m_processed = true;

    m_audioBuffer.initialize(p.audioBufferSize);
    m_inputChannels = new InputChannel[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_inputChannels[i].transferBuffer = new float[p.inputBufferSize];
        m_inputChannels[i].data.initialize(p.inputBufferSize);
    }

    m_filters = new ProcessingFilters[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            m_audioParameters.airNoiseFrequencyCutoff, m_audioSampleRate);

        m_filters[i].derivative.m_dt = 1 / m_audioSampleRate;

        m_filters[i].inputDcFilter.setCutoffFrequency(10.0);
        m_filters[i].inputDcFilter.m_dt = 1 / m_audioSampleRate;

        m_filters[i].jitterFilter.initialize(
            10,
            m_audioParameters.inputSampleNoiseFrequencyCutoff,
            m_audioSampleRate);

        m_filters[i].antialiasing.setCutoffFrequency(1900.0f, m_audioSampleRate);
    }

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    m_levelingFilter.p_maxLevel = m_audioParameters.levelerMaxGain;
    m_levelingFilter.p_minLevel = m_audioParameters.levelerMinGain;
    m_antialiasing.setCutoffFrequency(m_audioSampleRate * 0.45f, m_audioSampleRate);

    m_outputStaging.resize(std::min(2000, m_audioBufferSize));
}

void Synthesizer::initializeImpulseResponse(
    const int16_t *impulseResponse,
    unsigned int samples,
    float volume,
    int index)
{
    if (index < 0 || index >= m_inputChannelCount) {
        throw std::out_of_range("Invalid impulse response channel");
    }
    if (m_thread != nullptr) {
        throw std::logic_error("Stop audio rendering before replacing an impulse response");
    }
    unsigned int clippedLength = 0;
    for (unsigned int i = 0; impulseResponse != nullptr && i < samples; ++i) {
        if (std::abs(impulseResponse[i]) > 100) {
            clippedLength = i + 1;
        }
    }

    const unsigned int sampleCount = std::min(10000U, clippedLength);
    m_filters[index].convolution.initialize(sampleCount);
    for (unsigned int i = 0; i < sampleCount; ++i) {
        m_filters[index].convolution.getImpulseResponse()[i] =
            volume * impulseResponse[i] / INT16_MAX;
    }
}

void Synthesizer::startAudioRenderingThread() {
    if (m_thread != nullptr) return;
    if (m_inputChannelCount == 0) return;
    m_run = true;
    m_thread = new std::thread(&Synthesizer::audioRenderingThread, this);
}

void Synthesizer::endAudioRenderingThread() {
    if (m_thread != nullptr) {
        {
            std::lock_guard<std::mutex> lock(m_lock0);
            m_run = false;
        }
        m_cv0.notify_all();

        m_thread->join();
        delete m_thread;

        m_thread = nullptr;
    }
}

void Synthesizer::destroy() {
    endAudioRenderingThread();
    m_audioBuffer.destroy();

    for (int i = 0; m_inputChannels != nullptr && i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.destroy();
        delete[] m_inputChannels[i].transferBuffer;
    }

    delete[] m_inputChannels;
    delete[] m_filters;

    m_inputChannels = nullptr;
    m_filters = nullptr;

    m_inputChannelCount = 0;
    m_pendingInputSamples = 0;
    m_latency = 0;
    m_processed = true;
    m_rendering = false;
    m_outputStaging.clear();
}

int Synthesizer::readAudioOutput(int samples, int16_t *buffer) {
    if (samples <= 0) return 0;
    std::lock_guard<std::mutex> lock(m_lock0);

    const int newDataLength = m_audioBuffer.size();
    if (newDataLength >= samples) {
        m_audioBuffer.readAndRemove(samples, buffer);
    }
    else {
        m_audioBuffer.readAndRemove(newDataLength, buffer);
        memset(
            buffer + newDataLength,
            0,
            sizeof(int16_t) * ((size_t)samples - newDataLength));
    }
    
    const int samplesConsumed = std::min(samples, newDataLength);
    m_cv0.notify_all();

    return samplesConsumed;
}

void Synthesizer::waitProcessed() {
    {
        std::unique_lock<std::mutex> lk(m_lock0);
        m_cv0.wait(lk, [this] { return !m_run || m_processed; });
    }
}

void Synthesizer::writeInput(const double *data) {
    std::lock_guard<std::mutex> lock(m_lock0);
    if (m_inputChannelCount == 0) return;
    m_inputWriteOffset = std::fmod(m_inputWriteOffset + (double)m_audioSampleRate / m_inputSampleRate,
        (double)m_inputBufferSize);

    for (int i = 0; i < m_inputChannelCount; ++i) {
        RingBuffer<float> &buffer = m_inputChannels[i].data;
        const double lastInputSample = m_inputChannels[i].lastInputSample;
        const size_t baseIndex = buffer.writeIndex();
        const double distance =
            inputDistance(m_inputWriteOffset, m_lastInputSampleOffset);
        if (distance <= 0) continue;
        double s =
            inputDistance(baseIndex, m_lastInputSampleOffset);
        for (; s <= distance; s += 1.0) {
            if (s >= m_inputBufferSize) s -= m_inputBufferSize;

            const double f = s / distance;
            const double sample = lastInputSample * (1 - f) + data[i] * f;

            // RingBuffer drops its oldest sample on overflow. Keep the count of
            // committed (renderable) samples consistent with that policy.
            if (i == 0 && buffer.size() == buffer.capacity() && m_pendingInputSamples > 0) {
                --m_pendingInputSamples;
            }
            buffer.write(m_filters[i].antialiasing.fast_f(static_cast<float>(sample)));
        }

        m_inputChannels[i].lastInputSample = data[i];
    }

    m_lastInputSampleOffset = m_inputWriteOffset;
    m_processed = m_pendingInputSamples == 0 && !m_rendering;
}

void Synthesizer::endInputBlock() {
    std::unique_lock<std::mutex> lk(m_lock0);
    m_pendingInputSamples = m_inputChannelCount == 0 ? 0 : (int)m_inputChannels[0].data.size();
    m_latency = m_pendingInputSamples;
    m_processed = m_pendingInputSamples == 0 && !m_rendering;

    lk.unlock();
    m_cv0.notify_all();
}

void Synthesizer::audioRenderingThread() {
    while (m_run) {
        renderBlock(true);
    }
}

#undef max
void Synthesizer::renderAudio() {
    if (m_thread != nullptr) throw std::logic_error("Audio worker already owns rendering");
    renderBlock(false);
}

void Synthesizer::renderBlock(bool waitForInput) {
    std::unique_lock<std::mutex> lk0(m_lock0);

    m_cv0.wait(lk0, [this, waitForInput] {
        return !m_run || !waitForInput
            || (m_pendingInputSamples > 0 && m_audioBuffer.size() < m_outputStaging.size());
    });
    if (!m_run || m_pendingInputSamples == 0) return;

    const int n = std::min(
        (int)m_outputStaging.size(),
        std::min((int)(m_outputStaging.size() - m_audioBuffer.size()), m_pendingInputSamples));
    if (n == 0) return;

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.readAndRemove(n, m_inputChannels[i].transferBuffer);
    }
    
    m_pendingInputSamples -= n;
    m_rendering = true;
    const AudioParameters params = m_audioParameters;

    lk0.unlock();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            params.airNoiseFrequencyCutoff, m_audioSampleRate);
        m_filters[i].jitterFilter.setJitterScale(params.inputSampleNoise);
    }

    for (int i = 0; i < n; ++i) {
        m_outputStaging[i] = renderSample(i, params);
    }
    lk0.lock();
    for (int i = 0; i < n; ++i) m_audioBuffer.write(m_outputStaging[i]);
    m_levelerGain = m_levelingFilter.getAttenuation();
    m_rendering = false;
    m_processed = m_pendingInputSamples == 0;
    lk0.unlock();
    m_cv0.notify_all();
}

double Synthesizer::getLatency() const {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_audioSampleRate > 0 ? (double)m_latency / m_audioSampleRate : 0.0;
}

int Synthesizer::inputDelta(int s1, int s0) const {
    return (s1 < s0)
        ? m_inputBufferSize - s0 + s1
        : s1 - s0;
}

double Synthesizer::inputDistance(double s1, double s0) const {
    return (s1 < s0)
        ? (double)m_inputBufferSize - s0 + s1
        : s1 - s0;
}

void Synthesizer::setInputSampleRate(double sampleRate) {
    if (!std::isfinite(sampleRate) || sampleRate <= 0) {
        throw std::invalid_argument("Input sample rate must be finite and positive");
    }
    std::lock_guard<std::mutex> lock(m_lock0);
    m_inputSampleRate = sampleRate;
}

double Synthesizer::getInputSampleRate() const {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_inputSampleRate;
}

int16_t Synthesizer::renderSample(int inputSample, const AudioParameters &params) {
    const float airNoise = params.airNoise;
    const float dF_F_mix = params.dF_F_mix;
    const float convAmount = params.convolution;

    float signal = 0;
    for (int i = 0; i < m_inputChannelCount; ++i) {
        const float r_0 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;

        const float jitteredSample =
            m_filters[i].jitterFilter.fast_f(m_inputChannels[i].transferBuffer[inputSample]);

        const float f_in = jitteredSample;
        const float f_dc = m_filters[i].inputDcFilter.fast_f(f_in);
        const float f = f_in - f_dc;
        const float f_p = m_filters[i].derivative.f(f_in);

        const float noise = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        const float r =
            m_filters[i].airNoiseLowPass.fast_f(noise);
        const float r_mixed =
            airNoise * r + (1 - airNoise);

        float v_in =
            f_p * dF_F_mix
            + f * r_mixed * (1 - dF_F_mix);
        if (std::fpclassify(v_in) == FP_SUBNORMAL) {
            v_in = 0;
        }

        const float v =
            convAmount * m_filters[i].convolution.f(v_in)
            + (1 - convAmount) * v_in;

        signal += v;
    }

    signal = m_antialiasing.fast_f(signal);

    m_levelingFilter.p_target = params.levelerTarget;
    m_levelingFilter.p_minLevel = params.levelerMinGain;
    m_levelingFilter.p_maxLevel = params.levelerMaxGain;
    const float v_leveled = m_levelingFilter.f(signal) * params.volume;
    if (!std::isfinite(v_leveled)) return 0;
    int r_int = std::lround(clamp(v_leveled, (float)INT16_MIN, (float)INT16_MAX));
    if (r_int > INT16_MAX) {
        r_int = INT16_MAX;
    }
    else if (r_int < INT16_MIN) {
        r_int = INT16_MIN;
    }

    return static_cast<int16_t>(r_int);
}

double Synthesizer::getLevelerGain() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_levelerGain;
}

Synthesizer::AudioParameters Synthesizer::getAudioParameters() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_audioParameters;
}

void Synthesizer::setAudioParameters(const AudioParameters &params) {
    std::lock_guard<std::mutex> lock(m_lock0);
    m_audioParameters = params;
}
