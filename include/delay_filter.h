#ifndef ATG_ENGINE_SIM_DELAY_FILTER_H
#define ATG_ENGINE_SIM_DELAY_FILTER_H

#include "filter.h"

#include "ring_buffer.h"

#include <cmath>
#include <vector>
#include <stdexcept>

class DelayFilter : public Filter {
public:
    DelayFilter() {
        m_latencySamples = 0;
        m_delay = 0;
        m_sampleRate = 0;
    }

    virtual ~DelayFilter() {
        /* void */
    }

    void initialize(double delay, double audioFrequency) {
        if (!std::isfinite(delay) || delay < 0 || !std::isfinite(audioFrequency) || audioFrequency <= 0) {
            throw std::invalid_argument("Invalid delay or sample rate");
        }
        const int samples = static_cast<int>(std::round(delay * audioFrequency));
        const int capacity = samples + 32;

        m_history.initialize(capacity);
        m_latencySamples = samples;
        m_delay = delay;
        m_sampleRate = audioFrequency;
    }

    void setSampleRate(double sampleRate) {
        if (sampleRate == m_sampleRate) return;
        if (!std::isfinite(sampleRate) || sampleRate <= 0) {
            throw std::invalid_argument("Delay sample rate must be positive");
        }
        const int newLatency = static_cast<int>(std::round(m_delay * sampleRate));
        const int oldSize = static_cast<int>(m_history.size());
        const int newSize = m_sampleRate > 0
            ? (std::min)(newLatency, static_cast<int>(std::round(oldSize * sampleRate / m_sampleRate))) : 0;
        std::vector<double> history(newSize);
        for (int i = 0; i < newSize; ++i) {
            const double oldIndex = oldSize - (newSize - i) * m_sampleRate / sampleRate;
            const double bounded = (std::max)(0.0, (std::min)(oldIndex, (double)(oldSize - 1)));
            const int lo = static_cast<int>(bounded);
            const int hi = (std::min)(lo + 1, oldSize - 1);
            history[i] = m_history.read(lo) * (1 - (bounded - lo)) + m_history.read(hi) * (bounded - lo);
        }
        m_history.initialize(newLatency + 32);
        for (double value : history) m_history.write(value);
        m_latencySamples = newLatency;
        m_sampleRate = sampleRate;
    }

    virtual float f(float sample) override {
        return static_cast<float>(fast_f(static_cast<double>(sample)));
    }

    inline double fast_f(double sample) {
        m_history.write(sample);

        if (m_history.size() <= static_cast<size_t>(m_latencySamples)) {
            return 0;
        }
        else {
            double v;
            m_history.readAndRemove(1, &v);

            return v;
        }
    }

protected:
    int m_latencySamples;
    double m_delay;
    double m_sampleRate;
    RingBuffer<double> m_history;
};

#endif /* ATG_ENGINE_SIM_DELAY_FILTER_H */
