#include "../include/synthesizer.h"
#include "../include/delay_filter.h"
#include "../include/wave_reader.h"
#include <atomic>
#include <chrono>
#include <deque>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #condition); } while (false)

void ringTests() {
    RingBuffer<int> ring;
    ring.initialize(8);
    for (int i = 0; i < 8; ++i) ring.write(i);
    CHECK(ring.size() == 8);
    ring.write(8);
    CHECK(ring.read(0) == 1);
    ring.overwrite(42, 7);
    CHECK(ring.read(7) == 42);
    ring.initialize(8);
    std::deque<int> expected;
    std::mt19937 rng(17);
    for (int i = 0; i < 10000; ++i) {
        if (expected.empty() || rng() % 2) {
            ring.write(i);
            if (expected.size() == 8) expected.pop_front();
            expected.push_back(i);
        } else {
            const int n = 1 + rng() % expected.size();
            int values[8];
            ring.readAndRemove(n, values);
            for (int j = 0; j < n; ++j) {
                CHECK(values[j] == expected.front());
                expected.pop_front();
            }
        }
        CHECK(ring.size() == expected.size());
    }
}

void convolutionTests() {
    ConvolutionFilter filter;
    CHECK(filter.f(0.5f) == 0.5f);
    filter.initialize(0);
    CHECK(filter.f(0.5f) == 0.5f);
    filter.initialize(1);
    filter.getImpulseResponse()[0] = 1;
    CHECK(filter.f(0.5f) == 0.5f);
    filter.initialize(3);
    filter.getImpulseResponse()[0] = 1;
    filter.getImpulseResponse()[1] = 0.5f;
    filter.getImpulseResponse()[2] = 0.25f;
    CHECK(filter.f(1) == 1);
    CHECK(filter.f(0) == 0.5f);
    CHECK(filter.f(0) == 0.25f);
    CHECK(filter.f(0) == 0);
    filter.destroy();
    filter.destroy();
    CHECK(filter.f(0.25f) == 0.25f);
    // Compare odd lengths, SIMD tails and ring wrap against causal convolution
    // accumulated in double precision, independently of storage layout.
    std::mt19937 random(91);
    std::uniform_real_distribution<float> distribution(-0.5f, 0.5f);
    for (int taps : {1, 3, 4, 7, 8, 31, 257, 10000}) {
        filter.initialize(taps);
        std::vector<float> impulse(taps), input(2 * taps + 11);
        for (int i = 0; i < taps; ++i) filter.getImpulseResponse()[i] = impulse[i] = distribution(random);
        for (int n = 0; n < static_cast<int>(input.size()); ++n) {
            input[n] = distribution(random);
            double expected = 0;
            for (int k = 0; k < taps && k <= n; ++k) expected += double(impulse[k]) * input[n - k];
            CHECK(std::abs(filter.f(input[n]) - expected) < 1E-4);
        }
    }
}

void waveTests() {
    const auto path = std::filesystem::temp_directory_path() /
        ("engine-sim-wave-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".wav");
    std::vector<int16_t> samples;
    CHECK(!readImpulseWave(path.string(), 44100, samples));
    // Minimal PCM16 mono file with one positive and one negative sample.
    std::vector<unsigned char> wave = {
        'R','I','F','F',40,0,0,0,'W','A','V','E',
        'f','m','t',' ',16,0,0,0,1,0,1,0,68,172,0,0,136,88,1,0,2,0,16,0,
        'd','a','t','a',4,0,0,0,255,127,0,128};
    auto write = [&] {
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char *>(wave.data()), wave.size());
    };
    write();
    CHECK(readImpulseWave(path.string(), 44100, samples));
    CHECK(samples.size() == 2 && samples[0] == 32767 && samples[1] == -32768);
    CHECK(!readImpulseWave(path.string(), 48000, samples));
    CHECK(samples.empty());
    wave[22] = 2; // unsupported stereo
    write();
    CHECK(!readImpulseWave(path.string(), 44100, samples));
    wave[22] = 1;
    wave[16] = 255; // oversized format chunk must never overrun a fixed buffer
    write();
    CHECK(!readImpulseWave(path.string(), 44100, samples));
    wave.resize(20); // truncated file
    write();
    CHECK(!readImpulseWave(path.string(), 44100, samples));
    std::filesystem::remove(path);
}

void delayTests() {
    for (int rate : {10000, 20000, 40000}) {
        DelayFilter delay;
        delay.initialize(0.01, rate);
        for (int i = 0; i <= rate / 100 + 1; ++i) {
            CHECK(delay.fast_f(i == 0 ? 1 : 0) == (i == rate / 100 ? 1 : 0));
        }
    }
    DelayFilter delay;
    delay.initialize(0.01, 10000);
    for (int i = 0; i < 100; ++i) delay.fast_f(0);
    delay.fast_f(1);
    for (int i = 0; i < 25; ++i) delay.fast_f(0);
    delay.setSampleRate(20000);
    int peak = -1;
    double amplitude = 0;
    for (int i = 0; i < 220; ++i) {
        const double sample = delay.fast_f(0);
        if (sample > amplitude) { peak = i; amplitude = sample; }
    }
    CHECK(peak >= 147 && peak <= 150);
    CHECK(amplitude > 0.9);
}

Synthesizer::Parameters parameters(int outputCapacity = 1024) {
    Synthesizer::Parameters p;
    p.inputChannelCount = 2;
    p.inputBufferSize = 256;
    p.audioBufferSize = outputCapacity;
    p.inputSampleRate = 44100;
    p.audioSampleRate = 44100;
    p.initialAudioParameters.airNoise = 0;
    p.initialAudioParameters.inputSampleNoise = 0;
    p.initialAudioParameters.dF_F_mix = 0;
    return p;
}

std::vector<int16_t> renderSequence(bool threaded) {
    std::srand(123);
    Synthesizer synth;
    synth.initialize(parameters());
    CHECK(synth.getLatency() == 0);
    const int16_t quiet[] = {0, 12, -100};
    synth.initializeImpulseResponse(quiet, 3, 1, 0);
    synth.initializeImpulseResponse(nullptr, 0, 1, 1);
    CHECK(synth.m_filters[0].convolution.getSampleCount() == 0);
    if (threaded) synth.startAudioRenderingThread();
    std::vector<int16_t> result;
    for (int block = 0; block < 32; ++block) {
        for (int i = 0; i < 64; ++i) {
            const double value = 1000 * std::sin((block * 64 + i) * 0.1);
            const double data[] = {value, value * 0.3};
            synth.writeInput(data);
        }
        synth.endInputBlock();
        if (threaded) synth.waitProcessed();
        else synth.renderAudio();
        int16_t audio[128];
        const int n = synth.readAudioOutput(128, audio);
        result.insert(result.end(), audio, audio + n);
    }
    CHECK(result.size() == 32 * 64 + 1);
    synth.destroy();
    return result;
}

void threadedStress() {
    Synthesizer synth;
    synth.initialize(parameters(17)); // force output-full waits and frequent wraparound
    synth.startAudioRenderingThread();
    std::atomic<bool> finished{false};
    std::atomic<int> consumed{0};
    std::thread consumer([&] {
        int16_t audio[13];
        while (!finished) {
            consumed += synth.readAudioOutput(13, audio);
            std::this_thread::yield();
        }
    });
    std::thread settings([&] {
        for (int i = 0; !finished; ++i) {
            auto p = synth.getAudioParameters();
            p.volume = (i % 10) * 0.1f;
            p.airNoise = (i % 2) * 0.5f;
            synth.setAudioParameters(p);
            CHECK(std::isfinite(synth.getLevelerGain()));
            CHECK(std::isfinite(synth.getLatency()));
            std::this_thread::yield();
        }
    });
    for (int block = 0; block < 500; ++block) {
        for (int i = 0; i < 64; ++i) {
            double data[] = {100.0, 200.0};
            synth.writeInput(data);
        }
        synth.endInputBlock();
        synth.waitProcessed();
    }
    finished = true;
    consumer.join();
    settings.join();
    int16_t remaining[17];
    consumed += synth.readAudioOutput(17, remaining);
    CHECK(consumed == 500 * 64 + 1);
    synth.endAudioRenderingThread();
    synth.destroy();
}

void noiseChannelTests() {
    const auto p = parameters();
    ButterworthLowPassFilter<float> reference[2];
    for (auto &filter : reference) filter.setCutoffFrequency(p.initialAudioParameters.airNoiseFrequencyCutoff, p.audioSampleRate);
    std::srand(789);
    for (int sample = 0; sample < 33; ++sample) {
        for (auto &filter : reference) {
            const float noise = 2.0 * ((double)std::rand() / RAND_MAX) - 1.0;
            filter.fast_f(noise);
        }
    }
    std::srand(789);
    Synthesizer synth;
    synth.initialize(p);
    for (int i = 0; i < 32; ++i) {
        double input[] = {1, 2};
        synth.writeInput(input);
    }
    synth.endInputBlock();
    synth.renderAudio();
    for (int i = 0; i < 2; ++i) {
        CHECK(std::abs(synth.m_filters[i].airNoiseLowPass.fast_f(0) - reference[i].fast_f(0)) < 1e-6);
    }
}

void lifecycleTests() {
    Synthesizer synth;
    auto invalid = parameters();
    invalid.inputChannelCount = 0;
    bool rejected = false;
    try { synth.initialize(invalid); } catch (const std::invalid_argument &) { rejected = true; }
    CHECK(rejected);
    for (int cycle = 0; cycle < 100; ++cycle) {
        synth.initialize(parameters(8));
        synth.startAudioRenderingThread();
        synth.startAudioRenderingThread(); // repeated start must not replace a live worker
        if (cycle % 2) {
            for (int i = 0; i < 40; ++i) {
                double input[] = {1, 2};
                synth.writeInput(input);
            }
            synth.endInputBlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        synth.destroy(); // covers both idle and output-full worker shutdown
        synth.destroy();
    }
    synth.initialize(parameters());
    for (int i = 0; i < 1000; ++i) {
        const double data[] = {1, 2};
        synth.writeInput(data);
    }
    synth.endInputBlock();
    synth.renderAudio();
    int16_t output[512];
    CHECK(synth.readAudioOutput(512, output) == 256);
}

int main() {
    try {
        ringTests();
        convolutionTests();
        waveTests();
        delayTests();
        CHECK(renderSequence(false) == renderSequence(true));
        threadedStress();
        noiseChannelTests();
        lifecycleTests();
        std::cout << "Audio, queue, delay, concurrency and lifecycle regressions passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
