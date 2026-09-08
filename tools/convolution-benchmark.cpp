#include "../include/convolution_filter.h"
#include "../include/wave_reader.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

// Manual benchmark, deliberately excluded from CTest: wall-clock throughput
// depends on the machine and is not a correctness assertion.
int main(int argc, char **argv) {
    std::vector<int16_t> impulse;
    if (argc != 2 || !readImpulseWave(argv[1], 44100, impulse)) {
        std::cerr << "Usage: engine-sim-convolution-benchmark <mono-44100Hz-PCM16.wav>\n";
        return 1;
    }
    constexpr int count = 44100;
    std::vector<float> input(count);
    for (int i = 0; i < count; ++i) input[i] = std::sin(i * 0.03f);
    for (int run = 0; run < 3; ++run) {
        ConvolutionFilter filter;
        filter.initialize(static_cast<int>(impulse.size()));
        for (size_t i = 0; i < impulse.size(); ++i) filter.getImpulseResponse()[i] = impulse[i] / 32768.0f;
        double checksum = 0;
        const auto start = std::chrono::steady_clock::now();
        for (float sample : input) checksum += filter.f(sample);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << impulse.size() << " taps, one channel, 1s audio: " << seconds
            << "s CPU wall time; checksum=" << checksum << '\n';
    }
}
