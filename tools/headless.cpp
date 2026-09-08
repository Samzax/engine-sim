#include "../scripting/include/compiler.h"
#include "../include/simulator.h"
#include "../include/wave_reader.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif

namespace {
double parseNumber(const char *text, const char *name) {
    try {
        size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (text[consumed] != '\0' || !std::isfinite(value))
            throw std::invalid_argument("Invalid numeric value");
        return value;
    }
    catch (const std::exception &) {
        throw std::invalid_argument(std::string(name) + " must be a finite number without unit suffixes: " + text);
    }
}

class WaveOutput {
public:
    explicit WaveOutput(const char *path) {
        if (!path) return;
        stream.open(path, std::ios::binary | std::ios::trunc);
        if (!stream) throw std::runtime_error("Cannot open output WAV");
        writeHeader(0);
    }
    void append(const int16_t *data, int count) {
        if (!stream.is_open()) return;
        // The runner's one-hour duration limit stays below RIFF's 4GB limit.
        for (int i = 0; i < count; ++i) writeNumber(static_cast<uint16_t>(data[i]), 2);
        bytes += static_cast<uint32_t>(count) * 2;
        if (!stream) throw std::runtime_error("Writing output WAV failed");
    }
    void finish() {
        if (!stream.is_open()) return;
        stream.seekp(0);
        writeHeader(bytes);
        stream.flush();
        if (!stream) throw std::runtime_error("Finalizing output WAV failed");
    }
private:
    void writeNumber(uint32_t value, int size) {
        for (int i = 0; i < size; ++i) stream.put(static_cast<char>((value >> (8 * i)) & 255));
    }
    void writeHeader(uint32_t size) {
        stream.write("RIFF", 4); writeNumber(36 + size, 4);
        stream.write("WAVEfmt ", 8); writeNumber(16, 4);
        writeNumber(1, 2); writeNumber(1, 2); writeNumber(44100, 4);
        writeNumber(88200, 4); writeNumber(2, 2); writeNumber(16, 2);
        stream.write("data", 4); writeNumber(size, 4);
    }
    std::ofstream stream;
    uint32_t bytes = 0;
};

struct ScriptOwner {
    es_script::Compiler::Output output;
    ~ScriptOwner() {
        if (output.engine) { output.engine->destroy(); delete output.engine; }
        delete output.vehicle;
        delete output.transmission;
    }
};
}

int main(int argc, char **argv) {
#if defined(_MSC_VER) && defined(_DEBUG)
    // Headless diagnostics must report failures without opening desktop dialogs.
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
#endif
    if (argc < 4 || argc > 7) {
        std::cerr << "Usage: engine-sim-headless <script.mr> <es-library-directory> <seconds> [starter-seconds=1] [throttle=0.1] [output.wav]\n";
        return 1;
    }
    try {
        const double duration = parseNumber(argv[3], "Duration");
        const double starterDuration = argc > 4 ? parseNumber(argv[4], "Starter duration")
            : (duration < 1.0 ? duration : 1.0);
        const double throttle = argc > 5 ? parseNumber(argv[5], "Throttle") : 0.1;
        if (!std::isfinite(duration) || duration <= 0 || duration > 3600) {
            throw std::invalid_argument("Duration must be between 0 and 3600 seconds");
        }
        if (!std::isfinite(starterDuration) || starterDuration < 0 || starterDuration > duration
            || !std::isfinite(throttle) || throttle < 0 || throttle > 1) {
            throw std::invalid_argument("Starter duration must be within the run; throttle must be 0..1");
        }
        if (argc > 6) {
            std::error_code pathError;
            if (std::filesystem::equivalent(argv[1], argv[6], pathError))
                throw std::invalid_argument("Output WAV must not overwrite the input engine script");
        }
        ScriptOwner script;
        es_script::Compiler compiler;
        compiler.initialize(std::filesystem::absolute(argv[2]).string());
        if (!compiler.compile(std::filesystem::absolute(argv[1]).string())) {
            compiler.destroy();
            throw std::runtime_error("Script compilation failed; see error_log.log");
        }
        script.output = compiler.execute();
        compiler.destroy();
        auto &out = script.output;
        if (!out.success) throw std::runtime_error("Script execution failed; see error_log.log");
        if (!out.vehicle) {
            Vehicle::Parameters p;
            p.mass = units::mass(1597, units::kg);
            p.diffRatio = 3.42;
            p.tireRadius = units::distance(10, units::inch);
            p.dragCoefficient = 0.25;
            p.crossSectionArea = units::distance(6, units::foot) * units::distance(6, units::foot);
            p.rollingResistance = 2000;
            out.vehicle = new Vehicle;
            out.vehicle->initialize(p);
        }
        if (!out.transmission) {
            const double ratios[] = {2.97, 2.07, 1.43, 1.00, 0.84, 0.56};
            Transmission::Parameters p{6, ratios, units::torque(1000, units::ft_lb)};
            out.transmission = new Transmission;
            out.transmission->initialize(p);
        }
        std::unique_ptr<Simulator> simulator(out.engine->createSimulator(out.vehicle, out.transmission));
        auto &audio = simulator->synthesizer();
        auto parameters = audio.getAudioParameters();
        parameters.inputSampleNoise = static_cast<float>(out.engine->getInitialJitter());
        parameters.airNoise = static_cast<float>(out.engine->getInitialNoise());
        parameters.dF_F_mix = static_cast<float>(out.engine->getInitialHighFrequencyGain());
        audio.setAudioParameters(parameters);
        for (int i = 0; i < out.engine->getExhaustSystemCount(); ++i) {
            const auto *response = out.engine->getExhaustSystem(i)->getImpulseResponse();
            std::vector<int16_t> impulse;
            if (!response || !readImpulseWave(response->getFilename(), 44100, impulse)) {
                std::cerr << "Channel " << i << ": dry audio; impulse response "
                    << (response ? response->getFilename() : "<not configured>")
                    << " is missing or unsupported. Expected mono PCM16 at 44100 Hz.\n";
            }
            audio.initializeImpulseResponse(impulse.data(), static_cast<unsigned>(impulse.size()),
                response ? static_cast<float>(response->getVolume()) : 1.0f, i);
        }
        simulator->setTargetSynthesizerLatency(0);
        out.engine->getIgnitionModule()->m_enabled = true;
        out.engine->setSpeedControl(throttle);
        WaveOutput wave(argc > 6 ? argv[6] : nullptr);
        double simulated = 0, energy = 0, peakRpm = 0;
        uint64_t samples = 0, clipped = 0;
        const auto start = std::chrono::steady_clock::now();
        while (simulated < duration) {
            simulator->m_starterMotor.m_enabled = simulated < starterDuration;
            simulator->startFrame(0.01);
            int steps = 0;
            while (simulator->simulateStep()) ++steps;
            if (steps == 0) throw std::runtime_error("Simulation made no progress");
            simulated += steps * simulator->getTimestep();
            simulator->endFrame();
            for (;;) {
                audio.renderAudio();
                int16_t block[2000];
                const int count = audio.readAudioOutput(2000, block);
                if (count == 0) break;
                samples += count;
                wave.append(block, count);
                for (int i = 0; i < count; ++i) {
                    const double value = block[i] / 32768.0;
                    energy += value * value;
                    if (block[i] == INT16_MIN || block[i] == INT16_MAX) ++clipped;
                }
            }
            if (!std::isfinite(out.engine->getRpm()) || !std::isfinite(out.engine->getManifoldPressure())) {
                throw std::runtime_error("Non-finite engine state");
            }
            peakRpm = std::fmax(peakRpm, std::abs(out.engine->getRpm()));
        }
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        wave.finish();
        double wallTemperature = 0, coolantEnergy = 0;
        for (int i = 0; i < out.engine->getCylinderCount(); ++i) {
            wallTemperature += out.engine->getChamber(i)->getWallTemperature();
            coolantEnergy += out.engine->getChamber(i)->getCoolantEnergy();
        }
        wallTemperature /= out.engine->getCylinderCount();
        std::cout << out.engine->getName() << ": simulated=" << simulated << "s, wall=" << wall
            << "s, rpm=" << out.engine->getRpm() << ", peak_rpm=" << peakRpm << ", samples=" << samples
            << ", rms=" << (samples ? std::sqrt(energy / samples) : 0)
            << ", clipped=" << clipped << ", wall_temperature_K=" << wallTemperature
            << ", coolant_energy_J=" << coolantEnergy << '\n';
        return samples ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
