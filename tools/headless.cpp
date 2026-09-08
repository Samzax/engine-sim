#include "../scripting/include/compiler.h"
#include "../include/simulator.h"
#include "../include/wave_reader.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
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
    if (argc < 4 || argc > 6) {
        std::cerr << "Usage: engine-sim-headless <script.mr> <es-library-directory> <seconds> [starter-seconds=1] [throttle=0.1]\n";
        return 1;
    }
    try {
        const double duration = std::stod(argv[3]);
        const double starterDuration = argc > 4 ? std::stod(argv[4]) : 1.0;
        const double throttle = argc > 5 ? std::stod(argv[5]) : 0.1;
        if (!std::isfinite(duration) || duration <= 0 || duration > 3600) {
            throw std::invalid_argument("Duration must be between 0 and 3600 seconds");
        }
        if (!std::isfinite(starterDuration) || starterDuration < 0 || starterDuration > duration
            || !std::isfinite(throttle) || throttle < 0 || throttle > 1) {
            throw std::invalid_argument("Starter duration must be within the run; throttle must be 0..1");
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
                std::cerr << "Channel " << i << ": dry audio (missing/unsupported impulse)\n";
            }
            audio.initializeImpulseResponse(impulse.data(), static_cast<unsigned>(impulse.size()),
                response ? static_cast<float>(response->getVolume()) : 1.0f, i);
        }
        simulator->setTargetSynthesizerLatency(0);
        out.engine->getIgnitionModule()->m_enabled = true;
        out.engine->setThrottle(throttle);
        double simulated = 0, energy = 0;
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
                for (int i = 0; i < count; ++i) {
                    const double value = block[i] / 32768.0;
                    energy += value * value;
                    if (block[i] == INT16_MIN || block[i] == INT16_MAX) ++clipped;
                }
            }
            if (!std::isfinite(out.engine->getRpm()) || !std::isfinite(out.engine->getManifoldPressure())) {
                throw std::runtime_error("Non-finite engine state");
            }
        }
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << out.engine->getName() << ": simulated=" << simulated << "s, wall=" << wall
            << "s, rpm=" << out.engine->getRpm() << ", samples=" << samples
            << ", rms=" << (samples ? std::sqrt(energy / samples) : 0)
            << ", clipped=" << clipped << '\n';
        return samples ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
