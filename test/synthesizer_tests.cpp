#include <gtest/gtest.h>
#include "../include/synthesizer.h"
#include <stdexcept>
#include <cmath>
#include <limits>

TEST(SynthesizerTests, AudioRecoversAfterInvalidInput) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()}) {
        Synthesizer synth;
        Synthesizer::Parameters params;
        params.inputSampleRate = params.audioSampleRate;
        params.initialAudioParameters.inputSampleNoise = 0;
        params.initialAudioParameters.airNoise = 0;
        params.initialAudioParameters.dF_F_mix = 0;
        params.initialAudioParameters.convolution = 0;
        synth.initialize(params);
        synth.writeInput(&invalid);
        for (int i = 0; i < 512; ++i) {
            const double sample = 1000 * std::sin(i * 0.1);
            synth.writeInput(&sample);
        }
        synth.endInputBlock();
        synth.renderAudio();
        int16_t output[1024];
        const int count = synth.readAudioOutput(1024, output);
        EXPECT_GT(count, 0);
        int audibleSamples = 0;
        for (int i = 0; i < count; ++i) audibleSamples += output[i] != 0;
        EXPECT_GT(audibleSamples, 400);
        synth.destroy();
    }
}

TEST(SynthesizerTests, EmptyImpulseAndRepeatedCleanup) {
    Synthesizer synth;
    Synthesizer::Parameters params;
    synth.initialize(params);
    EXPECT_DOUBLE_EQ(synth.getLatency(), 0);
    const int16_t quiet[] = {0, 1, -50};
    synth.initializeImpulseResponse(quiet, 3, 1, 0);
    for (int i = 0; i < 32; ++i) {
        const double sample[] = {0};
        synth.writeInput(sample);
    }
    synth.endInputBlock();
    synth.renderAudio();
    int16_t output[256];
    const int count = synth.readAudioOutput(256, output);
    EXPECT_GT(count, 0);
    for (int i = 0; i < 256; ++i) EXPECT_EQ(output[i], 0);
    synth.destroy();
    synth.destroy();
}

TEST(SynthesizerTests, RejectZeroChannels) {
    Synthesizer synth;
    Synthesizer::Parameters params;
    params.inputChannelCount = 0;
    EXPECT_THROW(synth.initialize(params), std::invalid_argument);
}
