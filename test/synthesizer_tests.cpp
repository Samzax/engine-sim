#include <gtest/gtest.h>
#include "../include/synthesizer.h"
#include <stdexcept>

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
