#ifndef ATG_ENGINE_SIM_WAVE_READER_H
#define ATG_ENGINE_SIM_WAVE_READER_H
#include <cstdint>
#include <string>
#include <vector>
// Read at most 10,000 mono PCM16 taps at the requested rate. Malformed,
// unsupported or missing files return false and leave samples empty.
bool readImpulseWave(const std::string &path, unsigned sampleRate, std::vector<int16_t> &samples);
#endif
