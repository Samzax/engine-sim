#include "../include/wave_reader.h"
#include <algorithm>
#include <fstream>
#include <cstring>

namespace {
unsigned u16(const unsigned char *p) { return p[0] | (unsigned(p[1]) << 8); }
uint32_t u32(const unsigned char *p) { return u16(p) | (uint32_t(u16(p + 2)) << 16); }
}

bool readImpulseWave(const std::string &path, unsigned sampleRate, std::vector<int16_t> &samples) {
    samples.clear();
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return false;
    const auto size = file.tellg();
    if (size < 12) return false;
    file.seekg(0);
    unsigned char header[12];
    if (!file.read(reinterpret_cast<char *>(header), 12)
        || std::memcmp(header, "RIFF", 4) || std::memcmp(header + 8, "WAVE", 4)) return false;
    const uint64_t end = uint64_t(u32(header + 4)) + 8;
    if (end > static_cast<uint64_t>(size) || end < 12) return false;
    bool validFormat = false;
    uint64_t dataOffset = 0;
    uint32_t dataSize = 0;
    for (uint64_t offset = 12; offset + 8 <= end;) {
        file.seekg(offset);
        unsigned char chunk[8];
        if (!file.read(reinterpret_cast<char *>(chunk), 8)) return false;
        const uint32_t length = u32(chunk + 4);
        const uint64_t next = offset + 8 + length + (length & 1);
        if (next > end) return false;
        if (std::memcmp(chunk, "fmt ", 4) == 0) {
            unsigned char format[16];
            if (length < 16 || !file.read(reinterpret_cast<char *>(format), 16)) return false;
            validFormat = u16(format) == 1 && u16(format + 2) == 1
                && u32(format + 4) == sampleRate && u16(format + 12) == 2 && u16(format + 14) == 16;
            if (!validFormat) return false;
        } else if (std::memcmp(chunk, "data", 4) == 0) {
            dataOffset = offset + 8;
            dataSize = length;
        }
        offset = next;
    }
    if (!validFormat || dataOffset == 0 || dataSize == 0 || dataSize % 2) return false;
    const unsigned count = std::min(10000U, dataSize / 2);
    std::vector<unsigned char> bytes(count * 2);
    file.seekg(dataOffset);
    if (!file.read(reinterpret_cast<char *>(bytes.data()), bytes.size())) return false;
    samples.resize(count);
    for (unsigned i = 0; i < count; ++i) {
        const unsigned value = u16(bytes.data() + 2 * i);
        samples[i] = static_cast<int16_t>(value < 32768 ? int(value) : int(value) - 65536);
    }
    return true;
}
