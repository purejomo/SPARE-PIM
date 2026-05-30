#ifndef SPARE_PIM_BANK_STORAGE_H
#define SPARE_PIM_BANK_STORAGE_H

#include <vector>
#include <unordered_map>
#include <cstdint>
#include "../../golden/fp16.h"

namespace Ramulator {

class BankStorageManager {
public:
    static constexpr size_t FP16_PER_COLUMN = 16; // 32 bytes per column = 16 FP16 elements

    // Pack physical address to a single uint64_t key
    static uint64_t pack_addr(int channel, int pch, int bg, int bank, int row, int col) {
        uint64_t key = 0;
        key |= (static_cast<uint64_t>(channel) & 0xFF) << 56;
        key |= (static_cast<uint64_t>(pch) & 0xFF) << 48;
        key |= (static_cast<uint64_t>(bg) & 0xFF) << 40;
        key |= (static_cast<uint64_t>(bank) & 0xFF) << 32;
        key |= (static_cast<uint64_t>(row) & 0xFFFF) << 16;
        key |= (static_cast<uint64_t>(col) & 0xFFFF);
        return key;
    }

    void write_col(int channel, int pch, int bg, int bank, int row, int col, const std::vector<Golden::fp16>& data) {
        uint64_t key = pack_addr(channel, pch, bg, bank, row, col);
        m_storage[key] = data;
        m_storage[key].resize(FP16_PER_COLUMN); // Ensure exact size
    }

    const std::vector<Golden::fp16>& read_col(int channel, int pch, int bg, int bank, int row, int col) {
        uint64_t key = pack_addr(channel, pch, bg, bank, row, col);
        if (m_storage.find(key) == m_storage.end()) {
            m_storage[key].resize(FP16_PER_COLUMN, Golden::fp16(0.0f));
        }
        return m_storage[key];
    }

    void clear() {
        m_storage.clear();
    }

private:
    std::unordered_map<uint64_t, std::vector<Golden::fp16>> m_storage;
};

} // namespace Ramulator

#endif // SPARE_PIM_BANK_STORAGE_H
