#ifndef SPARE_PIM_GOLDEN_FP16_H
#define SPARE_PIM_GOLDEN_FP16_H

#include <cstdint>
#include <cstring>

namespace Ramulator {
namespace Golden {

// Minimal FP16 wrapper for IEEE 754 half-precision conversion.
struct fp16 {
    uint16_t v;

    fp16() : v(0) {}

    // Convert from float to FP16
    fp16(float f) {
        uint32_t x;
        std::memcpy(&x, &f, sizeof(float));
        uint32_t sign = (x >> 16) & 0x8000;
        int32_t exp = ((x >> 23) & 0xff) - 127 + 15;
        uint32_t frac = x & 0x7fffff;
        
        if ((x & 0x7fffffff) == 0) {
            v = sign;
        } else if (((x >> 23) & 0xff) == 0xff) { // NaN or Inf
            v = sign | 0x7c00 | (frac ? 1 : 0);
        } else {
            if (exp >= 31) { // Overflow -> Infinity
                v = sign | 0x7c00;
            } else if (exp <= 0) { // Underflow -> Subnormal or Zero
                if (exp < -10) {
                    v = sign;
                } else {
                    frac = (frac | 0x800000) >> (1 - exp);
                    v = sign | (frac >> 13);
                }
            } else {
                v = sign | (exp << 10) | (frac >> 13);
            }
        }
    }

    // Convert from FP16 to float
    operator float() const {
        uint32_t sign = (v & 0x8000) << 16;
        uint32_t exp = (v & 0x7c00) >> 10;
        uint32_t frac = (v & 0x03ff);

        uint32_t x;
        if (exp == 0) {
            if (frac == 0) {
                x = sign;
            } else {
                // Subnormal
                int32_t e = 127 - 14;
                while ((frac & 0x0400) == 0) {
                    frac <<= 1;
                    e--;
                }
                frac &= 0x03ff;
                x = sign | (e << 23) | (frac << 13);
            }
        } else if (exp == 0x1f) { // NaN or Inf
            x = sign | 0x7f800000 | (frac ? 0x00400000 : 0);
        } else {
            x = sign | ((exp + 127 - 15) << 23) | (frac << 13);
        }
        
        float f;
        std::memcpy(&f, &x, sizeof(uint32_t));
        return f;
    }
};

} // namespace Golden
} // namespace Ramulator

#endif // SPARE_PIM_GOLDEN_FP16_H
