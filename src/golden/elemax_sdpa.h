#ifndef SPARE_PIM_GOLDEN_ELEMAX_SDPA_H
#define SPARE_PIM_GOLDEN_ELEMAX_SDPA_H

#include <vector>
#include <cstdint>
#include "fp16.h"

namespace Ramulator {
namespace Golden {

class ElemaxSDPA {
public:
    ElemaxSDPA(size_t batch, size_t num_heads, size_t num_kv_heads, size_t seq_len, size_t d_head);

    // Run the full ESEF single-pass SDPA logic
    void compute(
        const std::vector<fp16>& q,
        const std::vector<fp16>& k,
        const std::vector<fp16>& v,
        const std::vector<fp16>& we,
        std::vector<float>& out
    );

    // Getters for intermediate results to allow debugging
    const std::vector<fp16>& get_S() const { return m_S; }
    const std::vector<fp16>& get_P() const { return m_P; }
    const std::vector<uint8_t>& get_bitmask() const { return m_bitmask; }

private:
    size_t m_batch;
    size_t m_num_heads;
    size_t m_num_kv_heads;
    size_t m_seq_len;
    size_t m_d_head;

    std::vector<fp16> m_S;       // Q x K^T scores (FP16)
    std::vector<uint8_t> m_bitmask; // 1-bit per token, 1 if ReLU(S) > 0, else 0
    std::vector<fp16> m_P;       // wE * ReLU(S)
};

} // namespace Golden
} // namespace Ramulator

#endif // SPARE_PIM_GOLDEN_ELEMAX_SDPA_H
