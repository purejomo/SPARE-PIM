#include "elemax_sdpa.h"
#include <algorithm>

namespace Ramulator {
namespace Golden {

ElemaxSDPA::ElemaxSDPA(size_t batch, size_t num_heads, size_t num_kv_heads, size_t seq_len, size_t d_head)
    : m_batch(batch), m_num_heads(num_heads), m_num_kv_heads(num_kv_heads),
      m_seq_len(seq_len), m_d_head(d_head) {
    
    size_t score_size = m_batch * m_num_heads * m_seq_len * m_seq_len;
    m_S.resize(score_size);
    m_bitmask.resize(score_size);
    m_P.resize(score_size);
}

void ElemaxSDPA::compute(
    const std::vector<fp16>& q,
    const std::vector<fp16>& k,
    const std::vector<fp16>& v,
    const std::vector<fp16>& we,
    std::vector<float>& out
) {
    size_t out_size = m_batch * m_num_heads * m_seq_len * m_d_head;
    out.assign(out_size, 0.0f);

    for (size_t b = 0; b < m_batch; ++b) {
        for (size_t h = 0; h < m_num_heads; ++h) {
            size_t kv_h = (h * m_num_kv_heads) / m_num_heads; // Grouped-Query mapping
            
            for (size_t sq = 0; sq < m_seq_len; ++sq) {
                // Base index for Q
                size_t q_idx = b * (m_num_heads * m_seq_len * m_d_head) +
                               h * (m_seq_len * m_d_head) +
                               sq * m_d_head;

                for (size_t sk = 0; sk < m_seq_len; ++sk) {
                    // Base index for K
                    size_t k_idx = b * (m_num_kv_heads * m_seq_len * m_d_head) +
                                   kv_h * (m_seq_len * m_d_head) +
                                   sk * m_d_head;

                    // Q x K^T
                    float score = 0.0f;
                    for (size_t d = 0; d < m_d_head; ++d) {
                        float q_val = static_cast<float>(q[q_idx + d]);
                        float k_val = static_cast<float>(k[k_idx + d]);
                        score += q_val * k_val;
                    }
                    
                    // Cast to FP16
                    fp16 score_fp16(score);

                    size_t s_idx = b * (m_num_heads * m_seq_len * m_seq_len) +
                                   h * (m_seq_len * m_seq_len) +
                                   sq * m_seq_len + sk;
                    
                    m_S[s_idx] = score_fp16;

                    // Elemax ReLU
                    float relu_s = std::max(0.0f, static_cast<float>(score_fp16));
                    
                    // Bitmask
                    m_bitmask[s_idx] = (relu_s > 0.0f) ? 1 : 0;

                    // wE * ReLU(S)
                    size_t we_idx = b * (m_num_heads * m_seq_len) +
                                    h * m_seq_len + sq;
                    float we_val = static_cast<float>(we[we_idx]);
                    
                    float p_val = we_val * relu_s;
                    m_P[s_idx] = fp16(p_val);
                }

                // Sparse PV
                for (size_t d = 0; d < m_d_head; ++d) {
                    float pv_acc = 0.0f;
                    
                    for (size_t sk = 0; sk < m_seq_len; ++sk) {
                        size_t s_idx = b * (m_num_heads * m_seq_len * m_seq_len) +
                                       h * (m_seq_len * m_seq_len) +
                                       sq * m_seq_len + sk;
                        
                        // Use bitmask for sparsity
                        if (m_bitmask[s_idx]) {
                            size_t v_idx = b * (m_num_kv_heads * m_seq_len * m_d_head) +
                                           kv_h * (m_seq_len * m_d_head) +
                                           sk * m_d_head + d;
                                           
                            float p_val = static_cast<float>(m_P[s_idx]);
                            float v_val = static_cast<float>(v[v_idx]);
                            pv_acc += p_val * v_val;
                        }
                    }

                    // Accumulate in FP32
                    size_t out_idx = b * (m_num_heads * m_seq_len * m_d_head) +
                                     h * (m_seq_len * m_d_head) +
                                     sq * m_d_head + d;
                    out[out_idx] = pv_acc;
                }
            }
        }
    }
}

} // namespace Golden
} // namespace Ramulator
