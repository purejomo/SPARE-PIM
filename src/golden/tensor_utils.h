#ifndef SPARE_PIM_GOLDEN_TENSOR_UTILS_H
#define SPARE_PIM_GOLDEN_TENSOR_UTILS_H

#include <vector>
#include <string>
#include "fp16.h"

namespace Ramulator {
namespace Golden {

// Random tensor generation
void generate_random_tensor(std::vector<fp16>& tensor, size_t size, float min_val, float max_val);
void generate_q_tensor(std::vector<fp16>& q, size_t batch, size_t num_heads, size_t seq_len, size_t d_head);
void generate_k_tensor(std::vector<fp16>& k, size_t batch, size_t num_kv_heads, size_t seq_len, size_t d_head);
void generate_v_tensor(std::vector<fp16>& v, size_t batch, size_t num_kv_heads, size_t seq_len, size_t d_head);
void generate_we_tensor(std::vector<fp16>& we, size_t batch, size_t num_heads, size_t seq_len);

// Output dumping utilities
void dump_fp16_tensor(const std::string& filename, const std::vector<fp16>& tensor, size_t rows, size_t cols);
void dump_fp32_tensor(const std::string& filename, const std::vector<float>& tensor, size_t rows, size_t cols);
void dump_bitmask(const std::string& filename, const std::vector<uint8_t>& bitmask, size_t rows, size_t cols);

// Comparison utility
bool compare_tensors(const std::vector<float>& hw_output, const std::vector<float>& golden_output, float abs_tol, float rel_tol);

} // namespace Golden
} // namespace Ramulator

#endif // SPARE_PIM_GOLDEN_TENSOR_UTILS_H
