#include "tensor_utils.h"
#include <random>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace Ramulator {
namespace Golden {

void generate_random_tensor(std::vector<fp16>& tensor, size_t size, float min_val, float max_val) {
    tensor.resize(size);
    std::mt19937 gen(42); // Deterministic seed for testing
    std::uniform_real_distribution<float> dist(min_val, max_val);
    for (size_t i = 0; i < size; ++i) {
        tensor[i] = fp16(dist(gen));
    }
}

void generate_q_tensor(std::vector<fp16>& q, size_t batch, size_t num_heads, size_t seq_len, size_t d_head) {
    generate_random_tensor(q, batch * num_heads * seq_len * d_head, -1.0f, 1.0f);
}

void generate_k_tensor(std::vector<fp16>& k, size_t batch, size_t num_kv_heads, size_t seq_len, size_t d_head) {
    generate_random_tensor(k, batch * num_kv_heads * seq_len * d_head, -1.0f, 1.0f);
}

void generate_v_tensor(std::vector<fp16>& v, size_t batch, size_t num_kv_heads, size_t seq_len, size_t d_head) {
    generate_random_tensor(v, batch * num_kv_heads * seq_len * d_head, -1.0f, 1.0f);
}

void generate_we_tensor(std::vector<fp16>& we, size_t batch, size_t num_heads, size_t seq_len) {
    generate_random_tensor(we, batch * num_heads * seq_len, 0.0f, 1.0f);
}

void dump_fp16_tensor(const std::string& filename, const std::vector<fp16>& tensor, size_t rows, size_t cols) {
    std::ofstream out(filename);
    if (!out) return;
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            size_t idx = i * cols + j;
            if (idx < tensor.size()) {
                out << std::fixed << std::setprecision(4) << static_cast<float>(tensor[idx]) << (j == cols - 1 ? "" : " ");
            }
        }
        out << "\n";
    }
}

void dump_fp32_tensor(const std::string& filename, const std::vector<float>& tensor, size_t rows, size_t cols) {
    std::ofstream out(filename);
    if (!out) return;
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            size_t idx = i * cols + j;
            if (idx < tensor.size()) {
                out << std::fixed << std::setprecision(4) << tensor[idx] << (j == cols - 1 ? "" : " ");
            }
        }
        out << "\n";
    }
}

void dump_bitmask(const std::string& filename, const std::vector<uint8_t>& bitmask, size_t rows, size_t cols) {
    std::ofstream out(filename);
    if (!out) return;
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            size_t idx = i * cols + j;
            if (idx < bitmask.size()) {
                out << static_cast<int>(bitmask[idx]) << (j == cols - 1 ? "" : " ");
            }
        }
        out << "\n";
    }
}

bool compare_tensors(const std::vector<float>& hw_output, const std::vector<float>& golden_output, float abs_tol, float rel_tol) {
    if (hw_output.size() != golden_output.size()) {
        std::cerr << "[Validator] Size mismatch: " << hw_output.size() << " vs " << golden_output.size() << "\n";
        return false;
    }
    
    bool passed = true;
    for (size_t i = 0; i < hw_output.size(); ++i) {
        float val1 = hw_output[i];
        float val2 = golden_output[i];
        float diff = std::abs(val1 - val2);
        if (diff > abs_tol && diff > rel_tol * std::abs(val2)) {
            std::cerr << "[Validator] Mismatch at index " << i << ": HW=" << val1 << " Golden=" << val2 << "\n";
            passed = false;
            // Optionally limit the number of printed errors
            if (!passed) break; 
        }
    }
    return passed;
}

} // namespace Golden
} // namespace Ramulator
