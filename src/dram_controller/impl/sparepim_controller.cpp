#include <algorithm>
#include <deque>
#include <vector>
#include <iostream>

#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include "bank_storage.h"
#include "../../golden/tensor_utils.h"
#include "../../golden/elemax_sdpa.h"

namespace Ramulator {

class SPAREPIMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, SPAREPIMController, "SPAREPIM", "SPARE-PIM memory controller.");

  private:
    enum class OpKind {
      Command,
      Delay,
      ConfigureSPM,
      Complete,
    };

    struct Op {
      OpKind kind = OpKind::Command;
      int command = -1;
      AddrVec_t addr_vec {};
      int cycles = 0;
      Request request {0, Request::Type::Read};
      bool has_request = false;
    };

    struct VOCSummary {
      int max_voc = 0;
      int total_voc = 0;
      int bgmu_reads = 0;
      int voc_values = 0;
      int max_updates = 0;
    };

    std::deque<Request> m_request_queue;
    std::deque<Op> m_op_queue;

    int m_queue_size = 32;
    int m_mac_commands_per_token = 1;
    int m_acc_commands_per_token = 1;
    int m_spm_cycles_per_voc = 1;
    int m_spm_fixed_cycles = 0;
    bool m_bgmu_read_as_delay = true;
    bool m_enable_act_spm_pre = true;
    bool m_enable_esdm = true;
    bool m_enable_qk_act_pre = true;
    int m_kt_row_offset = 0;
    int m_v_row_offset = 1;
    int m_kt_col_start = 0;
    int m_v_col_start = 0;
    int m_esdm_columns_per_row = -1;

    int m_num_levels = -1;
    int m_channel_level = -1;
    int m_pch_level = -1;
    int m_bg_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_col_level = -1;

    int m_num_pchs = 1;
    int m_num_bgs = 1;
    int m_num_banks = 1;
    int m_num_rows = 1;
    int m_num_cols = 1;

    int m_req_read = -1;
    int m_req_write = -1;
    int m_req_sparepim = -1;
    int m_req_mac = -1;
    int m_req_emul = -1;
    int m_req_mov = -1;
    int m_req_spm = -1;
    int m_req_acc = -1;

    int m_cmd_act = -1;
    int m_cmd_pre = -1;
    int m_cmd_rd = -1;
    int m_cmd_wr = -1;
    int m_cmd_mac = -1;
    int m_cmd_emul = -1;
    int m_cmd_mov = -1;
    int m_cmd_spm = -1;
    int m_cmd_acc = -1;

    bool m_delay_active = false;
    Clk_t m_delay_done_clk = -1;

    size_t s_sparepim_tokens = 0;
    size_t s_sparepim_mac_cmds = 0;
    size_t s_sparepim_mov_cmds = 0;
    size_t s_sparepim_emul_cmds = 0;
    size_t s_sparepim_spm_cmds = 0;
    size_t s_sparepim_acc_cmds = 0;
    size_t s_sparepim_act_cmds = 0;
    size_t s_sparepim_pre_cmds = 0;
    size_t s_sparepim_bgmu_reads = 0;
    size_t s_sparepim_bgmu_voc_values = 0;
    size_t s_sparepim_bgmu_max_updates = 0;
    size_t s_sparepim_bgmu_read_cycles = 0;
    size_t s_sparepim_spm_cycles = 0;
    size_t s_sparepim_skipped_spm = 0;
    size_t s_sparepim_total_voc = 0;
    size_t s_sparepim_max_voc_sum = 0;
    size_t s_sparepim_completed = 0;
    size_t s_sparepim_rejected = 0;
    size_t s_esdm_qk_rows = 0;
    size_t s_esdm_qk_page_hits = 0;
    size_t s_esdm_pv_rows = 0;
    size_t s_esdm_pv_page_hits = 0;
    size_t s_esdm_spm_skipped_act_pre_cmds = 0;

    // Functional Dataflow 
    BankStorageManager m_bank_storage;
    bool m_enable_functional = true;
    int m_seq_len = 256;
    int m_d_head = 64;
    int m_num_heads = 1;
    int m_num_kv_heads = 1;
    int m_batch_size = 1;
    int m_current_sq = 0;

    std::vector<Golden::fp16> m_q_tensor;
    std::vector<Golden::fp16> m_k_tensor;
    std::vector<Golden::fp16> m_v_tensor;
    std::vector<Golden::fp16> m_we_tensor;
    std::vector<float> m_golden_output;
    std::vector<float> m_hw_output;

  public:
    void init() override {
      m_queue_size = param<int>("queue_size").default_val(32);
      m_mac_commands_per_token = param<int>("mac_commands_per_token").default_val(1);
      m_acc_commands_per_token = param<int>("acc_commands_per_token").default_val(1);
      m_spm_cycles_per_voc = param<int>("spm_cycles_per_voc").default_val(1);
      m_spm_fixed_cycles = param<int>("spm_fixed_cycles").default_val(0);
      m_bgmu_read_as_delay = param<bool>("bgmu_read_as_delay").default_val(true);
      m_enable_act_spm_pre = param<bool>("enable_act_spm_pre").default_val(true);
      m_enable_esdm = param<bool>("enable_esdm").default_val(true);
      m_enable_qk_act_pre = param<bool>("enable_qk_act_pre").default_val(true);
      m_kt_row_offset = param<int>("kt_row_offset").default_val(0);
      m_v_row_offset = param<int>("v_row_offset").default_val(1);
      m_kt_col_start = param<int>("kt_col_start").default_val(0);
      m_v_col_start = param<int>("v_col_start").default_val(0);
      m_esdm_columns_per_row = param<int>("esdm_columns_per_row").default_val(-1);
      m_enable_functional = param<bool>("enable_functional").default_val(false);
      m_seq_len = param<int>("seq_len").default_val(256);
      m_d_head = param<int>("d_head").default_val(64);
      m_num_heads = param<int>("num_heads").default_val(1);
      m_num_kv_heads = param<int>("num_kv_heads").default_val(1);
      m_batch_size = param<int>("batch_size").default_val(1);
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();

      m_num_levels = m_dram->m_levels.size();
      m_channel_level = m_dram->m_levels("channel");
      m_pch_level = m_dram->m_levels("pseudochannel");
      m_bg_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");
      m_col_level = m_num_levels - 1;

      m_num_pchs = std::max(1, m_dram->get_level_size("pseudochannel"));
      m_num_bgs = std::max(1, m_dram->get_level_size("bankgroup"));
      m_num_banks = std::max(1, m_dram->get_level_size("bank"));
      m_num_rows = std::max(1, m_dram->get_level_size("row"));
      m_num_cols = std::max(1, m_dram->get_level_size("column"));
      if (m_esdm_columns_per_row <= 0) {
        m_esdm_columns_per_row = m_num_cols;
      }

      m_req_read = Request::Type::Read;
      m_req_write = Request::Type::Write;
      m_req_sparepim = get_request_id("spare-pim");
      m_req_mac = get_request_id("mac");
      m_req_emul = get_request_id("emul");
      m_req_mov = get_request_id("mov");
      m_req_spm = get_request_id("spm");
      m_req_acc = get_request_id("acc");

      m_cmd_act = m_dram->m_commands("ACT");
      m_cmd_pre = m_dram->m_commands("PRE");
      m_cmd_rd = m_dram->m_commands("RD");
      m_cmd_wr = m_dram->m_commands("WR");
      m_cmd_mac = m_dram->m_commands("MAC");
      m_cmd_emul = m_dram->m_commands("EMUL");
      m_cmd_mov = m_dram->m_commands("MOV");
      m_cmd_spm = m_dram->m_commands("SPM");
      m_cmd_acc = m_dram->m_commands("ACC");

      register_stat(s_sparepim_tokens).name("sparepim_tokens_{}", m_channel_id);
      register_stat(s_sparepim_mac_cmds).name("sparepim_mac_cmds_{}", m_channel_id);
      register_stat(s_sparepim_mov_cmds).name("sparepim_mov_cmds_{}", m_channel_id);
      register_stat(s_sparepim_emul_cmds).name("sparepim_emul_cmds_{}", m_channel_id);
      register_stat(s_sparepim_spm_cmds).name("sparepim_spm_cmds_{}", m_channel_id);
      register_stat(s_sparepim_acc_cmds).name("sparepim_acc_cmds_{}", m_channel_id);
      register_stat(s_sparepim_act_cmds).name("sparepim_act_cmds_{}", m_channel_id);
      register_stat(s_sparepim_pre_cmds).name("sparepim_pre_cmds_{}", m_channel_id);
      register_stat(s_sparepim_bgmu_reads).name("sparepim_bgmu_reads_{}", m_channel_id);
      register_stat(s_sparepim_bgmu_voc_values).name("sparepim_bgmu_voc_values_{}", m_channel_id);
      register_stat(s_sparepim_bgmu_max_updates).name("sparepim_bgmu_max_updates_{}", m_channel_id);
      register_stat(s_sparepim_bgmu_read_cycles).name("sparepim_bgmu_read_cycles_{}", m_channel_id);
      register_stat(s_sparepim_spm_cycles).name("sparepim_spm_cycles_{}", m_channel_id);
      register_stat(s_sparepim_skipped_spm).name("sparepim_skipped_spm_{}", m_channel_id);
      register_stat(s_sparepim_total_voc).name("sparepim_total_voc_{}", m_channel_id);
      register_stat(s_sparepim_max_voc_sum).name("sparepim_max_voc_sum_{}", m_channel_id);
      register_stat(s_sparepim_completed).name("sparepim_completed_{}", m_channel_id);
      register_stat(s_sparepim_rejected).name("sparepim_rejected_{}", m_channel_id);
      register_stat(s_esdm_qk_rows).name("sparepim_esdm_qk_rows_{}", m_channel_id);
      register_stat(s_esdm_qk_page_hits).name("sparepim_esdm_qk_page_hits_{}", m_channel_id);
      register_stat(s_esdm_pv_rows).name("sparepim_esdm_pv_rows_{}", m_channel_id);
      register_stat(s_esdm_pv_page_hits).name("sparepim_esdm_pv_page_hits_{}", m_channel_id);
      register_stat(s_esdm_spm_skipped_act_pre_cmds).name("sparepim_esdm_skipped_act_pre_cmds_{}", m_channel_id);

      if (m_enable_functional) {
        setup_functional_data();
      }
    };

    bool send(Request& req) override {
      if (m_request_queue.size() >= static_cast<size_t>(m_queue_size)) {
        s_sparepim_rejected++;
        return false;
      }

      req.arrive = m_clk;

      if (req.type_id == m_req_sparepim) {
        m_request_queue.push_back(req);
        return true;
      }

      if (is_direct_pim_request(req.type_id) || req.type_id == m_req_read || req.type_id == m_req_write) {
        m_request_queue.push_back(req);
        return true;
      }

      s_sparepim_rejected++;
      return false;
    };

    bool priority_send(Request& req) override {
      if (m_request_queue.size() >= static_cast<size_t>(m_queue_size)) {
        s_sparepim_rejected++;
        return false;
      }

      req.arrive = m_clk;
      m_request_queue.push_front(req);
      return true;
    };

    void tick() override {
      m_clk++;

      if (m_op_queue.empty() && !m_request_queue.empty()) {
        Request req = m_request_queue.front();
        m_request_queue.pop_front();
        expand_request(req);
      }

      if (m_op_queue.empty()) {
        return;
      }

      Op& op = m_op_queue.front();
      switch (op.kind) {
        case OpKind::Delay:
          tick_delay(op);
          return;
        case OpKind::ConfigureSPM:
          m_dram->notify("sparepim_spm_latency", op.cycles);
          m_op_queue.pop_front();
          return;
        case OpKind::Complete:
          complete_request(op);
          m_op_queue.pop_front();
          return;
        case OpKind::Command:
          issue_if_ready(op);
          return;
      }
    };

  private:
    int get_request_id(std::string_view name) const {
      try {
        return m_dram->m_requests(name);
      } catch (const std::out_of_range&) {
        return -1;
      }
    }

    bool is_direct_pim_request(int type_id) const {
      return type_id == m_req_mac ||
             type_id == m_req_emul ||
             type_id == m_req_mov ||
             type_id == m_req_spm ||
             type_id == m_req_acc;
    }

    void expand_request(const Request& req) {
      if (req.type_id == m_req_sparepim) {
        expand_sparepim_token(req);
      } else {
        expand_direct_request(req);
      }
    }

    void expand_sparepim_token(const Request& req) {
      s_sparepim_tokens++;

      if (m_enable_functional && m_current_sq < m_seq_len) {
          execute_functional_token(m_current_sq);
          m_current_sq++;
      }

      AddrVec_t base_addr = normalize_addr(req.addr_vec);
      VOCSummary voc_summary = read_bgmu_vocs_and_find_max(req);
      int max_voc = voc_summary.max_voc;
      int total_voc = voc_summary.total_voc;
      int mac_commands = req.scratchpad[2] > 0 ? req.scratchpad[2] : m_mac_commands_per_token;
      int acc_commands = req.scratchpad[3] > 0 ? req.scratchpad[3] : m_acc_commands_per_token;

      if (total_voc == 0 && max_voc > 0) {
        total_voc = max_voc;
      }

      s_sparepim_total_voc += total_voc;
      s_sparepim_max_voc_sum += max_voc;
      s_sparepim_bgmu_max_updates += voc_summary.max_updates;

      enqueue_qk_stage(base_addr, mac_commands);

      AddrVec_t elemax_addr = make_all_bank_addr(make_esdm_addr(base_addr, m_kt_row_offset, m_kt_col_start));
      enqueue_command(m_cmd_mov, elemax_addr);
      enqueue_command(m_cmd_emul, elemax_addr);
      enqueue_bgmu_read_delay(voc_summary.bgmu_reads, voc_summary.voc_values);

      if (max_voc == 0) {
        s_sparepim_skipped_spm++;
        if (m_enable_act_spm_pre) {
          s_esdm_spm_skipped_act_pre_cmds += total_bank_count() * 2;
        }
        enqueue_complete(req);
        return;
      }

      enqueue_sparse_pv_stage(base_addr, max_voc);

      for (int i = 0; i < acc_commands; i++) {
        enqueue_command(m_cmd_acc, make_acc_addr(base_addr, i));
      }

      enqueue_complete(req);
    }

    void expand_direct_request(const Request& req) {
      AddrVec_t addr_vec = normalize_addr(req.addr_vec);
      int command = -1;

      if (req.type_id == m_req_read) {
        command = m_cmd_rd;
      } else if (req.type_id == m_req_write) {
        command = m_cmd_wr;
      } else {
        command = m_dram->m_request_translations(req.type_id);
      }

      enqueue_command(command, addr_vec);
      enqueue_complete(req);
    }

    int ceil_div(int lhs, int rhs) const {
      return (lhs + rhs - 1) / rhs;
    }

    int total_bank_count() const {
      return m_num_pchs * m_num_bgs * m_num_banks;
    }

    VOCSummary read_bgmu_vocs_and_find_max(const Request& req) const {
      VOCSummary summary;
      const auto* vocs = static_cast<const std::vector<int>*>(req.m_payload);
      if (vocs != nullptr && !vocs->empty()) {
        int vocs_per_bgmu = std::max(1, m_num_banks);
        summary.bgmu_reads = ceil_div(vocs->size(), vocs_per_bgmu);

        for (int bg = 0; bg < summary.bgmu_reads; bg++) {
          int base_idx = bg * vocs_per_bgmu;
          for (int bank = 0; bank < vocs_per_bgmu; bank++) {
            int voc_idx = base_idx + bank;
            if (voc_idx >= static_cast<int>(vocs->size())) {
              break;
            }

            int tmp = std::max(0, (*vocs)[voc_idx]);
            summary.voc_values++;
            summary.total_voc += tmp;
            if (tmp > summary.max_voc) {
              summary.max_voc = tmp;
              summary.max_updates++;
            }
          }
        }

        return summary;
      }

      summary.max_voc = std::max(0, req.scratchpad[0]);
      summary.total_voc = std::max(0, req.scratchpad[1]);
      summary.bgmu_reads = m_num_bgs;
      summary.voc_values = m_num_bgs * m_num_banks;
      summary.max_updates = summary.max_voc > 0 ? 1 : 0;
      return summary;
    }

    AddrVec_t make_esdm_addr(AddrVec_t addr_vec, int row_offset, int column) const {
      if (!m_enable_esdm) {
        return addr_vec;
      }

      int columns_per_row = std::max(1, m_esdm_columns_per_row);
      int logical_col = std::max(0, column);
      int row_delta = logical_col / columns_per_row;
      int column_id = logical_col % columns_per_row;
      int base_row = std::max(0, addr_vec[m_row_level]);

      addr_vec[m_row_level] = (base_row + row_offset + row_delta) % m_num_rows;
      addr_vec[m_col_level] = column_id % m_num_cols;
      return addr_vec;
    }

    void enqueue_qk_stage(const AddrVec_t& base_addr, int mac_commands) {
      if (mac_commands <= 0) {
        return;
      }

      if (!m_enable_esdm) {
        AddrVec_t all_bank_addr = make_all_bank_addr(base_addr);
        for (int i = 0; i < mac_commands; i++) {
          enqueue_command(m_cmd_mac, all_bank_addr);
        }
        return;
      }

      int issued = 0;
      while (issued < mac_commands) {
        int row_col = (m_kt_col_start + issued) % std::max(1, m_esdm_columns_per_row);
        int chunk = std::min(mac_commands - issued, std::max(1, m_esdm_columns_per_row) - row_col);
        AddrVec_t row_addr = make_esdm_addr(base_addr, m_kt_row_offset, m_kt_col_start + issued);

        s_esdm_qk_rows++;
        if (chunk > 0) {
          s_esdm_qk_page_hits += chunk - 1;
        }

        if (m_enable_qk_act_pre) {
          enqueue_per_bank_command(m_cmd_act, row_addr);
        }

        for (int i = 0; i < chunk; i++) {
          AddrVec_t mac_addr = make_all_bank_addr(make_esdm_addr(base_addr, m_kt_row_offset, m_kt_col_start + issued + i));
          enqueue_command(m_cmd_mac, mac_addr);
        }

        if (m_enable_qk_act_pre) {
          enqueue_per_bank_command(m_cmd_pre, row_addr);
        }

        issued += chunk;
      }
    }

    void enqueue_sparse_pv_stage(const AddrVec_t& base_addr, int max_voc) {
      if (!m_enable_esdm) {
        int spm_latency = std::max(1, m_spm_fixed_cycles + max_voc * m_spm_cycles_per_voc);
        s_sparepim_spm_cycles += spm_latency;
        enqueue_configure_spm(spm_latency);

        if (m_enable_act_spm_pre) {
          enqueue_per_bank_command(m_cmd_act, base_addr);
        }

        enqueue_command(m_cmd_spm, make_all_bank_addr(base_addr));

        if (m_enable_act_spm_pre) {
          enqueue_per_bank_command(m_cmd_pre, base_addr);
        }
        return;
      }

      int processed = 0;
      while (processed < max_voc) {
        int row_col = (m_v_col_start + processed) % std::max(1, m_esdm_columns_per_row);
        int chunk = std::min(max_voc - processed, std::max(1, m_esdm_columns_per_row) - row_col);
        AddrVec_t row_addr = make_esdm_addr(base_addr, m_v_row_offset, m_v_col_start + processed);
        AddrVec_t all_bank_row_addr = make_all_bank_addr(row_addr);

        s_esdm_pv_rows++;
        if (chunk > 0) {
          s_esdm_pv_page_hits += chunk - 1;
        }

        int spm_latency = std::max(1, m_spm_fixed_cycles + chunk * m_spm_cycles_per_voc);
        s_sparepim_spm_cycles += spm_latency;
        enqueue_configure_spm(spm_latency);

        if (m_enable_act_spm_pre) {
          enqueue_per_bank_command(m_cmd_act, row_addr);
        }

        enqueue_command(m_cmd_spm, all_bank_row_addr);

        if (m_enable_act_spm_pre) {
          enqueue_per_bank_command(m_cmd_pre, row_addr);
        }

        processed += chunk;
      }
    }

    AddrVec_t normalize_addr(AddrVec_t addr_vec) const {
      addr_vec.resize(m_num_levels, -1);
      addr_vec[m_channel_level] = m_channel_id;
      if (addr_vec[m_row_level] < 0) {
        addr_vec[m_row_level] = 0;
      }
      if (addr_vec[m_col_level] < 0) {
        addr_vec[m_col_level] = 0;
      }
      return addr_vec;
    }

    AddrVec_t make_all_bank_addr(AddrVec_t addr_vec) const {
      addr_vec[m_channel_level] = m_channel_id;
      addr_vec[m_pch_level] = -1;
      addr_vec[m_bg_level] = -1;
      addr_vec[m_bank_level] = -1;
      return addr_vec;
    }

    AddrVec_t make_bank_addr(AddrVec_t addr_vec, int pch, int bg, int bank) const {
      addr_vec[m_channel_level] = m_channel_id;
      addr_vec[m_pch_level] = pch;
      addr_vec[m_bg_level] = bg;
      addr_vec[m_bank_level] = bank;
      return addr_vec;
    }

    AddrVec_t make_acc_addr(const AddrVec_t& base_addr, int acc_idx) const {
      int banks_per_pch = m_num_bgs * m_num_banks;
      int total_banks = m_num_pchs * banks_per_pch;
      int bank_idx = total_banks > 0 ? acc_idx % total_banks : 0;
      int pch = bank_idx / banks_per_pch;
      int rem = bank_idx % banks_per_pch;
      int bg = rem / m_num_banks;
      int bank = rem % m_num_banks;
      return make_bank_addr(base_addr, pch, bg, bank);
    }

    void enqueue_command(int command, const AddrVec_t& addr_vec) {
      Op op;
      op.kind = OpKind::Command;
      op.command = command;
      op.addr_vec = addr_vec;
      m_op_queue.push_back(op);
    }

    void enqueue_per_bank_command(int command, const AddrVec_t& base_addr) {
      for (int pch = 0; pch < m_num_pchs; pch++) {
        for (int bg = 0; bg < m_num_bgs; bg++) {
          for (int bank = 0; bank < m_num_banks; bank++) {
            enqueue_command(command, make_bank_addr(base_addr, pch, bg, bank));
          }
        }
      }
    }

    void enqueue_bgmu_read_delay(int bgmu_reads, int voc_values) {
      int read_count = std::max(1, bgmu_reads);
      int bgmu_read_latency = m_dram->m_timing_vals("nCL") +
                              (read_count - 1) * m_dram->m_timing_vals("nCCDS") +
                              m_dram->m_timing_vals("nBL");

      s_sparepim_bgmu_reads += read_count;
      s_sparepim_bgmu_voc_values += voc_values > 0 ? voc_values : read_count * m_num_banks;
      s_sparepim_bgmu_read_cycles += bgmu_read_latency;

      if (!m_bgmu_read_as_delay) {
        return;
      }

      Op op;
      op.kind = OpKind::Delay;
      op.cycles = std::max(1, bgmu_read_latency);
      m_op_queue.push_back(op);
    }

    void enqueue_configure_spm(int spm_latency) {
      Op op;
      op.kind = OpKind::ConfigureSPM;
      op.cycles = spm_latency;
      m_op_queue.push_back(op);
    }

    void enqueue_complete(const Request& req) {
      Op op;
      op.kind = OpKind::Complete;
      op.request = req;
      op.has_request = true;
      m_op_queue.push_back(op);
    }

    void tick_delay(const Op& op) {
      if (!m_delay_active) {
        m_delay_active = true;
        m_delay_done_clk = m_clk + op.cycles;
      }

      if (m_clk >= m_delay_done_clk) {
        m_delay_active = false;
        m_delay_done_clk = -1;
        m_op_queue.pop_front();
      }
    }

    void issue_if_ready(const Op& op) {
      if (!m_dram->check_ready(op.command, op.addr_vec)) {
        return;
      }

      m_dram->issue_command(op.command, op.addr_vec);
      update_command_stats(op.command);
      m_op_queue.pop_front();
    }

    void complete_request(Op& op) {
      if (!op.has_request) {
        return;
      }

      op.request.depart = m_clk;
      s_sparepim_completed++;
      if (op.request.callback) {
        op.request.callback(op.request);
      }
    }

    void update_command_stats(int command) {
      if (command == m_cmd_mac) {
        s_sparepim_mac_cmds++;
      } else if (command == m_cmd_mov) {
        s_sparepim_mov_cmds++;
      } else if (command == m_cmd_emul) {
        s_sparepim_emul_cmds++;
      } else if (command == m_cmd_spm) {
        s_sparepim_spm_cmds++;
      } else if (command == m_cmd_acc) {
        s_sparepim_acc_cmds++;
      } else if (command == m_cmd_act) {
        s_sparepim_act_cmds++;
      } else if (command == m_cmd_pre) {
        s_sparepim_pre_cmds++;
      }
    }

    void setup_functional_data() {
      Golden::generate_q_tensor(m_q_tensor, m_batch_size, m_num_heads, m_seq_len, m_d_head);
      Golden::generate_k_tensor(m_k_tensor, m_batch_size, m_num_kv_heads, m_seq_len, m_d_head);
      Golden::generate_v_tensor(m_v_tensor, m_batch_size, m_num_kv_heads, m_seq_len, m_d_head);
      Golden::generate_we_tensor(m_we_tensor, m_batch_size, m_num_heads, m_seq_len);

      Golden::ElemaxSDPA golden_model(m_batch_size, m_num_heads, m_num_kv_heads, m_seq_len, m_d_head);
      golden_model.compute(m_q_tensor, m_k_tensor, m_v_tensor, m_we_tensor, m_golden_output);
      
      m_hw_output.assign(m_golden_output.size(), 0.0f);

      int fsu_width = BankStorageManager::FP16_PER_COLUMN; 
      AddrVec_t logical_base = normalize_addr({});
      int cols_per_kt = (m_d_head + fsu_width - 1) / fsu_width;

      for (int sk = 0; sk < m_seq_len; ++sk) {
        for (int d = 0; d < m_d_head; d += fsu_width) {
          int chunk = std::min(fsu_width, m_d_head - d);
          std::vector<Golden::fp16> col_data(fsu_width, Golden::fp16(0.0f));
          for (int i = 0; i < chunk; ++i) {
            size_t k_idx = 0 * (m_num_kv_heads * m_seq_len * m_d_head) + 0 + sk * m_d_head + d + i;
            col_data[i] = m_k_tensor[k_idx];
          }
          int logical_col_kt = sk * cols_per_kt + (d / fsu_width);
          AddrVec_t esdm_addr = make_esdm_addr(logical_base, m_kt_row_offset, m_kt_col_start + logical_col_kt);
          
          for (int pch = 0; pch < m_num_pchs; pch++) {
            for (int bg = 0; bg < m_num_bgs; bg++) {
              for (int bank = 0; bank < m_num_banks; bank++) {
                m_bank_storage.write_col(m_channel_id, pch, bg, bank, esdm_addr[m_row_level], esdm_addr[m_col_level], col_data);
              }
            }
          }
        }

        for (int d = 0; d < m_d_head; d += fsu_width) {
          int chunk = std::min(fsu_width, m_d_head - d);
          std::vector<Golden::fp16> v_col_data(fsu_width, Golden::fp16(0.0f));
          for (int i = 0; i < chunk; ++i) {
            size_t v_idx = 0 * (m_num_kv_heads * m_seq_len * m_d_head) + 0 + sk * m_d_head + d + i;
            v_col_data[i] = m_v_tensor[v_idx];
          }
          int logical_col_v = sk * cols_per_kt + (d / fsu_width);
          AddrVec_t esdm_v_addr = make_esdm_addr(logical_base, m_v_row_offset, m_v_col_start + logical_col_v);
          
          for (int pch = 0; pch < m_num_pchs; pch++) {
            for (int bg = 0; bg < m_num_bgs; bg++) {
              for (int bank = 0; bank < m_num_banks; bank++) {
                m_bank_storage.write_col(m_channel_id, pch, bg, bank, esdm_v_addr[m_row_level], esdm_v_addr[m_col_level], v_col_data);
              }
            }
          }
        }
      }
    }

    void execute_functional_token(int sq) {
        int b = 0;
        int h = 0; 
        int kv_h = 0;
        int fsu_width = BankStorageManager::FP16_PER_COLUMN; 
        int cols_per_kt = (m_d_head + fsu_width - 1) / fsu_width;

        AddrVec_t logical_base = normalize_addr({});
        std::vector<Golden::fp16> VB_B(m_seq_len, Golden::fp16(0.0f));
        std::vector<Golden::fp16> VB_A(m_seq_len, Golden::fp16(0.0f));
        std::vector<uint8_t> bitmask(m_seq_len, 0);

        for (int sk = 0; sk < m_seq_len; ++sk) {
            float score = 0.0f;
            for (int d = 0; d < m_d_head; d += fsu_width) {
                int logical_col_kt = sk * cols_per_kt + (d / fsu_width);
                AddrVec_t esdm_addr = make_esdm_addr(logical_base, m_kt_row_offset, m_kt_col_start + logical_col_kt);
                
                auto col_data = m_bank_storage.read_col(m_channel_id, 0, 0, 0, esdm_addr[m_row_level], esdm_addr[m_col_level]);
                
                int chunk = std::min(fsu_width, m_d_head - d);
                for (int i = 0; i < chunk; ++i) {
                    size_t q_idx = b * (m_num_heads * m_seq_len * m_d_head) + h * (m_seq_len * m_d_head) + sq * m_d_head + d + i;
                    float q_val = static_cast<float>(m_q_tensor[q_idx]);
                    float k_val = static_cast<float>(col_data[i]);
                    score += q_val * k_val;
                }
            }
            VB_B[sk] = Golden::fp16(score);
        }

        for (int sk = 0; sk < m_seq_len; ++sk) {
            float relu_s = std::max(0.0f, static_cast<float>(VB_B[sk]));
            bitmask[sk] = (relu_s > 0.0f) ? 1 : 0;

            size_t we_idx = b * (m_num_heads * m_seq_len) + h * m_seq_len + sq;
            float we_val = static_cast<float>(m_we_tensor[we_idx]);
            VB_A[sk] = Golden::fp16(we_val * relu_s);
        }

        for (int d = 0; d < m_d_head; ++d) {
            float pv_acc = 0.0f;
            for (int sk = 0; sk < m_seq_len; ++sk) {
                if (bitmask[sk]) {
                    int logical_col_v = sk * cols_per_kt + (d / fsu_width);
                    AddrVec_t esdm_v_addr = make_esdm_addr(logical_base, m_v_row_offset, m_v_col_start + logical_col_v);
                    auto v_col_data = m_bank_storage.read_col(m_channel_id, 0, 0, 0, esdm_v_addr[m_row_level], esdm_v_addr[m_col_level]);
                    
                    float p_val = static_cast<float>(VB_A[sk]);
                    int v_offset = d % fsu_width; 
                    float v_val = static_cast<float>(v_col_data[v_offset]);
                    
                    pv_acc += p_val * v_val;
                }
            }
            size_t out_idx = b * (m_num_heads * m_seq_len * m_d_head) + h * (m_seq_len * m_d_head) + sq * m_d_head + d;
            m_hw_output[out_idx] = pv_acc;
        }
    }

    void finalize() override {
      if (m_enable_functional) {
        std::cout << "==========================================\n";
        std::cout << "[SPARE-PIM] Finalizing Functional Simulation\n";
        
        // Resize golden output to only compare the tokens actually processed
        size_t processed_elements = m_current_sq * m_d_head;
        m_hw_output.resize(processed_elements);
        m_golden_output.resize(processed_elements);

        bool passed = Golden::compare_tensors(m_hw_output, m_golden_output, 1e-2f, 1e-2f);
        if (passed) {
          std::cout << "[SPARE-PIM] Status: PASSED (Outputs match Golden Model for " << m_current_sq << " tokens)\n";
        } else {
          std::cerr << "[SPARE-PIM] Status: FAILED (Outputs mismatch)\n";
          Golden::dump_fp32_tensor("debug_hw_output.txt", m_hw_output, m_current_sq, m_d_head);
          Golden::dump_fp32_tensor("debug_golden_output.txt", m_golden_output, m_current_sq, m_d_head);
        }
        std::cout << "==========================================\n";
      }
    }
};

}   // namespace Ramulator
