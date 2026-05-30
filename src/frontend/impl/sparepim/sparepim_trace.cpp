#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include "base/exception.h"
#include "dram/dram.h"
#include "frontend/frontend.h"

namespace Ramulator {

namespace fs = std::filesystem;

class SPAREPIMTrace final : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, SPAREPIMTrace, "SPAREPIMTrace", "SPARE-PIM token/VOC trace frontend.")

  private:
    struct Trace {
      Addr_t addr = 0;
      int max_voc = 0;
      int total_voc = 0;
      int mac_commands = 0;
      int acc_commands = 0;
      int voc_count = 0;
    };

    std::vector<Trace> m_trace;
    size_t m_trace_length = 0;
    size_t m_curr_trace_idx = 0;
    size_t m_issued_requests = 0;
    size_t m_completed_requests = 0;

    std::string m_trace_path;
    Addr_t m_default_addr = 0;
    Addr_t m_addr_stride = 64;
    int m_num_tokens = 0;
    int m_fixed_max_voc = -1;
    int m_fixed_total_voc = -1;
    int m_d_head = 0;
    int m_fsu_width = 16;
    int m_default_mac_commands = 0;
    int m_default_acc_commands = 0;
    int m_sparepim_req_id = -1;
    int m_seq_len = 0;
    int m_expected_voc_count = -1;
    int m_max_voc_per_bpu = -1;
    bool m_validate_voc_count = true;

    Logger_t m_logger;

    size_t s_sparepim_trace_tokens = 0;
    size_t s_sparepim_trace_issued = 0;
    size_t s_sparepim_trace_completed = 0;
    size_t s_sparepim_trace_total_voc = 0;
    size_t s_sparepim_trace_max_voc_sum = 0;

  public:
    void init() override {
      m_clock_ratio = param<uint>("clock_ratio").required();
      m_trace_path = param<std::string>("path").default_val("");
      m_default_addr = param<Addr_t>("default_addr").default_val(0);
      m_addr_stride = param<Addr_t>("addr_stride").default_val(64);
      m_num_tokens = param<int>("num_tokens").default_val(0);
      m_fixed_max_voc = param<int>("fixed_max_voc").default_val(-1);
      m_fixed_total_voc = param<int>("fixed_total_voc").default_val(-1);
      m_d_head = param<int>("d_head").default_val(0);
      m_fsu_width = param<int>("fsu_width").default_val(16);
      m_default_mac_commands = param<int>("mac_commands_per_token").default_val(-1);
      m_default_acc_commands = param<int>("acc_commands_per_token").default_val(0);
      m_seq_len = param<int>("seq_len").default_val(0);
      m_expected_voc_count = param<int>("expected_voc_count").default_val(-1);
      m_max_voc_per_bpu = param<int>("max_voc_per_bpu").default_val(-1);
      m_validate_voc_count = param<bool>("validate_voc_count").default_val(true);

      if (m_default_mac_commands < 0) {
        m_default_mac_commands = m_d_head > 0 ? ceil_div(m_d_head, std::max(1, m_fsu_width)) : 0;
      }
      if (m_max_voc_per_bpu <= 0) {
        m_max_voc_per_bpu = std::max(1, m_fsu_width);
      }

      m_logger = Logging::create_logger("SPAREPIMTrace");

      if (!m_trace_path.empty()) {
        init_trace(m_trace_path);
      } else {
        init_fixed_trace();
      }

      m_trace_length = m_trace.size();
      s_sparepim_trace_tokens = m_trace_length;
      for (const auto& trace : m_trace) {
        s_sparepim_trace_total_voc += trace.total_voc;
        s_sparepim_trace_max_voc_sum += trace.max_voc;
      }

      register_stat(s_sparepim_trace_tokens).name("sparepim_trace_tokens");
      register_stat(s_sparepim_trace_issued).name("sparepim_trace_issued");
      register_stat(s_sparepim_trace_completed).name("sparepim_trace_completed");
      register_stat(s_sparepim_trace_total_voc).name("sparepim_trace_total_voc");
      register_stat(s_sparepim_trace_max_voc_sum).name("sparepim_trace_max_voc_sum");
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      IDRAM* dram = memory_system->get_ifce<IDRAM>();
      m_sparepim_req_id = dram->m_requests("spare-pim");
      if (m_expected_voc_count <= 0) {
        int num_bankgroups = std::max(1, dram->get_level_size("bankgroup"));
        int num_banks = std::max(1, dram->get_level_size("bank"));
        m_expected_voc_count = num_bankgroups * num_banks;
      }
      validate_trace();
    }

    void tick() override {
      if (m_curr_trace_idx >= m_trace_length) {
        return;
      }

      const Trace& t = m_trace[m_curr_trace_idx];
      Request req(t.addr, m_sparepim_req_id, 0, [this](Request&) {
        m_completed_requests++;
        s_sparepim_trace_completed = m_completed_requests;
      });
      req.scratchpad[0] = t.max_voc;
      req.scratchpad[1] = t.total_voc;
      req.scratchpad[2] = t.mac_commands;
      req.scratchpad[3] = t.acc_commands;

      bool request_sent = m_memory_system->send(req);
      if (request_sent) {
        m_curr_trace_idx++;
        m_issued_requests++;
        s_sparepim_trace_issued = m_issued_requests;
      }
    };

    bool is_finished() override {
      return m_curr_trace_idx >= m_trace_length &&
             m_completed_requests >= m_trace_length;
    };

  private:
    static int ceil_div(int lhs, int rhs) {
      return (lhs + rhs - 1) / rhs;
    }

    static std::string trim(std::string text) {
      auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
        return std::isspace(ch);
      });
      auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
        return std::isspace(ch);
      }).base();

      if (first >= last) {
        return "";
      }
      return std::string(first, last);
    }

    static int64_t parse_int(const std::string& token) {
      if (token.compare(0, 2, "0x") == 0 || token.compare(0, 2, "0X") == 0) {
        return std::stoll(token.substr(2), nullptr, 16);
      }
      return std::stoll(token);
    }

    static std::vector<int> parse_voc_list(const std::string& csv) {
      std::vector<int> vocs;
      std::vector<std::string> tokens;
      tokenize(tokens, csv, ",");
      for (const auto& token : tokens) {
        if (!token.empty()) {
          vocs.push_back(std::max<int64_t>(0, parse_int(token)));
        }
      }
      return vocs;
    }

    void init_trace(const std::string& file_path_str) {
      fs::path trace_path(file_path_str);
      if (!fs::exists(trace_path)) {
        throw ConfigurationError("Trace {} does not exist!", file_path_str);
      }

      std::ifstream trace_file(trace_path);
      if (!trace_file.is_open()) {
        throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
      }

      m_logger->info("Loading SPARE-PIM trace file {} ...", file_path_str);

      std::string line;
      int line_num = 0;
      while (std::getline(trace_file, line)) {
        line_num++;
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
          line = line.substr(0, comment_pos);
        }
        line = trim(line);
        if (line.empty()) {
          continue;
        }

        std::vector<std::string> tokens;
        tokenize(tokens, line, " ");
        m_trace.push_back(parse_trace_line(tokens, line_num));
      }

      trace_file.close();
      m_logger->info("Loaded {} SPARE-PIM tokens.", m_trace.size());
    }

    void init_fixed_trace() {
      if (m_num_tokens <= 0) {
        throw ConfigurationError("SPAREPIMTrace requires either path or num_tokens > 0.");
      }

      int max_voc = std::max(0, m_fixed_max_voc);
      int total_voc = m_fixed_total_voc >= 0 ? m_fixed_total_voc : max_voc;

      for (int i = 0; i < m_num_tokens; i++) {
        m_trace.push_back({
          m_default_addr + i * m_addr_stride,
          max_voc,
          total_voc,
          m_default_mac_commands,
          m_default_acc_commands,
          0,
        });
      }
      m_logger->info("Generated {} fixed SPARE-PIM tokens.", m_trace.size());
    }

    Trace parse_trace_line(const std::vector<std::string>& tokens, int line_num) const {
      if (tokens.empty()) {
        throw ConfigurationError("SPARE-PIM trace line {} is empty.", line_num);
      }

      if (tokens[0] == "TOKEN") {
        return parse_token_line(tokens, line_num);
      }
      if (tokens[0] == "VOC") {
        return parse_voc_line(tokens, line_num, 1);
      }
      if (tokens[0].find(',') != std::string::npos) {
        return parse_voc_line(tokens, line_num, 0);
      }
      return parse_bare_line(tokens, line_num);
    }

    Trace parse_token_line(const std::vector<std::string>& tokens, int line_num) const {
      if (tokens.size() >= 6 && tokens[5].find(',') != std::string::npos) {
        Trace trace = make_trace_from_vocs(parse_voc_list(tokens[5]));
        trace.addr = m_default_addr + static_cast<Addr_t>(m_trace.size()) * m_addr_stride;
        if (tokens.size() > 6) {
          trace.mac_commands = parse_int(tokens[6]);
        }
        if (tokens.size() > 7) {
          trace.acc_commands = parse_int(tokens[7]);
        }
        return trace;
      }

      if (tokens.size() < 3) {
        throw ConfigurationError("SPARE-PIM TOKEN trace line {} must be TOKEN max_voc total_voc [addr] [mac] [acc].", line_num);
      }

      Trace trace;
      trace.max_voc = std::max<int64_t>(0, parse_int(tokens[1]));
      trace.total_voc = std::max<int64_t>(0, parse_int(tokens[2]));
      trace.addr = tokens.size() > 3 ? parse_int(tokens[3]) : m_default_addr + static_cast<Addr_t>(m_trace.size()) * m_addr_stride;
      trace.mac_commands = tokens.size() > 4 ? parse_int(tokens[4]) : m_default_mac_commands;
      trace.acc_commands = tokens.size() > 5 ? parse_int(tokens[5]) : m_default_acc_commands;
      return trace;
    }

    Trace parse_voc_line(const std::vector<std::string>& tokens, int line_num, size_t voc_idx) const {
      if (tokens.size() <= voc_idx) {
        throw ConfigurationError("SPARE-PIM VOC trace line {} must include a comma-separated VOC list.", line_num);
      }

      Trace trace = make_trace_from_vocs(parse_voc_list(tokens[voc_idx]));
      size_t next_idx = voc_idx + 1;
      trace.addr = tokens.size() > next_idx ? parse_int(tokens[next_idx]) : m_default_addr + static_cast<Addr_t>(m_trace.size()) * m_addr_stride;
      trace.mac_commands = tokens.size() > next_idx + 1 ? parse_int(tokens[next_idx + 1]) : m_default_mac_commands;
      trace.acc_commands = tokens.size() > next_idx + 2 ? parse_int(tokens[next_idx + 2]) : m_default_acc_commands;
      return trace;
    }

    Trace parse_bare_line(const std::vector<std::string>& tokens, int line_num) const {
      if (tokens.size() < 2) {
        throw ConfigurationError("SPARE-PIM trace line {} must be max_voc total_voc [addr] [mac] [acc].", line_num);
      }

      Trace trace;
      trace.max_voc = std::max<int64_t>(0, parse_int(tokens[0]));
      trace.total_voc = std::max<int64_t>(0, parse_int(tokens[1]));
      trace.addr = tokens.size() > 2 ? parse_int(tokens[2]) : m_default_addr + static_cast<Addr_t>(m_trace.size()) * m_addr_stride;
      trace.mac_commands = tokens.size() > 3 ? parse_int(tokens[3]) : m_default_mac_commands;
      trace.acc_commands = tokens.size() > 4 ? parse_int(tokens[4]) : m_default_acc_commands;
      return trace;
    }

    Trace make_trace_from_vocs(const std::vector<int>& vocs) const {
      Trace trace;
      trace.max_voc = vocs.empty() ? 0 : *std::max_element(vocs.begin(), vocs.end());
      trace.total_voc = std::accumulate(vocs.begin(), vocs.end(), 0);
      trace.mac_commands = m_default_mac_commands;
      trace.acc_commands = m_default_acc_commands;
      trace.voc_count = vocs.size();
      return trace;
    }

    void validate_trace() const {
      for (size_t i = 0; i < m_trace.size(); i++) {
        const Trace& trace = m_trace[i];

        if (m_validate_voc_count &&
            trace.voc_count > 0 &&
            m_expected_voc_count > 0 &&
            trace.voc_count != m_expected_voc_count) {
          throw ConfigurationError(
              "SPARE-PIM trace token {} has {} VOC entries, but expected {} BPU entries per pseudochannel.",
              i,
              trace.voc_count,
              m_expected_voc_count);
        }

        if (trace.max_voc > m_max_voc_per_bpu) {
          throw ConfigurationError(
              "SPARE-PIM trace token {} has max_voc {}, which exceeds max_voc_per_bpu {}.",
              i,
              trace.max_voc,
              m_max_voc_per_bpu);
        }

        if (trace.total_voc > 0 && trace.max_voc > trace.total_voc) {
          throw ConfigurationError(
              "SPARE-PIM trace token {} has max_voc {} larger than total_voc {}.",
              i,
              trace.max_voc,
              trace.total_voc);
        }

        int max_total_voc = m_expected_voc_count * m_max_voc_per_bpu;
        if (max_total_voc > 0 && trace.total_voc > max_total_voc) {
          throw ConfigurationError(
              "SPARE-PIM trace token {} has total_voc {}, which exceeds {} BPU entries x {} max VOC.",
              i,
              trace.total_voc,
              m_expected_voc_count,
              m_max_voc_per_bpu);
        }
      }
    }
};

}        // namespace Ramulator
