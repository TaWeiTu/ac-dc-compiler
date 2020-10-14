#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

class Fuzzer {
  std::mt19937 rng;
  std::ofstream ofs;
  std::vector<std::string_view> ivar, fvar;
  std::unordered_set<std::string> pool;

  void GenerateVal();
  void GenerateExpr(uint32_t depth);
  std::string_view GetRandString();

  int max_depth;

public:
  Fuzzer(int seed, std::ofstream s, int max_depth)
      : rng(seed), ofs(std::move(s)), max_depth(max_depth) {}

  void GenerateDecls(int num_decls);
  void GenerateStmts(int num_stmts);
};

void Fuzzer::GenerateVal() { 
  uint32_t choice = rng() % 3; 
  if (choice == 0) {
    uint32_t var = rng() % ivar.size();
    ofs << ivar[var];
  }
  if (choice == 1) {
    uint32_t var = rng() % fvar.size();
    ofs << fvar[var];
  }
  if (choice == 2) {
    uint32_t x = rng() % 64;
    ofs << x;
    if (rng() & 1) {
      ofs << ".";
      uint32_t y = rng() % 10000;
      ofs << y;
    }
  }
}

void Fuzzer::GenerateExpr(uint32_t depth) {
  uint32_t choice = rng() % 6;
  if (depth == 0 || choice == 0) return GenerateVal();

  static char kOp[4] = {'+', '-', '*', '/'};

  if (choice == 5) {
    ofs << "(";
    GenerateExpr(depth - 1);
    ofs << ")";
  } else {
    GenerateExpr(depth - 1);
    ofs << " " << kOp[choice - 1] << " ";
    GenerateExpr(depth - 1);
  }
}

std::string_view Fuzzer::GetRandString() {
  std::string s = "";
  while (true) {
    s += 'a' + rng() % 26;
    if (s == "i" || s == "f" || s == "p") continue;
    auto [iter, found] = pool.insert(s);
    if (found) return *iter;
  }
}

void Fuzzer::GenerateDecls(int num_decls) {
  for (int i = 0; i < num_decls; ++i) {
    std::string_view s = GetRandString();
    ((rng() & 1) ? ivar : fvar).push_back(s);
  }
  if (ivar.empty()) {
    ivar.push_back(fvar.back());
    fvar.pop_back();
  } else if (fvar.empty()) {
    fvar.push_back(ivar.back());
    ivar.pop_back();
  }

  for (auto &s : ivar) ofs << "i " << s << "\n";
  for (auto &s : fvar) ofs << "f " << s << "\n";
}

void Fuzzer::GenerateStmts(int num_stmts) {
  for (int i = 0; i < num_stmts; ++i) {
    uint32_t choice = rng() % 3;
    if (choice == 0) {
      uint32_t var = rng() % (ivar.size() + fvar.size());
      ofs << "p " << (var >= ivar.size() ? fvar[var - ivar.size()] : ivar[var]) << "\n";
    }
    if (choice == 1 || choice == 2) {
      uint32_t var = rng() % fvar.size();
      ofs << fvar[var] << " = ";
      GenerateExpr(max_depth);
      ofs << "\n";
    }
  }
}

int main(int argc, const char **argv) {
  int seed = std::atoi(argv[1]);
  Fuzzer fuzzer(seed, std::ofstream(argv[2]), 10);
  fuzzer.GenerateDecls(std::atoi(argv[3]));
  fuzzer.GenerateStmts(std::atoi(argv[4]));
  return 0;
}
