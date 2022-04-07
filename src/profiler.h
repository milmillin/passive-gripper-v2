#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

inline long long Tick() {
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

struct ProfilingInfo {
  using Key = std::pair<long long, std::string>;
  struct _Info {
    std::vector<long long> durations;
    long long last_tick;
    bool is_open;

    _Info() : durations(), last_tick(0), is_open(false) {}

    void Open(long long tick) {
      is_open = true;
      last_tick = tick;
    }

    void Close(long long tick) {
      if (is_open) {
        is_open = false;
        durations.push_back(tick - last_tick);
      }
    }
  };

  std::map<Key, _Info> infos;

  void Open(const Key& key) { infos[key].Open(Tick()); }
  void Close(const Key& key) { infos[key].Close(Tick()); }
};

extern std::map<std::thread::id, ProfilingInfo> gInfos;

inline void ProfOpen(const ProfilingInfo::Key& key) {
  gInfos[std::this_thread::get_id()].Open(key);
}

inline void ProfClose(const ProfilingInfo::Key& key) {
  gInfos[std::this_thread::get_id()].Close(key);
}

// not thread safe
inline void ProfDumpToFile(const std::string& filename) {
  std::map<ProfilingInfo::Key, ProfilingInfo::_Info> combined;
  for (const auto& kv : gInfos) {
    for (const auto& info : kv.second.infos) {
      auto& entries = combined[info.first];
      entries.durations.insert(entries.durations.begin(),
                               info.second.durations.begin(),
                               info.second.durations.end());
    }
  }

  std::ofstream f(filename);
  for (const auto& kv : combined) {
    for (long long entry : kv.second.durations) {
      f << kv.first.first << "\t" << kv.first.second << "\t" << entry << "\n";
    }
  }
}

inline void ProfClear() {
  gInfos.clear();
}

#ifdef USE_PROF
#define PROF_OPEN(x, y) ProfOpen({x, y})
#define PROF_CLOSE(x, y) ProfClose({x, y})
#define PROF_DUMP(x) ProfDumpToFile(x)
#define PROF_CLEAR() ProfClear();
#else
#define PROF_OPEN(x, y)
#define PROF_CLOSE(x, y)
#define PROF_DUMP(x)
#define PROF_CLEAR()
#endif
