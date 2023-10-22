/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "champsim.h"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <vector>

#include "environment.h"
#include "ooo_cpu.h"
#include "operable.h"
#include "phase_info.h"
#include "tracereader.h"
#include <fmt/chrono.h>
#include <fmt/core.h>

constexpr int DEADLOCK_CYCLE{500};

auto start_time = std::chrono::steady_clock::now();

std::chrono::seconds elapsed_time() { return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time); }

namespace champsim
{
phase_stats do_phase(phase_info phase, environment& env, std::vector<tracereader>& traces)
{
  auto [phase_name, is_warmup, length, trace_index, trace_names] = phase;
  auto operables = env.operable_view();

  // Initialize phase
  for (champsim::operable& op : operables) {
    op.warmup = is_warmup;
    op.begin_phase();
  }

  // Perform phase
  int stalled_cycle{0};
  std::vector<bool> phase_complete(std::size(env.cpu_view()), false);
  while (!std::accumulate(std::begin(phase_complete), std::end(phase_complete), true, std::logical_and{})) {
    auto next_phase_complete = phase_complete;

    // Operate
    long progress{0};
    for (champsim::operable& op : operables) {
      progress += op._operate();
    }

    if (progress == 0) {
      ++stalled_cycle;
    } else {
      stalled_cycle = 0;
    }

    if (stalled_cycle >= DEADLOCK_CYCLE) {
      std::for_each(std::begin(operables), std::end(operables), [](champsim::operable& c) { c.print_deadlock(); });
      abort();
    }


    /*
      这段代码将 operables 容器按照 leap_operation 成员进行升序排序。
      原因在于 lambda 函数的比较操作：lhs.leap_operation < rhs.leap_operation。
      这个操作检查左侧元素（lhs）的 leap_operation 是否小于右侧元素（rhs）的 leap_operation。
      如果是，那么函数返回 true，意味着 lhs 应该在排序后的序列中位于 rhs 的前面。
      这样，std::sort 函数就会按照 leap_operation 的值从小到大（即升序）对 operables 容器进行排序。
    */
    std::sort(std::begin(operables), std::end(operables),
              [](const champsim::operable& lhs, const champsim::operable& rhs) { return lhs.leap_operation < rhs.leap_operation; });

    // Read from trace
    /*
      std::vector<std::reference_wrapper<O3_CPU>> cpu_view() override {
        return {
          std::ref(cpu0)
        };
      }
    */
    for (O3_CPU& cpu : env.cpu_view()) {
      /*
        std::vector 的 at 方法用于访问向量中指定位置的元素。它接受一个参数，即元素的索引，返回该位置的元素的引用。
        at 方法的一个特性是，如果提供的索引超出了向量的范围（即，索引大于或等于向量的大小），它会抛出一个 std::out_of_range 异常。
        这与 operator[] 不同，operator[] 不会检查索引是否有效，如果提供的索引超出了向量的范围，它的行为是未定义的
      */ 
      auto& trace = traces.at(trace_index.at(cpu.cpu));
      for (auto pkt_count = cpu.IN_QUEUE_SIZE - static_cast<long>(std::size(cpu.input_queue)); !trace.eof() && pkt_count > 0; --pkt_count)
        cpu.input_queue.push_back(trace());

      // If any trace reaches EOF, terminate all phases
      if (trace.eof())
        std::fill(std::begin(next_phase_complete), std::end(next_phase_complete), true);
    }

    // Check for phase finish
    for (O3_CPU& cpu : env.cpu_view()) {
      // Phase complete
      next_phase_complete[cpu.cpu] = next_phase_complete[cpu.cpu] || (cpu.sim_instr() >= length);
    }

    for (O3_CPU& cpu : env.cpu_view()) {
      if (next_phase_complete[cpu.cpu] != phase_complete[cpu.cpu]) {
        for (champsim::operable& op : operables)
          op.end_phase(cpu.cpu);

        fmt::print("{} finished CPU {} instructions: {} cycles: {} cumulative IPC: {:.4g} (Simulation time: {:%H hr %M min %S sec})\n", phase_name, cpu.cpu,
                   cpu.sim_instr(), cpu.sim_cycle(), std::ceil(cpu.sim_instr()) / std::ceil(cpu.sim_cycle()), elapsed_time());
      }
    }

    phase_complete = next_phase_complete;
  }

  for (O3_CPU& cpu : env.cpu_view()) {
    fmt::print("{} complete CPU {} instructions: {} cycles: {} cumulative IPC: {:.4g} (Simulation time: {:%H hr %M min %S sec})\n", phase_name, cpu.cpu,
               cpu.sim_instr(), cpu.sim_cycle(), std::ceil(cpu.sim_instr()) / std::ceil(cpu.sim_cycle()), elapsed_time());
  }

  phase_stats stats;
  stats.name = phase.name;

  for (std::size_t i = 0; i < std::size(trace_index); ++i)
    stats.trace_names.push_back(trace_names.at(trace_index.at(i)));

  auto cpus = env.cpu_view();
  std::transform(std::begin(cpus), std::end(cpus), std::back_inserter(stats.sim_cpu_stats), [](const O3_CPU& cpu) { return cpu.sim_stats; });
  std::transform(std::begin(cpus), std::end(cpus), std::back_inserter(stats.roi_cpu_stats), [](const O3_CPU& cpu) { return cpu.roi_stats; });

  auto caches = env.cache_view();
  std::transform(std::begin(caches), std::end(caches), std::back_inserter(stats.sim_cache_stats), [](const CACHE& cache) { return cache.sim_stats; });
  std::transform(std::begin(caches), std::end(caches), std::back_inserter(stats.roi_cache_stats), [](const CACHE& cache) { return cache.roi_stats; });

  auto dram = env.dram_view();
  std::transform(std::begin(dram.channels), std::end(dram.channels), std::back_inserter(stats.sim_dram_stats),
                 [](const DRAM_CHANNEL& chan) { return chan.sim_stats; });
  std::transform(std::begin(dram.channels), std::end(dram.channels), std::back_inserter(stats.roi_dram_stats),
                 [](const DRAM_CHANNEL& chan) { return chan.roi_stats; });

  return stats;
}

// simulation entry point
std::vector<phase_stats> main(environment& env, std::vector<phase_info>& phases, std::vector<tracereader>& traces)
{
  for (champsim::operable& op : env.operable_view())
    op.initialize();

  std::vector<phase_stats> results;
  for (auto phase : phases) {
    auto stats = do_phase(phase, env, traces);
    if (!phase.is_warmup)
      results.push_back(stats);
  }

  return results;
}
} // namespace champsim
