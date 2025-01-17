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

#include "channel.h"

#include <cassert>

#include "cache.h"
#include "champsim.h"
#include "instruction.h"
#include <fmt/core.h>

champsim::channel::channel(std::size_t rq_size, std::size_t pq_size, std::size_t wq_size, unsigned offset_bits, bool match_offset)
    : RQ_SIZE(rq_size), PQ_SIZE(pq_size), WQ_SIZE(wq_size), OFFSET_BITS(offset_bits), match_offset_bits(match_offset)
{
}

// 如果地址匹配, 就会根据func函数定义的行为来进行搜索
template <typename Iter, typename F>
bool do_collision_for(Iter begin, Iter end, champsim::channel::request_type& packet, unsigned shamt, F&& func)
{
  // is_translated: 是否已经经过了虚拟地址的转化
  // shamt: 表示的是OFFSET_BITS, Cache中进行地址匹配的时候通常是以CacheLine为粒度的,不在乎具体的offset

  // We make sure that both merge packet address have been translated. If
  // not this can happen: package with address virtual and physical X
  // (not translated) is inserted, package with physical address
  // (already translated) X.
  // 需要进行地址的匹配(与Queue中的其他Packet)
  // packet是要进行检查的数据包(目标)，而found是在Queue中找到的地址匹配的包
  if (auto found = std::find_if(begin, end, [addr = packet.address, shamt](const auto& x) { return (x.address >> shamt) == (addr >> shamt); });
      found != end && packet.is_translated == found->is_translated) {
    func(packet, *found);
    return true;
  }

  return false;
}

template <typename Iter>
bool do_collision_for_merge(Iter begin, Iter end, champsim::channel::request_type& packet, unsigned shamt)
{
  return do_collision_for(begin, end, packet, shamt, [](champsim::channel::request_type& source, champsim::channel::request_type& destination) {
    // 这一个是合并的实现函数
    // source: packet，即要检查的数据包
    // destination: found，要插入的地方
    destination.response_requested |= source.response_requested;

    // std::move会将destination.instr_depend_on_me中的数据移动到instr_copy，而不是复制。
    // 这意味着destination.instr_depend_on_me在这之后不应再被使用，因为它的数据已经被移动走了。
    auto instr_copy = std::move(destination.instr_depend_on_me);

    /* std::set_union 的作用：
      std::vector<int> v1 = {1, 2, 3, 4, 5};
      std::vector<int> v2 = {4, 5, 6, 7, 8};
      std::vector<int> dest;

      std::set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(dest));
        ==> 得到的结果：dest 将包含 {1, 2, 3, 4, 5, 6, 7, 8}
    */
    // program_order是自定义的比较条件
    std::set_union(std::begin(instr_copy), std::end(instr_copy), std::begin(source.instr_depend_on_me), std::end(source.instr_depend_on_me),
                   std::back_inserter(destination.instr_depend_on_me), ooo_model_instr::program_order);
  });
}

template <typename Iter>
bool do_collision_for_return(Iter begin, Iter end, champsim::channel::request_type& packet, unsigned shamt,
                             std::deque<champsim::channel::response_type>& returned)
{
  return do_collision_for(begin, end, packet, shamt, [&](champsim::channel::request_type& source, champsim::channel::request_type& destination) {
    if (source.response_requested)
      // std::deque::emplace_back:
      //   在 deque 的尾部直接构造一个元素，而不是先创建一个临时对象，然后再将其复制或移动到容器中。
      //   这种 "就地构造" 的方式通常会提高性能，因为它避免了不必要的拷贝或移动操作
      returned.emplace_back(source.address, source.v_address, destination.data, destination.pf_metadata, source.instr_depend_on_me);
  });
}

void champsim::channel::check_collision()
{
  auto write_shamt = match_offset_bits ? 0 : OFFSET_BITS;
  auto read_shamt = OFFSET_BITS;

  // WQ: WriteQueue 
  // RQ: ReadQueue 
  // PQ: PrefetchQueue

  // Check WQ for duplicates, merging if they are found
  for (auto wq_it = std::find_if(std::begin(WQ), std::end(WQ), std::not_fn(&request_type::forward_checked)); wq_it != std::end(WQ);) {
    if (do_collision_for_merge(std::begin(WQ), wq_it, *wq_it, write_shamt)) {
      // 仿真信息计数
      sim_stats.WQ_MERGED++;
      // 检查到有重复的之后会执行合并操作，然后把那个项给erase掉
      wq_it = WQ.erase(wq_it);
    } else {
      // 标记已经检查过了
      wq_it->forward_checked = true;
      ++wq_it;
    }
  }

  // Check RQ for forwarding from WQ (return if found), then for duplicates (merge if found)
  for (auto rq_it = std::find_if(std::begin(RQ), std::end(RQ), std::not_fn(&request_type::forward_checked)); rq_it != std::end(RQ);) {
    if (do_collision_for_return(std::begin(WQ), std::end(WQ), *rq_it, write_shamt, returned)) {
      // 这里应该是检查RQ和WQ中会不会重叠的情况??
      sim_stats.WQ_FORWARD++;
      rq_it = RQ.erase(rq_it);
    } else if (do_collision_for_merge(std::begin(RQ), rq_it, *rq_it, read_shamt)) {
      sim_stats.RQ_MERGED++;
      rq_it = RQ.erase(rq_it);
    } else {
      rq_it->forward_checked = true;
      ++rq_it;
    }
  }

  // Check PQ for forwarding from WQ (return if found), then for duplicates (merge if found)
  for (auto pq_it = std::find_if(std::begin(PQ), std::end(PQ), std::not_fn(&request_type::forward_checked)); pq_it != std::end(PQ);) {
    if (do_collision_for_return(std::begin(WQ), std::end(WQ), *pq_it, write_shamt, returned)) {
      sim_stats.WQ_FORWARD++;
      pq_it = PQ.erase(pq_it);
    } else if (do_collision_for_merge(std::begin(PQ), pq_it, *pq_it, read_shamt)) {
      sim_stats.PQ_MERGED++;
      pq_it = PQ.erase(pq_it);
    } else {
      pq_it->forward_checked = true;
      ++pq_it;
    }
  }
}

template <typename R>
bool champsim::channel::do_add_queue(R& queue, std::size_t queue_size, const typename R::value_type& packet)
{
  assert(packet.address != 0);

  // check occupancy
  if (std::size(queue) >= queue_size) {
    if constexpr (champsim::debug_print) {
      fmt::print("[channel] {} instr_id: {} address: {:#x} v_address: {:#x} type: {} FULL\n", __func__, packet.instr_id, packet.address, packet.v_address,
          access_type_names.at(champsim::to_underlying(packet.type)));
    }
    return false; // cannot handle this request
  }

  if constexpr (champsim::debug_print) {
    fmt::print("[channel] {} instr_id: {} address: {:#x} v_address: {:#x} type: {}\n", __func__, packet.instr_id, packet.address, packet.v_address,
        access_type_names.at(champsim::to_underlying(packet.type)));
  }

  // Insert the packet ahead of the translation misses
  auto fwd_pkt = packet;
  fwd_pkt.forward_checked = false;
  queue.push_back(fwd_pkt);

  return true;
}

bool champsim::channel::add_rq(const request_type& packet)
{
  sim_stats.RQ_ACCESS++;

  auto result = do_add_queue(RQ, RQ_SIZE, packet);

  if (result)
    sim_stats.RQ_TO_CACHE++;
  else
    sim_stats.RQ_FULL++;

  return result;
}

bool champsim::channel::add_wq(const request_type& packet)
{
  sim_stats.WQ_ACCESS++;

  auto result = do_add_queue(WQ, WQ_SIZE, packet);

  if (result)
    sim_stats.WQ_TO_CACHE++;
  else
    sim_stats.WQ_FULL++;

  return result;
}

bool champsim::channel::add_pq(const request_type& packet)
{
  sim_stats.PQ_ACCESS++;

  auto fwd_pkt = packet;
  auto result = do_add_queue(PQ, PQ_SIZE, fwd_pkt);
  if (result)
    sim_stats.PQ_TO_CACHE++;
  else
    sim_stats.PQ_FULL++;

  return result;
}

std::size_t champsim::channel::rq_occupancy() const { return std::size(RQ); }

std::size_t champsim::channel::wq_occupancy() const { return std::size(WQ); }

std::size_t champsim::channel::pq_occupancy() const { return std::size(PQ); }

std::size_t champsim::channel::rq_size() const { return RQ_SIZE; }

std::size_t champsim::channel::wq_size() const { return WQ_SIZE; }

std::size_t champsim::channel::pq_size() const { return PQ_SIZE; }
