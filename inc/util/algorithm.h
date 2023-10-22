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

#ifndef UTIL_ALGORITHM_H
#define UTIL_ALGORITHM_H

#include <algorithm>

#include "util/span.h"

namespace champsim
{

/*
  这个 extract_if 模板函数执行了一个分割操作，它接受一个输入序列（由 begin 和 end 迭代器定义）
  和一个输出序列（由 d_begin 迭代器定义），以及一个谓词函数 func。这个函数将输入序列中满足 func 
  的元素移动到输出序列，而不满足 func 的元素则留在输入序列的开始部分。函数返回一个对包含两个迭代器
  的 std::pair，这两个迭代器分别指向输入序列和输出序列中的下一个位置。
*/
template <typename InputIt, typename OutputIt, typename F>
auto extract_if(InputIt begin, InputIt end, OutputIt d_begin, F func)
{
  begin = std::find_if(begin, end, func);
  for (auto i = begin; i != end; ++i) {
    if (func(*i))
      *d_begin++ = std::move(*i);
    else
      *begin++ = std::move(*i);
  }
  return std::pair{begin, d_begin};
}

template <typename R, typename Output, typename F, typename G>
long int transform_while_n(R& queue, Output out, long int sz, F&& test_func, G&& transform_func)
{
  auto [begin, end] = champsim::get_span_p(std::begin(queue), std::end(queue), sz, std::forward<F>(test_func));
  auto retval = std::distance(begin, end);
  std::transform(begin, end, out, std::forward<G>(transform_func));
  queue.erase(begin, end);
  return retval;
}
} // namespace champsim

#endif
