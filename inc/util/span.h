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

#ifndef UTIL_SPAN_H
#define UTIL_SPAN_H

#include <algorithm>
#include <cassert>
#include <iterator>
#include <limits>

namespace champsim
{

// 它接受两个迭代器 begin 和 end，以及一个大小 sz
// 返回一个在 [begin, end) 范围内，大小为 sz 的范围的开始和结束迭代器
template <typename It>
std::pair<It, It> get_span(It begin, It end, typename std::iterator_traits<It>::difference_type sz)
{
  assert(std::distance(begin, end) >= 0);
  assert(sz >= 0);
  // 计算 begin 和 end 之间的距离和 sz 之间的较小值，然后将结果存储在 distance 中。这是为了确保我们不会尝试访问超出 [begin, end) 范围的元素
  auto distance = std::min(std::distance(begin, end), sz);
  return {begin, std::next(begin, distance)};
}

template <typename It, typename F>
std::pair<It, It> get_span_p(It begin, It end, typename std::iterator_traits<It>::difference_type sz, F&& func)
{
  auto [span_begin, span_end] = get_span(begin, end, sz);
  // std::find_if_not 函数在 [span_begin, span_end) 范围内查找第一个不满足 func 条件的元素，然后返回指向该元素的迭代器
  // std::forward<F>(func) 是对 func 的完美转发，这样在调用 func 时可以保持其原有的值类别
  /*
    在 C++ 中，"完美转发"是一种技术，它允许函数模板将其参数传递给其他函数，同时保持原有参数的所有属性，包括是否是左值或右值
      (1) 左值：左值表示一个对象的身份（也就是说它有一个可以引用的地址）。例如，一个变量或数组元素就是一个左值。左值有一个持续的生命周期，它们会一直存在，直到离开其作用域。
      (2) 右值：右值表示一个对象的值，或者是一个将要结束生命周期的对象。右值通常来自于临时对象，如函数返回值，或者字面量（如 42 或 "hello"）。右值的生命周期在表达式结束后就结束
        std::string s1 = "Hello, world!"; // s1 是左值
        std::string s2 = std::move(s1); // std::move(s1) 是右值

    // print 函数，接受一个字符串并打印它
    void print(const std::string& s) {std::cout << "print(const std::string&): " << s << std::endl;}
    void print(std::string&& s)      {std::cout << "print(std::string&&): " << s << std::endl;} // 这里 && 表示的是右值引用

    // call_print 函数，使用完美转发来调用 print 函数
    template <typename T>
    void call_print(T&& s) { print(std::forward<T>(s));}

    int main() {
        std::string s = "Hello, world!";
        call_print(s); // s 是左值
        call_print("Hello, world!"); // "Hello, world!" 是右值
        return 0;
    }
  */
  return {span_begin, std::find_if_not(span_begin, span_end, std::forward<F>(func))};
}

template <typename It, typename F>
std::pair<It, It> get_span_p(It begin, It end, F&& func)
{
  return get_span_p(begin, end, std::numeric_limits<typename std::iterator_traits<It>::difference_type>::max(), std::forward<F>(func));
}
} // namespace champsim

#endif
