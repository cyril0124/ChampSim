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

#ifndef OPERABLE_H
#define OPERABLE_H

namespace champsim
{

class operable
{
public:
  const double CLOCK_SCALE;

  double leap_operation = 0;
  uint64_t current_cycle = 0;
  bool warmup = true;

  explicit operable(double scale) : CLOCK_SCALE(scale - 1) {}

  long _operate()
  {
    /*
      在 champsim::operable 类中，leap_operation 用于控制类的 _operate() 方法的执行频率。
      具体来说，每次 _operate() 调用时，如果 leap_operation 的值大于等于1，那么 _operate() 方法将跳过执行并返回0，
      同时 leap_operation 的值将减1。如果 leap_operation 的值小于1，那么 _operate() 方法将调用 operate() 方法
      （这是一个纯虚函数，具体实现由派生类提供）。然后，leap_operation 的值增加 CLOCK_SCALE 的值，current_cycle 值加1。

      CLOCK_SCALE 变量是在类的构造函数中初始化的，它的值等于参数 scale 减1。
      这个值决定了 leap_operation 每次增加的大小，进而影响 _operate() 方法的执行频率。

      综上，leap_operation 在这个类中的主要作用是通过跳过 _operate() 方法的一部分调用来模拟操作的执行速度或者频率。
      这样，可以通过调整 CLOCK_SCALE 的值来控制 _operate() 方法的执行频率，实现对模拟器的精细控制
    */

    // skip periodically
    if (leap_operation >= 1) {
      leap_operation -= 1;
      return 0;
    }

    auto result = operate();

    leap_operation += CLOCK_SCALE;
    ++current_cycle;

    return result;
  }

  virtual void initialize() {} // LCOV_EXCL_LINE
  virtual long operate() = 0;
  virtual void begin_phase() {}       // LCOV_EXCL_LINE
  virtual void end_phase(unsigned) {} // LCOV_EXCL_LINE
  virtual void print_deadlock() {}    // LCOV_EXCL_LINE
};

} // namespace champsim

#endif
