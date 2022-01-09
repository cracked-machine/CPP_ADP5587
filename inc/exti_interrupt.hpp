// MIT License

// Copyright (c) 2021 Chris Sutton

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __EXTI_INTERRUPT_HPP__
#define __EXTI_INTERRUPT_HPP__

#include <interrupt_base.hpp>
#include <adp5587.hpp>
#include <memory>

namespace adp5587
{

// forward declaration
class Driver;

class ExtiInterrupt : public stm32::isr::InterruptBase
{
public:
    ExtiInterrupt(std::unique_ptr<Driver> owner_ptr);
    virtual void ISR(void);

private:
    std::unique_ptr<Driver> interrupt_owner_ptr;
};


} // namespace adp5587

#endif // __EXTI_INTERRUPT_HPP__