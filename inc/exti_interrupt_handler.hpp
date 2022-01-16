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

#ifndef __EXTI_INTERRUPT_HANDLER_HPP__
#define __EXTI_INTERRUPT_HANDLER_HPP__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
    #include "main.h"
#pragma GCC diagnostic pop

#include <isr_manager_base_stm32g0.hpp>
#include <ll_tim_utils.hpp>
#include <memory>


namespace adp5587
{

// forward declaration
class Driver;

// @brief class for handling EXTI4_15 interrupts
class EXTI_InterruptHandler : public isr::stm32g0::ISRManagerBaseSTM32G0
{
public:
    // @brief Construct a new EXTI_InterruptHandler object
    // @param driver_instance The driver to call when the interrupt is triggered.
    EXTI_InterruptHandler(std::unique_ptr<adp5587::Driver> &driver_instance);

    // @brief called by isr::stm32g0::ISRManagerBaseSTM32G0 when registered interrupt is triggered by MCU
    virtual void ISR(void);

private:
    // @brief driver instance for ISR callback
    std::unique_ptr<adp5587::Driver> _driver_instance;

    uint32_t m_last_debounce_reading_usecs {0};
    static constexpr uint32_t m_debounce_threshold_usecs {50000};
};


} // namespace adp5587

#endif // __EXTI5_INTERRUPT_HANDLER_HPP__