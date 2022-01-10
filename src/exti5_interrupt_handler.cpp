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

#include <exti5_interrupt_handler.hpp>
#include <adp5587.hpp>

namespace adp5587
{

EXTI5_InterruptHandler::EXTI5_InterruptHandler(std::unique_ptr<adp5587::Driver> &driver_instance) 
: _driver_instance (std::move(driver_instance))
{
    // Pass the interrupt number/driver pointer up to the base class.
    std::unique_ptr<InterruptManagerBase> this_ext_interrupt = std::unique_ptr<InterruptManagerBase>(this);
    InterruptManagerBase::register_handler(
        isr::stm32g0::InterruptManagerBase::ISRVectorTableEnums::exti5_irqhandler, 
        this_ext_interrupt);
}

void EXTI5_InterruptHandler::ISR()
{
    // tell the driver to read keypad FIFO data and clear adp5587 HW interrupt registers
    _driver_instance->process_fifo();

    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_5) != RESET)
    {
        LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_5);
    }
}

} // namespace adp5587
