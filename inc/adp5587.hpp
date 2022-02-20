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

#ifndef __ADP5587_HPP__
#define __ADP5587_HPP__

#include <adp5587_common.hpp>

namespace adp5587
{

template<typename DEVICE_ISR_ENUM>
class Driver : public AllocationRestrictedBase, public CommonFunctions
{
public:

    // @brief Construct a new Driver object
    Driver(I2C_TypeDef *i2c_handle)
    {

        // set the I2C_TypeDef pointer here
        m_i2c_handle = std::unique_ptr<I2C_TypeDef>(i2c_handle);

        // register the interrupt with STM32G0InterruptManager
        m_ext_int_handler.register_driver(this);

        // check the slave device is talking
        probe_i2c();


        clear_fifo_and_isr();

        // check they have been reset. keep for debugging.
        // uint8_t int_stat_byte {0};
        // uint8_t key_lck_ec_stat_byte {0};
        // read_register(Registers::INT_STAT, int_stat_byte);
        // read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);    

    }    
    
    // @brief Get the list of key events (last 10)
    // @param key_events_list 
    void get_key_events(std::array<adp5587::Driver<DEVICE_ISR_ENUM>::KeyPadMappings, 10> &key_events_list)
    {
        key_events_list = m_key_event_fifo;
    }



private:
	struct ExtIntHandler : public stm32::isr::InterruptManagerStm32Base<DEVICE_ISR_ENUM>
	{
        // @brief the parent driver class
        Driver *m_parent_driver_ptr;
		// @brief initialise and register this handler instance with IsrManagerStm32g0
		// @param parent_driver_ptr the instance to register
		void register_driver(Driver *parent_driver_ptr)
		{
			m_parent_driver_ptr = parent_driver_ptr;
			// register pointer to this handler class in stm32::isr::IsrManagerStm32g0
			stm32::isr::InterruptManagerStm32Base<DEVICE_ISR_ENUM>::register_handler(DEVICE_ISR_ENUM::exti5 , this);
		}        
        // @brief The callback used by IsrManagerStm32g0
		virtual void ISR()
		{
            m_parent_driver_ptr->exti_isr();
		}        
	};
	// @brief handler object
    ExtIntHandler m_ext_int_handler;


};


} // namespace adp5587

#endif // __ADP5587_HPP__