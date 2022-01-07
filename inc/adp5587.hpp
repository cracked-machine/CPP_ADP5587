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

#if defined(USE_SSD1306_HAL_DRIVER) || defined(USE_SSD1306_LL_DRIVER)

	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wvolatile"
		#include "main.h"
		#include "i2c.h"	
	#pragma GCC diagnostic pop

#endif

#include <array>
#include <bitset>
#include <memory>

#include <ll_i2c_utils.hpp>

namespace adp5587
{

class Driver
{
public:
    // @brief Construct a new Driver object
    Driver();
    
    enum Registers
    {
        DEV_ID              = 0x00,
        CFG                 = 0x01,
        INT_STAT            = 0x02,
        KEY_LCK_EC_STAT     = 0x03,
        KP_GPIO1            = 0x1D,
        KP_GPIO2            = 0x1E,
        KP_GPIO3            = 0x1F
    }; 

    enum KeyEventRegisters
    {
        KEY_EVENTA          = 0x04,
        KEY_EVENTB          = 0x05,
        KEY_EVENTC          = 0x06,
        KEY_EVENTD          = 0x07
    };

    enum KeyEvents 
    {
        KEY0            = (1 << 0x00),
        KEY1            = (1 << 0x01),
        KEY2            = (1 << 0x02),
        KEY3            = (1 << 0x03),
        KEY4            = (1 << 0x04),
        KEY5            = (1 << 0x05),
        KEY6            = (1 << 0x06),
        KEY7            = (1 << 0x07)
    };

    // @brief Confirm ADP5587 replies to write_addr and read_addr with ACK 
    // @return true if both are successful, false if either fail.
    bool probe_i2c();

    void enable_key_interrupts();
    void clear_isr(uint8_t isr_mask);
    bool check_key_event(KeyEventRegisters ke_reg, uint8_t event_mask);
    bool is_key_isr_detected();
    void get_key_event_counter();

private:

    std::unique_ptr<I2C_TypeDef> _i2c_handle;


    enum ConfigRegister 
    {
        AUTO_INC            = (1 << 0x07),
        GPIEM_CFG           = (1 << 0x06),
        OVR_FLOW_M          = (1 << 0x05),
        INT_CFG             = (1 << 0x04),
        OVR_FLOW_IEN        = (1 << 0x03),
        K_LCK_IM            = (1 << 0x02),
        GPI_IEN             = (1 << 0x01),
        KE_IEN              = (1 << 0x00)
    };

    enum IsrRegister
    {
        OVR_FLOW_INT    = (1 << 0x03),
        K_LCK_INT       = (1 << 0x02),
        GPI_INT         = (1 << 0x01),
        KE_INT          = (1 << 0x00),
    };


    
    // @brief Read some bytes from the ADP5587 register
    // @tparam REG_SIZE 
    // @param reg The register to read
    template<std::size_t REG_SIZE>
    void read_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &rx_bytes);


    // @brief Write the byte array to the ADP5587 register
    // @tparam REG_SIZE 
    // @param reg The register to modify
    // @param tx_bytes The value to write
    template<std::size_t REG_SIZE>
    void write_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &res);    

    // @brief The write address for ADP5587ACPZ-1-R7
    const uint8_t write_addr {0x60};
    // @brief The read address for ADP5587ACPZ-1-R7
    const uint8_t read_addr {0x61};

    uint8_t device_id {0};

};


template<std::size_t REG_SIZE>
void Driver::read_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &rx_bytes)
{
	// read this number of bytes
	const uint8_t num_bytes {REG_SIZE};
	LL_I2C_SetTransferSize(_i2c_handle.get(), num_bytes);
	
	// send AD5587 write address and the register we want to read
	stm32::i2c::send_addr(_i2c_handle, write_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_command(_i2c_handle, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_addr(_i2c_handle, read_addr, stm32::i2c::MsgType::READ);
	stm32::i2c::receive_data(_i2c_handle, rx_bytes);

	LL_I2C_GenerateStopCondition(_i2c_handle.get());

	#if defined(USE_RTT) 
		SEGGER_RTT_printf(0, "\nreg: %u = %u", +reg, +rx_bytes.at(0)); 
	#endif		    
}

template<std::size_t REG_SIZE>
void Driver::write_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &tx_bytes)
{
	// write this number of bytes: The data byte(s) AND the address byte
	const uint8_t num_bytes {REG_SIZE + 1};
	LL_I2C_SetTransferSize(_i2c_handle.get(), num_bytes);
	
	// send AD5587 write address and the register we want to write
	stm32::i2c::send_addr(_i2c_handle, write_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_command(_i2c_handle, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_data(_i2c_handle, tx_bytes);

	LL_I2C_GenerateStopCondition(_i2c_handle.get());
 
}



} // namespace adp5587


#endif // __ADP5587_HPP__