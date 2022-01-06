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
#include <ll_i2c_utils.hpp>

namespace adp5587
{

enum class Registers
{
    DEV_ID              = 0x00,
    CFG                 = 0x01,
    INT_STAT            = 0x02
};

class Driver
{
public:
    Driver();

    // @brief Confirm ADP5587 replies to write_addr and read_addr with ACK 
    // @return true if both are successful, false if either fail.
    bool probe_i2c();
    
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

private:
    // @brief The write address for ADP5587ACPZ-1-R7
    const uint8_t write_addr {0x60};
    // @brief The read address for ADP5587ACPZ-1-R7
    const uint8_t read_addr {0x61};

};


template<std::size_t REG_SIZE>
void Driver::read_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &rx_bytes)
{
	// read this number of bytes
	const uint8_t num_bytes {REG_SIZE};
	LL_I2C_SetTransferSize(I2C3, num_bytes);
	
	// send AD5587 write address and the register we want to read
	stm32::i2c::send_addr(I2C3, write_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_command(I2C3, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_addr(I2C3, read_addr, stm32::i2c::MsgType::READ);
	stm32::i2c::receive_data(I2C3, rx_bytes);

	LL_I2C_GenerateStopCondition(I2C3);

	#if defined(USE_RTT) 
		SEGGER_RTT_printf(0, "\nreg: %u = %u", +reg, +rx_bytes.at(0)); 
	#endif		    
}

template<std::size_t REG_SIZE>
void Driver::write_register(const uint8_t reg, std::array<uint8_t, REG_SIZE> &tx_bytes)
{
	// write this number of bytes: The data byte(s) AND the address byte
	const uint8_t num_bytes {REG_SIZE + 1};
	LL_I2C_SetTransferSize(I2C3, num_bytes);
	
	// send AD5587 write address and the register we want to write
	stm32::i2c::send_addr(I2C3, write_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_command(I2C3, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_data(I2C3, tx_bytes);

	LL_I2C_GenerateStopCondition(I2C3);
 
}



} // namespace adp5587


#endif // __ADP5587_HPP__