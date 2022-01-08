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
        KEY_EVENTD          = 0x07,
        KEY_EVENTE          = 0x08,
        KEY_EVENTF          = 0x09,
        KEY_EVENTG          = 0x0A,
        KEY_EVENTH          = 0x0B,
        KEY_EVENTI          = 0x0C,
        KEY_EVENTJ          = 0x0D,
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

    void write_config_bits(uint8_t config_bits);
    void clear_isr(uint8_t isr_mask);
    bool check_key_event(KeyEventRegisters ke_reg, uint8_t event_mask);
    bool is_key_isr_detected();
    void get_key_event_counter();
    void get_isr_info();

private:

    std::unique_ptr<I2C_TypeDef> _i2c_handle;


    enum ConfigRegister 
    {
        AUTO_INC            = (0x08),
        GPIEM_CFG           = (0x07),
        OVR_FLOW_M          = (0x06),
        INT_CFG             = (0x05),
        OVR_FLOW_IEN        = (0x04),
        K_LCK_IM            = (0x03),
        GPI_IEN             = (0x02),
        KE_IEN              = (0x01)
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
    void read_register(const uint8_t reg, uint8_t &rx_byte);


    // @brief Write the byte array to the ADP5587 register
    // @tparam REG_SIZE 
    // @param reg The register to modify
    // @param tx_bytes The value to write
    void write_register(const uint8_t reg, uint8_t &tx_byte);    

    // @brief The i2c slave address for ADP5587ACPZ-1-R7
    const uint8_t m_i2c_addr {0x60};


    uint8_t m_device_id {0};

};


} // namespace adp5587


#endif // __ADP5587_HPP__