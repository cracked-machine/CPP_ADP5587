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

    enum KeyEventReg
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



    // @brief Confirm ADP5587 replies to write_addr and read_addr with ACK 
    // @return true if both are successful, false if either fail.
    bool probe_i2c();

    void write_config_bits(uint8_t config_bits);
    
    
    // @brief If "Key events interrupt" is set and "event counter" is non-zerom, read the FIFO data. 
    // Otherwise if "event counter" is zero, reset the "Key events interrupt"
    void process_fifo();

private:

    // @brief The CMSIS mem-mapped I2C periph. Set in the c'tor
    std::unique_ptr<I2C_TypeDef> _i2c_handle;

    // @brief local store for ADP5587 key event registers
    std::array<uint8_t, 10> key_event_fifo {0};
    
    // @brief Read the FIFO bytes into "key_event_fifo" member byte array
    void get_fifo_bytes();

    enum ConfigReg
    {
        KE_IEN              = 0x01,
        GPI_IEN             = 0x02,
        K_LCK_IM            = 0x03,
        OVR_FLOW_IEN        = 0x04,
        INT_CFG             = 0x05,
        OVR_FLOW_M          = 0x06,
        GPIEM_CFG           = 0x07,
        AUTO_INC            = 0x08,
    };

    enum IntStatusReg
    {
        KE_INT              = 0x01,
        GPI_INT             = 0x02,
        K_LCK_INT           = 0x03,
        OVR_FLOW_INT        = 0x04,
    };

    enum KeyLckEvCntReg
    {
        KEC1                = 0x01,
        KEC2                = 0x02,
        KEC3                = 0x03,
        LCK1                = 0x04,
        LCK2                = 0x05,
        K_LCK_EN            = 0x06,
    };
    
    // @brief Read some bytes from the ADP5587 register
    // @tparam REG_SIZE 
    // @param reg The register to read
    void read_register(const uint8_t reg, uint8_t &rx_byte);


    // @brief Write the byte array to the ADP5587 register
    // @tparam REG_SIZE 
    // @param reg The register to modify
    // @param tx_bytes The value to write
    void write_register(const uint8_t reg, uint8_t tx_byte);    

    // @brief clear the Key Event Registers (KEY_EVENTx) by reading them and 
    // clear the Interrupt status register (INT_STAT) by writing 1 to each bit
    void clear_fifo_and_isr();

 

    // @brief The i2c slave address for ADP5587ACPZ-1-R7
    const uint8_t m_i2c_addr {0x60};


    uint8_t m_device_id {0};

};


} // namespace adp5587


#endif // __ADP5587_HPP__