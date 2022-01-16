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

#include <adp5587.hpp>

// EXTI_InterruptHandler
#include <exti_interrupt_handler.hpp>

#include <cstdint>
#include <cassert>



namespace adp5587
{

Driver::Driver(I2C_TypeDef *i2c_handle)
{
    // pass this Driver instance to the external interrupt manager as reference
    std::unique_ptr<Driver> this_driver = std::unique_ptr<Driver>(this);
    interrupt_ptr = std::make_unique<EXTI_InterruptHandler>(this_driver);

    // set the I2C_TypeDef pointer here
    _i2c_handle = std::unique_ptr<I2C_TypeDef>(i2c_handle);


    // check the slave device is talking
    probe_i2c();

    // check the POR state of the "Interrupt status" and "Keylock and event counter" register
    uint8_t int_stat_byte {0};
    uint8_t key_lck_ec_stat_byte {0};
    read_register(Registers::INT_STAT, int_stat_byte);
    read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);

    clear_fifo_and_isr();
    
    // check they have been reset
    read_register(Registers::INT_STAT, int_stat_byte);
    read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);    

    // 1) Enable keypad interrupts
    write_config_bits(ConfigReg::KE_IEN);

    // 2) Enable the keypad rows and columns 
    uint8_t kpsel_byte {0xFF};
    write_register(Registers::KP_GPIO1, kpsel_byte);
    read_register(Registers::KP_GPIO1, kpsel_byte);
    write_register(Registers::KP_GPIO2, kpsel_byte);
    read_register(Registers::KP_GPIO2, kpsel_byte);
    write_register(Registers::KP_GPIO3, kpsel_byte);
    read_register(Registers::KP_GPIO3, kpsel_byte);   



}

void Driver::clear_fifo_and_isr()
{
    // clear the key event FIFO by reading each register
    uint8_t ke_byte {0};
    read_register(KeyEventReg::KEY_EVENTA, ke_byte);
    read_register(KeyEventReg::KEY_EVENTB, ke_byte);
    read_register(KeyEventReg::KEY_EVENTC, ke_byte);
    read_register(KeyEventReg::KEY_EVENTD, ke_byte);
    read_register(KeyEventReg::KEY_EVENTE, ke_byte);
    read_register(KeyEventReg::KEY_EVENTF, ke_byte);
    read_register(KeyEventReg::KEY_EVENTG, ke_byte);
    read_register(KeyEventReg::KEY_EVENTH, ke_byte);
    read_register(KeyEventReg::KEY_EVENTI, ke_byte);    

    // clear all interrupts
    write_register(Registers::INT_STAT, (IntStatusReg::KE_INT | IntStatusReg::GPI_INT | IntStatusReg::K_LCK_INT | IntStatusReg::OVR_FLOW_INT));    
}

void Driver::read_fifo_bytes_from_hw()
{
    // read the FIFO bytes into class member byte array

    uint8_t read_value {0};
    read_register(KeyEventReg::KEY_EVENTA, read_value);
    key_event_fifo.at(0) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTB, read_value);
    key_event_fifo.at(1) = static_cast<KeyPadMappings>(read_value);
    
    read_register(KeyEventReg::KEY_EVENTC, read_value);
    key_event_fifo.at(2) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTD, read_value);
    key_event_fifo.at(3) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTE, read_value);
    key_event_fifo.at(4) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTF, read_value);
    key_event_fifo.at(5) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTG, read_value);
    key_event_fifo.at(6) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTH, read_value);
    key_event_fifo.at(7) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTI, read_value);
    key_event_fifo.at(8) = static_cast<KeyPadMappings>(read_value);

    read_register(KeyEventReg::KEY_EVENTJ, read_value);
    key_event_fifo.at(9) = static_cast<KeyPadMappings>(read_value);


}

void Driver::update_key_events()
{
    // check the "Key events interrupt" bit is set
    uint8_t int_stat_byte {0};
    read_register(Registers::INT_STAT, int_stat_byte);

    if ( (int_stat_byte & IntStatusReg::KE_INT) == IntStatusReg::KE_INT)
    {
        // now check if the "key event counter" is zero
        uint8_t key_lck_ec_stat_byte {0};
        read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);
        if ((key_lck_ec_stat_byte & (KeyLckEvCntReg::KEC1 | KeyLckEvCntReg::KEC2 | KeyLckEvCntReg::KEC3)) == 0)
        {
            // The "Key events interrupt" bit needs resetting. Write 1 to the KE_INT bit to reset it
            write_register(Registers::INT_STAT, IntStatusReg::KE_INT);
        }
        else
        {
            // read the FIFO data (which also clears the FIFO and event counter)
            read_fifo_bytes_from_hw();


            // The "Key events interrupt" bit needs resetting. Write 1 to the KE_INT bit to reset it
            write_register(Registers::INT_STAT, IntStatusReg::KE_INT);

        }
    }

}

void Driver::get_key_events(std::array<KeyPadMappings, 10> &key_events_list)
{
    key_events_list = key_event_fifo;
}

bool Driver::probe_i2c()
{
	bool success {true};

    // check ADP5587 is listening on 0x60 (write). Left-shift of address is *not* required.
	if (stm32::i2c::send_addr(_i2c_handle, m_i2c_addr, stm32::i2c::MsgType::PROBE) == stm32::i2c::Status::NACK) 
    {
        success = false;
    }
 
    return success;
}

void Driver::write_config_bits(uint8_t config_bits)
{ 
    write_register(Registers::CFG, config_bits);
    uint8_t new_byte {0};
    read_register(Registers::CFG, new_byte);

}

void Driver::read_register(const uint8_t reg, uint8_t &rx_byte)
{
	// read this number of bytes
	LL_I2C_SetTransferSize(_i2c_handle.get(), 1);
	
	// send AD5587 write address and the register we want to read
	stm32::i2c::send_addr(_i2c_handle, m_i2c_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_byte(_i2c_handle, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_addr(_i2c_handle, m_i2c_addr, stm32::i2c::MsgType::READ);
	stm32::i2c::receive_byte(_i2c_handle, rx_byte);

	LL_I2C_GenerateStopCondition(_i2c_handle.get());

	#if defined(USE_RTT) 
        switch(reg)
        {
            case 0x00:
                SEGGER_RTT_printf(0, "\n\nDeviceID (%u): %u", +reg, +rx_byte);
                break;
            case 0x01:
                SEGGER_RTT_printf(0, "\nConfiguration Register 1 (%u): %u", +reg, +rx_byte);
                break;
            case 0x02: 
                SEGGER_RTT_printf(0, "\nInterrupt status register (%u): %u", +reg, +rx_byte);
                break;
            case 0x03: 
                SEGGER_RTT_printf(0, "\nKeylock and event counter register (%u): %u", +reg, +rx_byte);
                break;
            case 0x04: 
                SEGGER_RTT_printf(0, "\nKey Event Register A (%u): %u", +reg, +rx_byte);
                break;
            case 0x05: 
                SEGGER_RTT_printf(0, "\nKey Event Register B (%u): %u", +reg, +rx_byte);
                break;
            case 0x06: 
                SEGGER_RTT_printf(0, "\nKey Event Register C (%u): %u", +reg, +rx_byte);
                break;
            case 0x07: 
                SEGGER_RTT_printf(0, "\nKey Event Register D (%u): %u", +reg, +rx_byte);
                break;
            case 0x08: 
                SEGGER_RTT_printf(0, "\nKey Event Register E (%u): %u", +reg, +rx_byte);
                break;
            case 0x09: 
                SEGGER_RTT_printf(0, "\nKey Event Register F (%u): %u", +reg, +rx_byte);
                break;                
            case 0x0A: 
                SEGGER_RTT_printf(0, "\nKey Event Register G (%u): %u", +reg, +rx_byte);
                break;
            case 0x0B: 
                SEGGER_RTT_printf(0, "\nKey Event Register H (%u): %u", +reg, +rx_byte);
                break;
            case 0x0C: 
                SEGGER_RTT_printf(0, "\nKey Event Register I (%u): %u", +reg, +rx_byte);
                break;
            case 0x0D: 
                SEGGER_RTT_printf(0, "\nKey Event Register J (%u): %u", +reg, +rx_byte);
                break;
            case 0x1D: 
                SEGGER_RTT_printf(0, "\nR0-R7 Keypad selection (%u): %u", +reg, +rx_byte);
                break;
            case 0x1E: 
                SEGGER_RTT_printf(0, "\nC0-C7 Keypad selection (%u): %u", +reg, +rx_byte);
                break;
            case 0x1F: 
                SEGGER_RTT_printf(0, "\nC8-C9 Keypad selection (%u): %u", +reg, +rx_byte);
                break;                                

        }
		
	#endif		    
}


void Driver::write_register(const uint8_t reg, uint8_t tx_byte)
{
	// write this number of bytes: The data byte(s) AND the address byte
	const uint8_t num_bytes {2};
	LL_I2C_SetTransferSize(_i2c_handle.get(), num_bytes);
	
	// send AD5587 write address and the register we want to write
	stm32::i2c::send_addr(_i2c_handle, m_i2c_addr, stm32::i2c::MsgType::WRITE);
	stm32::i2c::send_byte(_i2c_handle, reg);

	// send AD5587 read address and get received data
	stm32::i2c::send_byte(_i2c_handle, tx_byte);

	LL_I2C_GenerateStopCondition(_i2c_handle.get());
 
}


} // namespace adp5587