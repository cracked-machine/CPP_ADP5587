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
#include <cstdint>


namespace adp5587
{

Driver::Driver()
{
    probe_i2c();

    // store the device id
    std::array<uint8_t, 1> dev_id_byte;
    read_register(Registers::DEV_ID, dev_id_byte);
    device_id = dev_id_byte.at(0);

    std::array<uint8_t, 1> kpsel_byte {0xFF};
    write_register(Registers::KP_GPIO1, kpsel_byte);
    read_register(Registers::KP_GPIO1, kpsel_byte);
    write_register(Registers::KP_GPIO2, kpsel_byte);
    read_register(Registers::KP_GPIO2, kpsel_byte);
    write_register(Registers::KP_GPIO3, kpsel_byte);
    read_register(Registers::KP_GPIO3, kpsel_byte);   

    enable_key_interrupts();

    get_key_event_counter();



    // check_key_event(KeyEventRegisters::KEY_EVENTA, 
    //     (KeyEvents::KEY0 | KeyEvents::KEY1 | KeyEvents::KEY2 | KeyEvents::KEY3 | 
    //     KeyEvents::KEY4 | KeyEvents::KEY5 | KeyEvents::KEY6 | KeyEvents::KEY7) );

    // // read the config register
    // std::array<uint8_t, 1> config_byte;
	// read_register(static_cast<uint8_t>(Registers::CFG), config_byte);
    
    // // write to the config register
    // std::array<uint8_t, 1> new_byte { 0x00 };
    // write_register(static_cast<uint8_t>(Registers::CFG), new_byte);
    
    // // read new value from the config register
    // read_register(static_cast<uint8_t>(Registers::CFG), config_byte);
	
}

bool Driver::probe_i2c()
{
	bool success {true};

    // check ADP5587 is listening on 0x60 (write) and 0x61 (read)
	if (stm32::i2c::send_addr(I2C3, write_addr, stm32::i2c::MsgType::PROBE) == stm32::i2c::Status::NACK) 
    {
        success = false;
    }

	if (stm32::i2c::send_addr(I2C3, read_addr, stm32::i2c::MsgType::PROBE) == stm32::i2c::Status::NACK) 
	{
        success = false;
	}	
 
    return success;
}

void Driver::enable_key_interrupts()
{
    std::array<uint8_t, 1> config_byte { ConfigRegister::KE_IEN };
    write_register(Registers::CFG, config_byte);
    read_register(Registers::CFG, config_byte);

}

bool Driver::is_key_isr_detected()
{
    std::array<uint8_t, 1> isr_byte {0};
    read_register(Registers::INT_STAT, isr_byte);        
    if ( (isr_byte.at(0) & IsrRegister::KE_INT) == IsrRegister::KE_INT)
    {
        return true;
    }
    return false;
}

void Driver::get_key_event_counter()
{
    std::array<uint8_t, 1> kec_byte {0};
    read_register(Registers::KEY_LCK_EC_STAT, kec_byte);    

}

void Driver::clear_isr(uint8_t isr_mask)
{
    std::array<uint8_t, 1> isr_byte { isr_mask };
    write_register(Registers::INT_STAT, isr_byte);
    read_register(Registers::INT_STAT, isr_byte);    
}

bool Driver::check_key_event(KeyEventRegisters ke_reg, uint8_t ke_mask)
{

    std::array<uint8_t, 1> ke_mask_byte { ke_mask };
    read_register(ke_reg, ke_mask_byte);
    if ( (ke_mask_byte.at(0) & ke_mask) == ke_mask ) { return true; }
    else { return false; }
}

} // namespace adp5587