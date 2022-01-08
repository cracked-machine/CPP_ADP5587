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
    // init the I2C periph handle
    _i2c_handle = std::unique_ptr<I2C_TypeDef>(I2C3);

    // check the slave device is talking
    probe_i2c();

    // get some default ISR values
    get_isr_info();

    // 1) Enable keypad interrupts
    write_config_bits(ConfigRegister::KE_IEN);

    // 2) Enable the keypad rows and columns 
    uint8_t kpsel_byte {0xFF};
    write_register(Registers::KP_GPIO1, kpsel_byte);
    read_register(Registers::KP_GPIO1, kpsel_byte);
    write_register(Registers::KP_GPIO2, kpsel_byte);
    read_register(Registers::KP_GPIO2, kpsel_byte);
    write_register(Registers::KP_GPIO3, kpsel_byte);
    read_register(Registers::KP_GPIO3, kpsel_byte);   

}

void Driver::get_isr_info()
{
    uint8_t rx_byte {0};
    read_register(Registers::DEV_ID, rx_byte);
    read_register(Registers::CFG, rx_byte);
    read_register(Registers::INT_STAT, rx_byte);
    read_register(Registers::KEY_LCK_EC_STAT, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTA, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTB, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTC, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTD, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTE, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTF, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTG, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTH, rx_byte);
    read_register(KeyEventRegisters::KEY_EVENTI, rx_byte);

    //              1       2       3       4       5       6       7       8       9       10      11      12      13      14      15      16
    //  UpperRow    131/3   141/13  151/23  161/33  171/43  181/53  191/63  201/73  132/4   142/14  152/24  162/34  172/44  182/54  192/64  202/74
    //  LowerRow    129/1   139/11  149/21  159/31  169/41  179/51  189/61  199/71  130/2   140/12  150/22  160/32  170/42  180/52  190/62  200/72

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

bool Driver::is_key_isr_detected()
{
    uint8_t isr_byte {0};
    read_register(Registers::INT_STAT, isr_byte);        
    if ( (isr_byte & IsrRegister::KE_INT) == IsrRegister::KE_INT)
    {
        return true;
    }
    return false;
}

void Driver::get_key_event_counter()
{
    uint8_t kec_byte {0};
    read_register(Registers::KEY_LCK_EC_STAT, kec_byte);    

}

void Driver::clear_isr(uint8_t isr_mask)
{
    uint8_t isr_byte { isr_mask };
    write_register(Registers::INT_STAT, isr_byte);
    read_register(Registers::INT_STAT, isr_byte);    
}

bool Driver::check_key_event(KeyEventRegisters ke_reg, uint8_t ke_mask)
{

    uint8_t ke_mask_byte { ke_mask };
    read_register(ke_reg, ke_mask_byte);
    if ( (ke_mask_byte & ke_mask) == ke_mask ) { return true; }
    else { return false; }
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


void Driver::write_register(const uint8_t reg, uint8_t &tx_byte)
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