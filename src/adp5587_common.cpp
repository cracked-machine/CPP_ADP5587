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

#include <adp5587_common.hpp>


namespace adp5587
{

void CommonFunctions::write_register(uint8_t reg [[maybe_unused]], uint8_t tx_byte [[maybe_unused]])
{
    #if not defined(X86_UNIT_TESTING_ONLY)
        // write this number of bytes: The data byte(s) AND the address byte
        stm32::i2c::set_numbytes(m_i2c_handle.get(), 2);
        
        // send AD5587 write address and the register we want to write
        stm32::i2c::send_addr(m_i2c_handle.get(), m_i2c_addr, stm32::i2c::MsgType::WRITE);
        stm32::i2c::send_byte(m_i2c_handle.get(), reg);

        // send AD5587 read address and get received data
        stm32::i2c::send_byte(m_i2c_handle.get(), tx_byte);

        stm32::i2c::generate_stop_condition(m_i2c_handle.get());        
    #endif
}

void CommonFunctions::exti_isr()
{
#if not defined(X86_UNIT_TESTING_ONLY)

            // tell the driver to read keypad FIFO data and clear adp5587 HW interrupt registers
            update_key_events();
            // clear the falling flag for EXTI Line 5
            EXTI->FPR1 = EXTI->FPR1 | EXTI_IMR1_IM5;
            

#endif
}

void CommonFunctions::enable_keypad_isr() { write_config_bits((ConfigReg::KE_IEN)); }


void CommonFunctions::disable_keypad_isr() { clear_config_bits(ConfigReg::KE_IEN); }


void CommonFunctions::enable_gpio_isr() { write_config_bits(ConfigReg::GPI_IEN); }


void CommonFunctions::disable_gpio_isr() { clear_config_bits(ConfigReg::GPI_IEN); }


void CommonFunctions::gpio_fifo_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::GPI_EM_REG1, row_mask);
    write_register(Registers::GPI_EM_REG2, col_mask0_7);
    write_register(Registers::GPI_EM_REG3, col_mask8_9);
}


void CommonFunctions::keypad_gpio_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::KP_GPIO1, row_mask);
    write_register(Registers::KP_GPIO2, col_mask0_7);
    write_register(Registers::KP_GPIO3, col_mask8_9);
}


void CommonFunctions::gpio_interrupt_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::GPIO_INT_EN1, row_mask);
    write_register(Registers::GPIO_INT_EN2, col_mask0_7);
    write_register(Registers::GPIO_INT_EN3, col_mask8_9);    
}


void CommonFunctions::set_gpo_out(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::GPIO_DIR1, row_mask);
    write_register(Registers::GPIO_DIR2, col_mask0_7);
    write_register(Registers::GPIO_DIR3, col_mask8_9);     
}


void CommonFunctions::set_gpi_active_high(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::GPIO_INT_LVL1, row_mask);
    write_register(Registers::GPIO_INT_LVL2, col_mask0_7);
    write_register(Registers::GPIO_INT_LVL3, col_mask8_9);     
}


void CommonFunctions::disable_debounce(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::DEBOUNCE_DIS1, row_mask);
    write_register(Registers::DEBOUNCE_DIS2, col_mask0_7);
    write_register(Registers::DEBOUNCE_DIS3, col_mask8_9);     
}


void CommonFunctions::disable_gpio_pullup(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
{
    write_register(Registers::GPIO_PULL1, row_mask);
    write_register(Registers::GPIO_PULL2, col_mask0_7);
    write_register(Registers::GPIO_PULL3, col_mask8_9);     
}


bool CommonFunctions::probe_i2c()
{
	bool success {true};

    // check ADP5587 is listening on 0x60 (write). Left-shift of address is *not* required.
	if (stm32::i2c::send_addr(m_i2c_handle.get(), m_i2c_addr, stm32::i2c::MsgType::PROBE) == stm32::i2c::Status::NACK) 
    {
        success = false;
    }
 
    return success;
}

void CommonFunctions::clear_fifo_and_isr()
{
    // clear the key event FIFO by reading each register
    uint8_t ke_byte {0};
    read_register(Registers::KEY_EVENTA, ke_byte);
    read_register(Registers::KEY_EVENTB, ke_byte);
    read_register(Registers::KEY_EVENTC, ke_byte);
    read_register(Registers::KEY_EVENTD, ke_byte);
    read_register(Registers::KEY_EVENTE, ke_byte);
    read_register(Registers::KEY_EVENTF, ke_byte);
    read_register(Registers::KEY_EVENTG, ke_byte);
    read_register(Registers::KEY_EVENTH, ke_byte);
    read_register(Registers::KEY_EVENTI, ke_byte);    

    // clear all interrupts
    write_register(Registers::INT_STAT, (IntStatusReg::KE_INT | IntStatusReg::GPI_INT | IntStatusReg::K_LCK_INT | IntStatusReg::OVR_FLOW_INT));    
}

void CommonFunctions::read_register(const uint8_t reg [[maybe_unused]], uint8_t &rx_byte [[maybe_unused]])
{
    #if not defined(X86_UNIT_TESTING_ONLY)
        // read this number of bytes
        stm32::i2c::set_numbytes(m_i2c_handle.get(), 1);
        
        // send AD5587 write address and the register we want to read
        stm32::i2c::send_addr(m_i2c_handle.get(), m_i2c_addr, stm32::i2c::MsgType::WRITE);
        stm32::i2c::send_byte(m_i2c_handle.get(), reg);

        // send AD5587 read address and get received data
        stm32::i2c::send_addr(m_i2c_handle.get(), m_i2c_addr, stm32::i2c::MsgType::READ);
        stm32::i2c::receive_byte(m_i2c_handle.get(), rx_byte);

        stm32::i2c::generate_stop_condition(m_i2c_handle.get());  

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
                case 0x11: 
                    SEGGER_RTT_printf(0, "\nGPIO Interrupt Status 1: (%u): %u", +reg, +rx_byte);
                    break;                                
                case 0x12: 
                    SEGGER_RTT_printf(0, "\nGPIO Interrupt Status 2: (%u): %u", +reg, +rx_byte);
                    break;                                
                case 0x13: 
                    SEGGER_RTT_printf(0, "\nGPIO Interrupt Status 3: (%u): %u", +reg, +rx_byte);
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
                case 0x20: 
                    SEGGER_RTT_printf(0, "\nGPI Key Mode 1 (%u): %u", +reg, +rx_byte);
                    break;  
                case 0x21: 
                    SEGGER_RTT_printf(0, "\nGPI Key Mode 2 (%u): %u", +reg, +rx_byte);
                    break;  
                case 0x22: 
                    SEGGER_RTT_printf(0, "\nGPI Key Mode 3 (%u): %u", +reg, +rx_byte);
                    break;                                              
            }
            
        #endif	// USE_RTT
    #endif	    
}


void CommonFunctions::update_key_events()
{
    // Steps for Key interrupt events
    // 1. Check the specific type of interrupt in the Interrupt Status regsister (INT_STAT)
    // 2. Check if there is event data in the FIFO by reading the event counter (KEY_LCK_EC_STAT:KEC[0:3])
    // 3. Read (and implicitly clear the data) in the FIFO
    // 4. Reset the interrupt statuses in Interrupt Status regsister (INT_STAT)

    // 1. check if the INT_STAT bits are set
    uint8_t int_stat_byte {0};
    read_register(Registers::INT_STAT, int_stat_byte);

    // if the ADP5587 interrupt register shows key or gpio event we need to process and reset the registers
    if ((int_stat_byte & IntStatusReg::KE_INT) == IntStatusReg::KE_INT)
    {
        // 2. non-zero event counter means there is FIFO data to read (which also clears the FIFO)
        uint8_t key_lck_ec_stat_byte {0};
        read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);
        if ((key_lck_ec_stat_byte & (KeyLckEvCntReg::KEC1 | KeyLckEvCntReg::KEC2 | KeyLckEvCntReg::KEC3 | KeyLckEvCntReg::KEC4)) > 0)
        {
            // 3. read the FIFO data (which also clears the FIFO and event counter)
            read_fifo_bytes_from_hw();

        }
        // 4. Make sure we clear the interrupt status (by writing 1). Interrupts are blocked until the register is cleared.
        write_register(Registers::INT_STAT, IntStatusReg::KE_INT);        
    }

    // Steps for GPI interrupt events
    // 1. Check the specific type of interrupt in the Interrupt Status regsister (INT_STAT)
    // 2. Check if GPIO events were configured to be sent to key event FIFO (GPI_EM_REG1, GPI_EM_REG2, GPI_EM_REG3)
    //    If so, read (and implicitly clear the data) in the FIFO
    // 3. Read (and implicitly clear the GPI interrupt data) in GPIO_INT_STAT1, GPIO_INT_STAT2, GPIO_INT_STAT3
    // 4. The Interrupt Status regsister (INT_STAT) can now be reset

    // 1. confirm GPIO interrupt status
    if ((int_stat_byte & IntStatusReg::GPI_INT) == IntStatusReg::GPI_INT) 
    {

        // 2. if we enabled GPI interrupts in the event FIFO then we must read the event FIFO to clear that data
        uint8_t read_gpi_em1_value{0};
        uint8_t read_gpi_em2_value{0};
        uint8_t read_gpi_em3_value{0};
        read_register(Registers::GPI_EM_REG1, read_gpi_em1_value);
        read_register(Registers::GPI_EM_REG2, read_gpi_em2_value);
        read_register(Registers::GPI_EM_REG3, read_gpi_em3_value);                

        if ((read_gpi_em1_value | read_gpi_em2_value | read_gpi_em3_value) > 0)
        {
            read_fifo_bytes_from_hw();
        }

        // 3. We need to clear the GPI Interrupt Status before we can continue,
        // Datasheet says read twice to clear but they can be stubborn so keep reading (usually 10x) until they clear.
        uint8_t gpio_int_stat1_value{0};
        uint8_t gpio_int_stat2_value{0};
        uint8_t gpio_int_stat3_value{0};
        read_register(Registers::GPIO_INT_STAT1, gpio_int_stat1_value);
        read_register(Registers::GPIO_INT_STAT2, gpio_int_stat2_value);
        read_register(Registers::GPIO_INT_STAT3, gpio_int_stat3_value);
        while((gpio_int_stat1_value | gpio_int_stat2_value | gpio_int_stat3_value) > 0)
        {
            read_register(Registers::GPIO_INT_STAT1, gpio_int_stat1_value);
            read_register(Registers::GPIO_INT_STAT2, gpio_int_stat2_value);        
            read_register(Registers::GPIO_INT_STAT3, gpio_int_stat3_value);
        }
        
        // 4. now we have cleared the cause of the interrupt, 
        // we can clear the GPI_INT bit in the shared Interrupt Status Register.
        uint8_t int_stat_value{0};
        write_register(Registers::INT_STAT, IntStatusReg::GPI_INT); 
        read_register(Registers::INT_STAT, int_stat_value);
    }
}


void CommonFunctions::clear_key_events()
{
    m_key_event_fifo.fill(KeyPadMappings::INIT);
}

void CommonFunctions::read_fifo_bytes_from_hw()
{
    // read the FIFO bytes into class member byte array

    uint8_t read_value {0};
    read_register(Registers::KEY_EVENTA, read_value);
    m_key_event_fifo.at(0) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTB, read_value);
    m_key_event_fifo.at(1) = static_cast<KeyPadMappings>(read_value);
    
    read_register(Registers::KEY_EVENTC, read_value);
    m_key_event_fifo.at(2) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTD, read_value);
    m_key_event_fifo.at(3) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTE, read_value);
    m_key_event_fifo.at(4) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTF, read_value);
    m_key_event_fifo.at(5) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTG, read_value);
    m_key_event_fifo.at(6) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTH, read_value);
    m_key_event_fifo.at(7) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTI, read_value);
    m_key_event_fifo.at(8) = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTJ, read_value);
    m_key_event_fifo.at(9) = static_cast<KeyPadMappings>(read_value);


}


void CommonFunctions::write_config_bits(uint8_t config_bits)
{ 
    uint8_t existing_byte {0};
    read_register(Registers::CFG, existing_byte);    
    write_register(Registers::CFG, existing_byte | config_bits);
    // maybe should read back and return bool based on comparison?
    uint8_t new_byte {0};
    read_register(Registers::CFG, new_byte);

}

void CommonFunctions::clear_config_bits(uint8_t config_bits)
{
    uint8_t existing_byte {0};
    read_register(Registers::CFG, existing_byte);
    write_register(Registers::CFG, (existing_byte &= ~(config_bits)));
}







} // namespace adp5587