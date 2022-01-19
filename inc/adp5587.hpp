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
// exti_isr
// #include <exti_interrupt_handler.hpp>


namespace adp5587
{

    //              1       2       3       4       5       6       7       8       9       10      11      12      13      14      15      16
    //  UpperRow    131/3   141/13  151/23  161/33  171/43  181/53  191/63  201/73  132/4   142/14  152/24  162/34  172/44  182/54  192/64  202/74
    //  LowerRow    129/1   139/11  149/21  159/31  169/41  179/51  189/61  199/71  130/2   140/12  150/22  160/32  170/42  180/52  190/62  200/72


// forward declaration
class exti_isr;

class Driver
{
public:

    // @brief Construct a new Driver object
    Driver(I2C_TypeDef *i2c_handle);
    
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

    // Keypad press encodings. see datasheet page 9 (https://www.analog.com/media/en/technical-documentation/data-sheets/adp5587.pdf)
    enum class KeyPadMappings
    {
        INIT=0,

        A7_OFF=71,	A6_OFF=61,	A5_OFF=51,	A4_OFF=41,	A3_OFF=31,	A2_OFF=21,	A1_OFF=11,	A0_OFF=1,
        B7_OFF=72,	B6_OFF=62,	B5_OFF=52,	B4_OFF=42,	B3_OFF=32,	B2_OFF=22,	B1_OFF=12,	B0_OFF=2,
        C7_OFF=73,	C6_OFF=63,	C5_OFF=53,	C4_OFF=43,	C3_OFF=33,	C2_OFF=23,	C1_OFF=13,	C0_OFF=3,
        D7_OFF=74,	D6_OFF=64,	D5_OFF=54,	D4_OFF=44,	D3_OFF=34,	D2_OFF=24,	D1_OFF=14,	D0_OFF=4,
        E7_OFF=75,	E6_OFF=65,	E5_OFF=55,	E4_OFF=45,	E3_OFF=35,	E2_OFF=25,	E1_OFF=15,	E0_OFF=5,
        F7_OFF=76,	F6_OFF=66,	F5_OFF=56,	F4_OFF=46,	F3_OFF=36,	F2_OFF=26,	F1_OFF=16,	F0_OFF=6,
        G7_OFF=77,	G6_OFF=67,	G5_OFF=57,	G4_OFF=47,	G3_OFF=37,	G2_OFF=27,	G1_OFF=17,	G0_OFF=7,
        H7_OFF=78,	H6_OFF=68,	H5_OFF=58,	H4_OFF=48,	H3_OFF=38,	H2_OFF=28,	H1_OFF=18,	H0_OFF=8,
        I7_OFF=79,	I6_OFF=69,	I5_OFF=59,	I4_OFF=49,	I3_OFF=39,	I2_OFF=29,	I1_OFF=19,	I0_OFF=9,
        J7_OFF=80,	J6_OFF=70,	J5_OFF=60,	J4_OFF=50,	J3_OFF=40,	J2_OFF=30,	J1_OFF=20,	J0_OFF=10, 

        A7_ON=199,	A6_ON=189,	A5_ON=179,	A4_ON=169,	A3_ON=159,	A2_ON=149,	A1_ON=139,	A0_ON=129,
        B7_ON=200,	B6_ON=190,	B5_ON=180,	B4_ON=170,	B3_ON=160,	B2_ON=150,	B1_ON=140,	B0_ON=130,
        C7_ON=201,	C6_ON=191,	C5_ON=181,	C4_ON=171,	C3_ON=161,	C2_ON=151,	C1_ON=141,	C0_ON=131,
        D7_ON=202,	D6_ON=192,	D5_ON=182,	D4_ON=172,	D3_ON=162,	D2_ON=152,	D1_ON=142,	D0_ON=132,
        E7_ON=203,	E6_ON=193,	E5_ON=183,	E4_ON=173,	E3_ON=163,	E2_ON=153,	E1_ON=143,	E0_ON=133,
        F7_ON=204,	F6_ON=194,	F5_ON=184,	F4_ON=174,	F3_ON=164,	F2_ON=154,	F1_ON=144,	F0_ON=134,
        G7_ON=205,	G6_ON=195,	G5_ON=185,	G4_ON=175,	G3_ON=165,	G2_ON=155,	G1_ON=145,	G0_ON=135,
        H7_ON=206,	H6_ON=196,	H5_ON=186,	H4_ON=176,	H3_ON=166,	H2_ON=156,	H1_ON=146,	H0_ON=136,
        I7_ON=207,	I6_ON=197,	I5_ON=187,	I4_ON=177,	I3_ON=167,	I2_ON=157,	I1_ON=147,	I0_ON=137,
        J7_ON=208,	J6_ON=198,	J5_ON=188,	J4_ON=178,	J3_ON=168,	J2_ON=158,	J1_ON=148,	J0_ON=138,
    };




    
    // @brief Updates the stored key events FIFO data and resets the HW ISR
    void update_key_events();

    // @brief Get the list of key events (last 10)
    // @param key_events_list 
    void get_key_events(std::array<KeyPadMappings, 10> &key_events_list);

    // @brief Notify this driver that the stored key events data has been read and can be cleared.
    void clear_key_events();

    void  exti_isr();

private:

    // @brief The i2c slave address for ADP5587ACPZ-1-R7
    const uint8_t m_i2c_addr {0x60};

    // @brief The CMSIS mem-mapped I2C periph. Set in the c'tor
    std::unique_ptr<I2C_TypeDef> m_i2c_handle;

    // exti_isr* m_interrupt_ptr; 
    // std::unique_ptr<exti_isr> m_interrupt_ptr;   

    // @brief local store for ADP5587 key event FIFO
    std::array<KeyPadMappings, 10> m_key_event_fifo {KeyPadMappings::INIT};
    
    // @brief Confirm ADP5587 replies to write_addr and read_addr with ACK 
    // @return true if both are successful, false if either fail.
    bool probe_i2c();

    // @brief Read the FIFO bytes into "m_key_event_fifo" member byte array
    void read_fifo_bytes_from_hw();

    void write_config_bits(uint8_t config_bits);
    void clear_config_bits(uint8_t config_bits);

    void enable_keypad_isr();
    void disable_keypad_isr();

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


};


} // namespace adp5587


#endif // __ADP5587_HPP__