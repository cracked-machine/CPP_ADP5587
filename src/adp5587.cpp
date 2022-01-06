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
    std::array<uint8_t, 1> read_byte;

    // read the config register
	read_register(static_cast<uint8_t>(Registers::CFG), read_byte);
    
    // write to the config register
    std::array<uint8_t, 1> new_byte { 0x00 };
    write_register(static_cast<uint8_t>(Registers::CFG), new_byte);
    
    // read new value from the config register
    read_register(static_cast<uint8_t>(Registers::CFG), read_byte);
	
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

}
