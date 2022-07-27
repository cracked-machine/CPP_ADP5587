// MIT License

// Copyright (c) 2022 Chris Sutton

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

#include <adp5587_common.hpp>
#if defined(USE_RTT)
  #include <SEGGER_RTT.h>
#endif

namespace adp5587
{

template <typename DEVICE_ISR_ENUM>
class Driver : public RestrictedBase, public adp5587::CommonData
{
public:
  // @brief Construct a new Driver object
  explicit Driver(I2C_TypeDef *i2c_handle)
  {

    // set the I2C_TypeDef pointer here
    // m_i2c_handle = std::unique_ptr<I2C_TypeDef>(i2c_handle);
    m_i2c_handle = i2c_handle;

    // register the interrupt with STM32G0InterruptManager
    m_ext_int_handler.register_driver(this);

    // check the slave device is talking
    probe_i2c();

    clear_fifo_and_isr();

    // check they have been reset. keep for debugging.
    // uint8_t int_stat_byte {0};
    // uint8_t key_lck_ec_stat_byte {0};
    // read_register(Registers::INT_STAT, int_stat_byte);
    // read_register(Registers::KEY_LCK_EC_STAT, key_lck_ec_stat_byte);
  }

  // @brief Confirm ADP5587 replies to write_addr and read_addr with ACK
  // @return true if both are successful, false if either fail.
  bool probe_i2c()
  {
    bool success{true};

    // check ADP5587 is listening on 0x60 (write). Left-shift of address is *not* required.
    if (stm32::i2c::initialise_slave_device(m_i2c_handle, m_i2c_addr, stm32::i2c::StartType::PROBE) == stm32::i2c::Status::NACK)
    {
      success = false;
    }
    return success;
  }

  // @brief Read some bytes from the ADP5587 register
  // @param reg The register to read
  void read_register(const uint8_t reg [[maybe_unused]], uint8_t &rx_byte [[maybe_unused]])
  {
    // read this number of bytes
    stm32::i2c::set_numbytes(m_i2c_handle, 1);

    // send AD5587 write address and the register we want to read
    stm32::i2c::initialise_slave_device(m_i2c_handle, m_i2c_addr, stm32::i2c::StartType::WRITE);
    stm32::i2c::send_byte(m_i2c_handle, reg);

    // send AD5587 read address and get received data
    stm32::i2c::initialise_slave_device(m_i2c_handle, m_i2c_addr, stm32::i2c::StartType::READ);
    stm32::i2c::receive_byte(m_i2c_handle, rx_byte);

    stm32::i2c::generate_stop_condition(m_i2c_handle);

#if defined(USE_RTT)
    switch (reg)
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

#endif // USE_RTT
  }
  // @brief Read the FIFO bytes into "m_key_event_fifo" member byte array
  void read_fifo_bytes_from_hw()
  {
    // read the FIFO bytes into class member byte array
    /// @note don't use std::array.at(), this will force exception handling to bloat the linked .elf
    uint8_t read_value{0};
    read_register(Registers::KEY_EVENTA, read_value);
    m_key_event_fifo[0] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTB, read_value);
    m_key_event_fifo[1] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTC, read_value);
    m_key_event_fifo[2] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTD, read_value);
    m_key_event_fifo[3] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTE, read_value);
    m_key_event_fifo[4] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTF, read_value);
    m_key_event_fifo[5] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTG, read_value);
    m_key_event_fifo[6] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTH, read_value);
    m_key_event_fifo[7] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTI, read_value);
    m_key_event_fifo[8] = static_cast<KeyPadMappings>(read_value);

    read_register(Registers::KEY_EVENTJ, read_value);
    m_key_event_fifo[9] = static_cast<KeyPadMappings>(read_value);
  }

  // @brief Get the list of key events (last 10)
  // @param key_events_list
  void get_key_events(std::array<adp5587::Driver<DEVICE_ISR_ENUM>::KeyPadMappings, 10> &key_events_list) { key_events_list = m_key_event_fifo; }

  void write_register(uint8_t reg [[maybe_unused]], uint8_t tx_byte [[maybe_unused]])
  {
    // write this number of bytes: The data byte(s) AND the address byte
    stm32::i2c::set_numbytes(m_i2c_handle, 2);

    // send AD5587 write address and the register we want to write
    stm32::i2c::initialise_slave_device(m_i2c_handle, m_i2c_addr, stm32::i2c::StartType::WRITE);
    stm32::i2c::send_byte(m_i2c_handle, reg);

    // send AD5587 read address and get received data
    stm32::i2c::send_byte(m_i2c_handle, tx_byte);

    stm32::i2c::generate_stop_condition(m_i2c_handle);
  }

  void exti_isr()
  {
    // tell the driver to read keypad FIFO data and clear adp5587 HW interrupt registers
    update_key_events();
    // clear the falling flag for EXTI Line 5
    EXTI->FPR1 = EXTI->FPR1 | EXTI_IMR1_IM5;
  }

  void enable_keypad_isr() { write_config_bits((ConfigReg::KE_IEN)); }

  void disable_keypad_isr() { clear_config_bits(ConfigReg::KE_IEN); }

  void enable_gpio_isr() { write_config_bits(ConfigReg::GPI_IEN); }

  void disable_gpio_isr() { clear_config_bits(ConfigReg::GPI_IEN); }

  void gpio_fifo_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::GPI_EM_REG1, row_mask);
    write_register(Registers::GPI_EM_REG2, col_mask0_7);
    write_register(Registers::GPI_EM_REG3, col_mask8_9);
  }

  void keypad_gpio_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::KP_GPIO1, row_mask);
    write_register(Registers::KP_GPIO2, col_mask0_7);
    write_register(Registers::KP_GPIO3, col_mask8_9);
  }

  void gpio_interrupt_select(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::GPIO_INT_EN1, row_mask);
    write_register(Registers::GPIO_INT_EN2, col_mask0_7);
    write_register(Registers::GPIO_INT_EN3, col_mask8_9);
  }

  void set_gpo_out(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::GPIO_DIR1, row_mask);
    write_register(Registers::GPIO_DIR2, col_mask0_7);
    write_register(Registers::GPIO_DIR3, col_mask8_9);
  }

  void set_gpi_active_high(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::GPIO_INT_LVL1, row_mask);
    write_register(Registers::GPIO_INT_LVL2, col_mask0_7);
    write_register(Registers::GPIO_INT_LVL3, col_mask8_9);
  }

  void disable_debounce(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::DEBOUNCE_DIS1, row_mask);
    write_register(Registers::DEBOUNCE_DIS2, col_mask0_7);
    write_register(Registers::DEBOUNCE_DIS3, col_mask8_9);
  }

  void disable_gpio_pullup(uint8_t row_mask, uint8_t col_mask0_7, uint8_t col_mask8_9)
  {
    write_register(Registers::GPIO_PULL1, row_mask);
    write_register(Registers::GPIO_PULL2, col_mask0_7);
    write_register(Registers::GPIO_PULL3, col_mask8_9);
  }

  void write_config_bits(uint8_t config_bits)
  {
    uint8_t existing_byte{0};
    read_register(Registers::CFG, existing_byte);
    write_register(Registers::CFG, existing_byte | config_bits);
    // maybe should read back and return bool based on comparison?
    uint8_t new_byte{0};
    read_register(Registers::CFG, new_byte);
  }

  void clear_config_bits(uint8_t config_bits)
  {
    uint8_t existing_byte{0};
    read_register(Registers::CFG, existing_byte);
    write_register(Registers::CFG, (existing_byte &= ~(config_bits)));
  }

  // @brief Updates the stored key events FIFO data and resets the HW ISR
  void update_key_events()
  {
    // Steps for Key interrupt events
    // 1. Check the specific type of interrupt in the Interrupt Status regsister (INT_STAT)
    // 2. Check if there is event data in the FIFO by reading the event counter (KEY_LCK_EC_STAT:KEC[0:3])
    // 3. Read (and implicitly clear the data) in the FIFO
    // 4. Reset the interrupt statuses in Interrupt Status regsister (INT_STAT)

    // 1. check if the INT_STAT bits are set
    uint8_t int_stat_byte{0};
    read_register(Registers::INT_STAT, int_stat_byte);

    // if the ADP5587 interrupt register shows key or gpio event we need to process and reset the registers
    if ((int_stat_byte & IntStatusReg::KE_INT) == IntStatusReg::KE_INT)
    {
      // 2. non-zero event counter means there is FIFO data to read (which also clears the FIFO)
      uint8_t key_lck_ec_stat_byte{0};
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
      while ((gpio_int_stat1_value | gpio_int_stat2_value | gpio_int_stat3_value) > 0)
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

  // @brief clear the Key Event Registers (KEY_EVENTx) by reading them and
  // clear the Interrupt status register (INT_STAT) by writing 1 to each bit
  void clear_fifo_and_isr()
  {
    // clear the key event FIFO by reading each register
    uint8_t ke_byte{0};
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

private:
  // @brief The CMSIS mem-mapped I2C periph. Set in the c'tor
  // std::unique_ptr<I2C_TypeDef> m_i2c_handle;
  I2C_TypeDef *m_i2c_handle;

  struct ExtIntHandler : public stm32::isr::InterruptManagerStm32Base<DEVICE_ISR_ENUM>
  {
    // @brief the parent driver class
    Driver *m_parent_driver_ptr;
    // @brief initialise and register this handler instance with IsrManagerStm32g0
    // @param parent_driver_ptr the instance to register
    void register_driver(Driver *parent_driver_ptr)
    {
      m_parent_driver_ptr = parent_driver_ptr;
      // register pointer to this handler class in stm32::isr::IsrManagerStm32g0
      stm32::isr::InterruptManagerStm32Base<DEVICE_ISR_ENUM>::register_handler(DEVICE_ISR_ENUM::exti5, this);
    }
    // @brief The callback used by IsrManagerStm32g0
    virtual void ISR() { m_parent_driver_ptr->exti_isr(); }
  };
  // @brief handler object
  ExtIntHandler m_ext_int_handler;
};

} // namespace adp5587

#endif // __ADP5587_HPP__