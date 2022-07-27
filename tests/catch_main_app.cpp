#include <adp5587.hpp>
#include <catch2/catch_all.hpp>

// mock ISR enum just for explicit instance below
enum class DummyBaseIsrEnum
{
  exti5,
  isr_type2,
  capacity
};

// enforce code coverage with explicit instances of func templates so that linker does not drop references
template adp5587::Driver<DummyBaseIsrEnum>::Driver(I2C_TypeDef *i2c_handle);
template void adp5587::Driver<DummyBaseIsrEnum>::get_key_events(std::array<adp5587::Driver<DummyBaseIsrEnum>::KeyEventIndex, 10> &key_events_list);

TEST_CASE("Test ADG2188", "[ADG2188]") { REQUIRE(true); }