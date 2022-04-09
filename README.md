[![CMake](https://github.com/cracked-machine/cpp_adp5587/actions/workflows/cmake.yml/badge.svg)](https://github.com/cracked-machine/cpp_adp5587/actions/workflows/cmake.yml)
[![Codecov](https://img.shields.io/codecov/c/github/cracked-machine/cpp_adp5587)](https://app.codecov.io/gh/cracked-machine/cpp_adp5587)


# CPP_ADP5587
Library for Analog Devices ADP5587 "I/O Expander and QWERTY Keypad Controller" for STM32

See the [wiki](https://github.com/cracked-machine/cpp_adp5587/wiki) for documentation / reference

See [readme](tests) for information on unit testing/mocking.


#### Adding this library to your STM32 Project

There are two ways to add this library to your project's CMakeLists.txt:

1. Required external dependencies can be added to your project by including the [external.cmake](cmake/external.cmake) to your top level project:

```
set(BUILD_NAME "MyProject")
add_executable(${BUILD_NAME} "")
include(cmake/external.cmake)
```

2. Alternatively, you can add the [embedded_utils](https://github.com/cracked-machine/embedded_utils.git) and [stm32_interrupt_managers](https://github.com/cracked-machine/stm32_interrupt_managers.git) to your project as submodules and add the subdirectories:

```
add_subdirectory(extern/embedded_utils)
add_subdirectory(extern/stm32_interrupt_managers)
```



