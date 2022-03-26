[![CMake](https://github.com/cracked-machine/cpp_adp5587/actions/workflows/cmake.yml/badge.svg)](https://github.com/cracked-machine/cpp_adp5587/actions/workflows/cmake.yml)
[![Codecov](https://img.shields.io/codecov/c/github/cracked-machine/cpp_adp5587)](https://app.codecov.io/gh/cracked-machine/cpp_adp5587)


# CPP_ADP5587
Library for Analog Devices ADP5587 "I/O Expander and QWERTY Keypad Controller" for STM32


See the [wiki](https://github.com/cracked-machine/cpp_adp5587/wiki) for documentation / reference

See `.vscode/tasks.json` for details on the individual toolchain commands.
#### Running Units Tests on X86

When you run the default CMake build, the output is linked with the Catch2 library. To run the testsuite use the command:
`./build/test_suite`


#### Adding this library to your STM32 Project

Include this repo into your project as a submodule and add the following line to your top-level CMakeFiles.txt:

`add_subdirectory(cpp_adp5587)`

This assumes your top-level CMakeFiles.txt is already configured for STM32 platform.



