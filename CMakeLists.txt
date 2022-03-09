add_library(adp5587 "")
target_sources(adp5587 PRIVATE
    # put source files here
    src/adp5587.cpp   
    src/adp5587_common.cpp 
)

target_include_directories(adp5587 PRIVATE 
    inc
    ../embedded_utils/inc
    ../stm32_interrupt_managers/rawptrs/inc
    ../stm32cube_workspace/G0B1KET6N/Drivers/CMSIS/Device/ST/STM32G0xx/Include
    ../stm32cube_workspace/G0B1KET6N/Drivers/CMSIS/Include    
)