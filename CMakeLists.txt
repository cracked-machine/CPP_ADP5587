target_sources(${BUILD_NAME} PRIVATE
    # put source files here
    src/adp5587.cpp
    src/exti_interrupt.cpp
    
)

target_include_directories(${BUILD_NAME} PRIVATE inc)