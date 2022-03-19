target_sources(${BUILD_NAME} PRIVATE
    # put source files here
    adp5587.cpp   
    adp5587_common.cpp 
)

target_include_directories(${BUILD_NAME} PRIVATE 
    ../include
)