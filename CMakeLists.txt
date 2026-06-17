cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Gimbal_V1.1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Force .c files that use C++ headers to compile as C++
# (Keil MDK allows C++ syntax in .c files, GCC requires proper language)
set_source_files_properties(
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
    RM2023_Lib_V1.2/communication.c
    PROPERTIES LANGUAGE CXX
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User application sources
    RM2023_Lib_V1.2/communication.c
    RM2023_Lib_V1.2/RM_Lib.cpp
    CH010/example_data.c
    CH010/hipnuc_dec.c
    CH010/nmea_dec.c
    Math_Lib/PID.cpp
    Math_Lib/SMC.cpp
    Motor_Lib/DM.cpp
    Motor_Lib/RM.cpp
    Motor_Lib/RMD.cpp
    Control/Pitch.cpp
    Control/Shoot.cpp
    Control/Yaw.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User defined include paths
    RM2023_Lib_V1.2
    Control
    CH010
    Motor_Lib
    Math_Lib
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# =============================================
# 烧录/调试 自定义命令
# =============================================

# find_program(OPENOCD openocd REQUIRED)

# Flash: 编译 + 烧录
add_custom_target(flash
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg
        -c "program ${CMAKE_PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "烧录 ${CMAKE_PROJECT_NAME}.elf 到 STM32F405"
)

# Erase: 擦除芯片
add_custom_target(erase
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg
        -c "init; reset halt; stm32f4x mass_erase 0; exit"
    COMMENT "擦除 STM32F405 全部 Flash"
)

# Reset: 复位芯片
add_custom_target(reset
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg
        -c "init; reset run; exit"
    COMMENT "复位 STM32F405"
)
