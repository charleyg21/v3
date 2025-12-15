cmake_minimum_required(VERSION 3.16)

# Append current directory to CMAKE_MODULE_PATH for making device specific cmake modules visible
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(TOOLCHAIN_TRIPLE arm-none-eabi)

set(CMAKE_C_COMPILER_ID "GNU")

set(CMAKE_C_COMPILER arm-none-eabi-gcc CACHE INTERNAL "C Compiler")
set(CMAKE_CXX_COMPILER arm-none-eabi-g++ CACHE INTERNAL "C++ Compiler")
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc CACHE INTERNAL "ASM Compiler")

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_TRIPLE})

set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_TRIPLE})

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER_TARGET ${TOOLCHAIN_TRIPLE})

# Without following flag, CMake is not able to pass test compilation check.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_SYSROOT /usr/arm-none-eabi)
set(CMAKE_FIND_ROOT_PATH /usr/arm-none-eabi)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
