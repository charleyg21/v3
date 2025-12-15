#!/bin/bash

rm -r build
rm -r OUTPUTS

export C_INCLUDE_PATH=/usr/arm/gcc-arm-none-eabi-10.3-2021.10/arm-none-eabi/include
export CPLUS_INCLUDE_PATH=/usr/arm/gcc-arm-none-eabi-10.3-2021.10/arm-none-eabi/include/c++/10.3.1:$C_INCLUDE_PATH

export PATH=$PATH:/usr/arm/gcc-arm-none-eabi-10.3-2021.10/bin

arm-none-eabi-gcc --version

mkdir -p OUTPUTS

mkdir -p build
cd build

cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/clang-arm-none-eabi.cmake ..
make

cp *.hex ../OUTPUTS/

cd ..

read -p "Enter to exit script."
