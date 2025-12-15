#!/bin/bash
mkdir -p build
cd build
cmake -DBOARD=production -DCMAKE_TOOLCHAIN_FILE=../cmake/gnu-arm-none-eabi.cmake ..
make
cd ..
read -p "Enter to exit script."
