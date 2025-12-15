# EcoCharger

This repository contains the STM microcontroller CubeMX project and source code for the EcoCharger project.
Within this README, we provide the following information:
* The requirements for building and loading the microcontroller code.
* The steps for compiling and loading the microcontroller code.
* An overview of this repository's directory structure and how the source code is organized.
* Licensing of the included software.

## Requirements for Compiling, Loading, and Debugging on Linux

To build and load the microcontroller code, the following software is required:
* [STM32CubeMX initialization code generator](https://www.st.com/en/development-tools/stm32cubemx.html)
* [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* [OpenOCD Open On-Chip Debugger](http://openocd.org/)
* [CMake Build Process System](https://cmake.org/)
* [GNU make build utility](https://www.gnu.org/software/make/)
* Optionally for debugging, [GNU Debugger for the ARM EABI target](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

In addition to the above packages,
access to a command-line shell such as [bash](https://www.gnu.org/software/bash/) or [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/about) is assumed.

Finally,
the loading and debugging requires the [STLINK-V3 modular in-circuit debugger and programmer](https://www.st.com/en/development-tools/stlink-v3set.html).
This can be purchased from [Mouser Electronics](https://www.mouser.com/ProductDetail/STMicroelectronics/STLINK-V3SET?qs=sGAEpiMZZMu3sxpa5v1qrhWKVY0UyKH45NIjGrvpIMc%3D).

On an Arch Linux system,
the above software packages (with the exceptions of a command-line shell and `STM32CubeMX`) can be installed through the command-line package manager as follows:
```bash
pacman -S cmake make arm-none-eabi-gcc arm-none-eabi-gdb openocd
```
running this as the root user if necessary.

Installing `STM32CubeMX` requires downloading and installing from the above link to the STM website.
This might require installing the Java runtime to install and use.
Please see the above link for more information on the Java runtime dependencies and installation instructions.

## Instructions for Compiling, Loading, and Debugging on Linux

First, the project must be compiled into a `.hex` file that can loaded on to the microcontroller.
This is done from the command line as follows:
1. Make a build folder with `mkdir -p build`.
2. Switch to this new directory with `cd build`.
3. Create the build scripts to build with `gnu-arm-none-eabi` with:
```bash
cmake -DBOARD=production -DCMAKE_TOOLCHAIN_FILE=../cmake/gnu-arm-none-eabi.cmake ..
```
4. Finally, build the project with `make` which will generate the file `ecocharger.hex`.

To load the the microcontroller code, do the following:
1. Attach the STLINK-V3 programmer and apply power to the board.
2. Then, run the following from the command-line:
```bash
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c "program ecocharger.hex verify exit"
```
3. Once this has run successfully, you may reset the board to run the new microcontroller code.

To perform breakpoint debugging on the microcontroller,
you can run `arm-none-eabi-gdb` as follows:
1. Attach the STLINK-V3 programmer and apply power to the board.
2. Within one command-line instance, run the following:
```bash
openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
```
allowing this process to continue to run as you move to the next step.

3. Within another command-line instance, run the following from the `build` folder:
```bash
arm-none-eabi-gdb -tui -x openocd-run.gdb
```
at this point, you are have connected GDB to the microcontroller through its JTAG interface and can run any GDB breakpoint or stepping command for debugging purposes.

## Requirements for Loading on Windows

To program the EcoCharger microcontroller, you'll need the [ST-Link flash utility](https://drive.google.com/file/d/1381fvMnnMxCfXKCC_Hjd0tlDM4IR-GWa/view?usp=sharing).

Download the zip file, extract to your desktop, and run the contained setup executable.
You'll have to click through various prompts to install, and an unknown publisher warning needs to be overridden by clicking "More Info" and then clicking "Install anyway".

To make running the flasher utility actually work, I also had to install both of the following Windows library packages:
* [vc\_redist.x64.exe](https://drive.google.com/file/d/1vjnXwnxfazKVI3q_0LTvCHD9jmK6IoYF/view?usp=sharing)
* [vc\_redist.x86.exe](https://drive.google.com/file/d/1svNTt0QRLUm5ylYwhrYxTBlVBuRv3t6X/view?usp=sharing)

## Instructions for Loading on Windows

To connect to the microcontroller,
I had to change the connections settings mode by going to the `Target->Settings` menu and changing Mode from `Normal` to `Connect Under Reset`.

At this point, you can flash the microcontroller by doing the following:

Select `Connect to target` which is the little plug button with the lightning bolt over it.
This should generate something along the lines of the following in the little bottom console:

```
14:17:54 : ST-LINK SN : 0669FF383337554E43203011
14:17:54 : V2J31M21
14:17:54 : Connected via SWD.
14:17:54 : SWD Frequency = 4,0 MHz.
14:17:54 : Connection mode : Connect Under Reset.
14:17:54 : Debug in Low Power mode enabled.
```

Select `Open file` which is the little file button farthest to the left.
You should see something along the lines of the following in the bottom text console:

```
14:26:44 : [testbed_firmware_90_100.hex] opened successfully.
              Address Ranges [0x08000000 0x080000B8] [0x080000C0 0x0801CE38]
14:26:44 : [testbed_firmware_90_100.hex] checksum : 0x009F1B28
```

Then finally, click `Program verify` which is the page-looking icon to the right of the eraser.
This will bring up a prompt that you can ignore and just hit `Start`.
You should then see something along the lines of the following in the bottom text console:

```
14:18:23 : Memory programmed in 4s and 172ms.
14:18:23 : Verification...OK
14:18:23 : Programmed memory Checksum: 0x009F1B28
14:26:44 : [testbed_firmware_90_100.hex] opened successfully.
                  Address Ranges [0x08000000 0x080000B8] [0x080000C0 0x0801CE38]
14:26:44 : [testbed_firmware_90_100.hex] checksum : 0x009F1B28
```

At this point, power cycling the system should run the new firmware.


## Directory Structure and Organization

The project consists of a large number of files and folders.
We enumerate the main ones here and include a high-level description for each one listed:

* `bsp/configuration.ioc`

This file is the project file for `STM32CubeMX`.
Within `STM32CubeMX`, the pinout, peripheral hardware configuration, and high-level FreeRTOS primitives are defined,
and from this, the initialization code can be generated.
This initialization code includes weakly-linked callbacks and declarations that are then overriden or used by the source code in the `inc` and `src` directories.

The initialization code is rarely edited directly;
instead, to make changes, the configuration is changed through CubeMX and the initialization code is regenerated.
This is done in order to make the porting process as easy as possible.

Please see the documentation on `STM32CubeMX` on the STM website above for more information.

* `bsp/CMakeLists.txt`

This is the configuration file telling `cmake` how to compile the code generated by `STM32CubeMX` tool.
Please see the documentation on the `cmake` website given above for more information.

* `cmake`

This folder contains `cmake` include scripts telling `cmake` which compiler and toolsets to use.
The files in here are first linked in by using the `cmake` `-D` directive given above.
Please see the documentation on the `cmake` website given above for more information.

* `CMakeLists.txt`

This is the configuration file telling `cmake` how to compile the code included in the `inc` and `src` folders.
Additionally,
it includes directives for including the source code in the `bsp` and `vendor` folders by using the `CMakeLists.txt` files in those folders.
Please see the documentation on the `cmake` website given above for more information.

* `docs`

This folder includes documents for the requirements, schematic, and IC datasheets.

* `inc`

This folder and its subfolders includes the `C++` `.hpp` header files used in ths project.
More information on each of its subfolders is given below.

* `inc/callbacks`

This folder contains files for declaring callbacks, where each file contains callbacks addressing a different purpose, such as different hardware systems.
For example, `inc/callbacks/gpio.hpp` includes callbacks for the GPIO that callback in the event a specific pin has a rising or falling edge.
Please see the each of the files in this directory for more details.

* `inc/console/commands.hpp`

This file contains declarations for each of the serial console commands.
Please see this file for more details.

* `inc/commands`

This folder contains files for declaring high-level "command" interfaces for different tasks.
For example, the `inc/commands/adc.hpp` defines an ADC command list that can be used to drive the ADC task in order to generate ADC values.
Please see the each of the files in this directory and the tasks defined in `src/tasks/controllers` for more details.

* `inc/defines`

This folder contains files for defining preprocessor macros for different parts of the system.
Some of these preprocessor macros are XMACROs which enumerate lists of things that are repeated often.
Please see the each of the files in this directory for more details.

* `inc/drivers`

This folder contains files for declaring driver classes or functions targeting different parts of the system.
For example, the file `inc/drivers/adc.hpp` declares driver function for interfacing with the ADC hardware.
Please see the each of the files in this directory for more details.

* `inc/helpers`

This folder contains helper classes and functions.
These are generally of a miscellaneous variety, but
the main one is `inc/helpers/cmsis_os.hpp` which includes class wrappers for each cmsis-os primitive used in this project.
Please see the each of the files in this directory for more details.

* `README.md`

This is the README file that you are currently reading.

* `scripts`

Contains any Python scripts used in this project.

* `src`

This folder and its subfolders includes the C++ `.cpp` source files used in ths project.
More information on each of its subfolders is given below.

* `src/callbacks`

This folder contains files for defining callbacks, where each file contains callbacks addressing a different part of the system, such as a different hardware component.
For example, `src/callbacks/buttons.cpp` includes callbacks for each of the GPIO buttons.
These callbacks are generally weakly-linked so that they may be overridden by other compilation units.
Please see the each of the files in this directory for more details.

* `src/console/commands`

This folder contains files that define the serial console commands.
Please see files contained in this directory for more details.

* `src/drivers`

This folder contains files for defining driver classes or functions targeting different parts of the system.
Please see the each of the files in this directory for more details.

* `src/tasks`

This folder contains the files where most of the work is done.
Each of these files declares FreeRTOS tasks that pertain to different parts of the system.
For example, the file `src/tasks/controllers/adc.cpp` defines the ADC task that is responsible for interfacing with the ADC hardware and digitizing analog values.
These tasks are declared in the code generated by `STM32CubeMX` and thus these tasks are configured through that tool.
Please see the each of the files in this directory for more details.

* `vendor`

This folder contains subfolders containing third-party software independent of that from `bsp`.

* `vendor/FreeRTOS-Plus-CLI`

This folder contains the files for the third-party [FreeRTOS CLI library](https://freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_Command_Line_Interface.html).

## Licensing

In general, the source code in this project is copyrighted per the following terms:

> Copyright (C) National Chemical Company, Inc. - All Rights Reserved.
> Unauthorized copying of this file, via any medium is strictly prohibited.
> Proprietary and confidential.

In some cases, however, the software was derived from open source software;
in these cases, the original license terms still apply and the original license headers have been preserved.
