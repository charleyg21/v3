cmake_minimum_required(VERSION 3.16)
enable_language(ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

# Allow switching of target platform
set(BOARD_LIST ecocharger)
set(BOARD ecocharger CACHE STRING "Default target platform is ecocharger")
set_property(CACHE BOARD PROPERTY STRINGS ${BOARD_LIST})

string(JOIN " " CPU_FLAGS
    "-mcpu=cortex-m0plus"
    "-mfloat-abi=soft"
    "-mthumb"
)

string(JOIN " " DEBUG_FLAGS
    "-Og"
    "-ggdb"
    "-g3"
)

list(APPEND WARNING_FLAGS
    "-Wall"
    "-Wextra"
    "-Wno-unused-parameter"
)

if (CMAKE_C_COMPILER_ID MATCHES "GNU")
    list(APPEND WARNING_FLAGS
        "-Wno-pointer-compare"
    )
elseif (CMAKE_C_COMPILER_ID MATCHES "Clang")
    list(APPEND WARNING_FLAGS
        "-Wno-parentheses-equality"
        "-Wno-deprecated-register"
        "-Wno-gnu-include-next"
        "-Wno-format-security"
    )
else()
    message(FATAL_ERROR "Toolchain '${CMAKE_C_COMPILER_ID}' not supported")
endif()

string(JOIN " " WARNING_FLAGS ${WARNING_FLAGS})

string(JOIN " " OPTION_FLAGS
    "-fno-builtin"
    "-ffunction-sections"
    "-fdata-sections"
    "-fomit-frame-pointer"
    "-fno-exceptions"
    "-fno-unroll-loops"
    "-ffast-math"
    "-ftree-vectorize"
    "-fshort-enums"
    "-fno-common"
)

string(JOIN " " CMAKE_C_FLAGS
    "-std=gnu11"
    ${CPU_FLAGS}
    ${DEBUG_FLAGS}
    ${WARNING_FLAGS}
    ${OPTION_FLAGS}
)

string(JOIN " " CMAKE_CXX_FLAGS
    "-std=c++17"
    ${CPU_FLAGS}
    ${DEBUG_FLAGS}
    ${WARNING_FLAGS}
    ${OPTION_FLAGS}
)


string(JOIN " " CMAKE_ASM_FLAGS
    "-x assembler-with-cpp"
    ${CPU_FLAGS}
    ${WARNING_FLAGS}
    ${OPTION_FLAGS}
)

set(BSP_DIR bsp CACHE PATH "BSP directory")
add_subdirectory(${BSP_DIR})

string(JOIN " " CMAKE_EXE_LINKER_FLAGS
    "-Wl,--gc-sections"
    "--specs=nosys.specs"
    ${CPU_FLAGS}
    "-T${LINKER_SCRIPT}"
)

add_subdirectory(vendor/FreeRTOS-Plus-CLI)

list(APPEND SOURCE_FILES
    src/callbacks/buttons.cpp
    src/callbacks/gpio.cpp
    src/console/commands/heater.cpp
    src/console/commands/rtc.cpp
    src/console/commands/syringe.cpp
    src/console/commands/task_list.cpp
    src/console/commands/thermocouple.cpp
    src/console/commands/valve.cpp
    src/drivers/gpio.cpp
    src/drivers/adc.cpp
    src/tasks/controllers/heater.cpp
    src/tasks/controllers/syringe.cpp
    src/tasks/controllers/thermocouple.cpp
    src/tasks/controllers/valve.cpp
    src/tasks/config.cpp
    src/tasks/console.cpp
    src/tasks/cycler.cpp
    src/tasks/ui.cpp
)

project(ecocharger)
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
    $<TARGET_OBJECTS:board>
    $<TARGET_OBJECTS:FreeRTOS_CLI>
)
add_dependencies(${PROJECT_NAME} board)

target_include_directories(${PROJECT_NAME} PUBLIC
    inc
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:FreeRTOS_CLI,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    $<TARGET_PROPERTY:board,INTERFACE_COMPILE_DEFINITIONS>
)

target_link_libraries(${PROJECT_NAME} PRIVATE c m nosys)

include(gdb-helper)
generate_run_gdb_openocd(${PROJECT_NAME})

include(binutils-arm-none-eabi)
create_bin_output(${PROJECT_NAME})
create_hex_output(${PROJECT_NAME})
print_section_sizes(${PROJECT_NAME})
