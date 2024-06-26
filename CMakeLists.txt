cmake_minimum_required(VERSION 3.20)

project(ssc-firmware)

set (PROJECT_NAME             "ssc-firmware")
set (LINKER_SCRIPT            "../STM32F042F6PX_FLASH.ld")
set (MCPU                     "-mcpu=cortex-m0")
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

file(
    GLOB_RECURSE
    SRC
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f0/Source/Templates/system_stm32f0xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Startup/startup_stm32f042f6px.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/src/*.c"
    #"${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f0/Source/Templates/gcc/startup_stm32f042x6.s"
)


enable_language(ASM)

add_executable(${PROJECT_NAME} ${SRC})

target_compile_definitions(
    ${CMAKE_PROJECT_NAME}
    PRIVATE
    STM32F042x6
)

target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f0/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_core/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_core/Core/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/hw"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyusb/src/portable/st/stm32_fsdev"
)

target_compile_definitions(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32F0
    CFG_TUD_ENABLE=1
    CFG_TUD_HID=1
    DEBUG=1
)


add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} "-Obinary" "${CMAKE_PROJECT_NAME}.elf" "${PROJECT_NAME}.bin"
    COMMAND ${CMAKE_OBJCOPY} "-Oihex" "${CMAKE_PROJECT_NAME}.elf" "${PROJECT_NAME}.hex"
    COMMAND hex2dfu -i "${PROJECT_NAME}.hex" -o "${CMAKE_PROJECT_NAME}.dfu"
    COMMAND ${CMAKE_SIZE} --format=berkeley "${PROJECT_NAME}.elf"
)
