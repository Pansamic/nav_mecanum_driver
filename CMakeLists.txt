#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1
#        For release build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "nav_mecanum_driver")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32F407ZGTX_FLASH.ld")
set (MCPU                     ${MCPU_CORTEX_M4})
set (MFPU                     ${MFPU_FPV5_D16})
set (MFLOAT_ABI               ${MFLOAT_ABI_HARDWARE})
set (RUNTIME_LIBRARY          ${RUNTIME_LIBRARY_REDUCED_C})
set (RUNTIME_LIBRARY_SYSCALLS ${RUNTIME_LIBRARY_SYSCALLS_MINIMAL})
set (CMAKE_BUILD_TYPE         Debug)

file (GLOB PROJECT_SOURCES
	BSP/Src/*.c
	Core/Src/*.c
	Core/cRingbuf/cRingbuf.c
	Core/Startup/startup_stm32f407zgtx.s
	Drivers/STM32F4xx_HAL_Driver/Src/*.c
	micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c
	micro_ros_stm32cubemx_utils/extra_sources/*.c
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c
	Middlewares/Third_Party/FreeRTOS/Source/*.c
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
	USB_DEVICE/App/*.c
	USB_DEVICE/Target/usbd_conf.c

	)

set (PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	STM32F407xx
	USE_HAL_DRIVER
	ARM_MATH_CM4
    )

set (PROJECT_INCLUDES
	# LIST INCLUDE DIRECTORIES HERE
	BSP/Inc
	Core/Inc
	Core/cRingbuf
	Drivers/CMSIS/Include
	Drivers/CMSIS/DSP/Include
	Drivers/CMSIS/Device/ST/STM32F4xx/Include
	Drivers/STM32F4xx_HAL_Driver/Inc
	micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
	Middlewares/Third_Party/FreeRTOS/Source/include
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
	USB_DEVICE/App
	USB_DEVICE/Target
    )

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################
set(CMAKE_TOOLCHAIN_FILE cubeide-gcc.cmake)
################## PROJECT SETUP ######################################
project(${PROJECT_NAME} C ASM)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${PROJECT_INCLUDES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a)
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a)

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
if(${CMAKE_BUILD_TYPE} STREQUAL Debug)
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -Og -g3 -ggdb")
elseif(${CMAKE_BUILD_TYPE} STREQUAL Release)
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -O2")
endif()
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
