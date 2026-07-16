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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME base)

# ThreadX configuration for Cortex-M7 / GCC
set(THREADX_ARCH cortex_m7)

# 加载配置（ROBOT/BOARD 在 apps/config.cmake 中定义）
include(${CMAKE_SOURCE_DIR}/../../apps/config.cmake)

# 生成头文件到 build 目录
set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
include(${CMAKE_SOURCE_DIR}/../../apps/generate_headers.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add ThreadX RTOS
add_subdirectory(../../threadx threadx)

# CherryUSB Device CDC ACM
set(CONFIG_CHERRYUSB_DEVICE ON CACHE BOOL "Enable CherryUSB device stack" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_CDC_ACM ON CACHE BOOL "Enable CDC ACM class" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_DWC2_ST ON CACHE BOOL "Use DWC2 OTG with STM32 glue" FORCE)
set(CONFIG_CHERRYUSB_OSAL "threadx" CACHE STRING "Use ThreadX OS abstraction layer" FORCE)

include(../../CherryUSB/cherryusb.cmake)
list(REMOVE_DUPLICATES cherryusb_srcs)
list(REMOVE_DUPLICATES cherryusb_incs)

add_library(cherryusb STATIC ${cherryusb_srcs})
target_include_directories(cherryusb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../bsp/USB
    ${cherryusb_incs}
)
target_compile_options(cherryusb PRIVATE -O3 -ffast-math -fno-math-errno)
target_link_libraries(cherryusb PUBLIC stm32cubemx azrtos::threadx utils)

# Add shared project libraries
add_subdirectory(../../utils ${CMAKE_CURRENT_BINARY_DIR}/utils)
add_subdirectory(../../board/bsp ${CMAKE_CURRENT_BINARY_DIR}/bsp)
add_subdirectory(../../robot ${CMAKE_CURRENT_BINARY_DIR}/robot)
add_subdirectory(../../modules ${CMAKE_CURRENT_BINARY_DIR}/modules)
add_subdirectory(../../apps ${CMAKE_CURRENT_BINARY_DIR}/apps)


# CMSIS-DSP  settings for Cortex-M7
set(LOOPUNROLL ON CACHE BOOL "Loop unrolling for max performance" FORCE)
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels (not needed for M7)" FORCE)

add_subdirectory(../../CMSIS-DSP ${CMAKE_CURRENT_BINARY_DIR}/cmsisdsp)

# Apply compiler flags to CMSIS-DSP
target_compile_options(CMSISDSP PRIVATE -O3 -ffast-math -fno-math-errno -flto)
# Provide CMSIS-Core headers (cmsis_compiler.h) to CMSIS-DSP
target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${_generated_dir}
    # Add user defined include paths
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
    azrtos::threadx
    utils
    bsp
    robot
    CMSISDSP
    cherryusb
    modules
    app
    # Add user defined libraries
)

# Enable LTO at link stage (required for -flto compiled objects)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -flto)
