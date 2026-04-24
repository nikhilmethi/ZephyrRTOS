# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

if(NOT DEFINED NCS_VERSION_MAJOR OR "${NCS_VERSION_MAJOR}.${NCS_VERSION_MINOR}" VERSION_LESS "3.2")
  message(FATAL_ERROR "nRF Connect SDK >= 3.2 is required (found: ${NCS_VERSION})")
endif()

project(duke-bme554-ecg-temp-ble-lab)

set(NO_BUILD_TYPE_WARNING TRUE)

# make sure to add the source code for new libraries
target_sources(app PRIVATE
    src/main.c
    src/calc_freq.c
    src/ble-lib.c
)
