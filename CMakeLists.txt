# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(duke-bme554-ecg-temp-ble-lab)

# make sure to add the source code for new libraries
target_sources(app PRIVATE src/main.c)
