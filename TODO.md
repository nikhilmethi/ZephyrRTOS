# TODO

## Spring 2026

- [ ] Remove local `west` config to avoid large `external/` directory and be more flexible for different local OS SDK installs.

    - [ ] Update CI script to accomodate this change

    - [ ] Recommend the `v3.X` SDK for everything

In `CMakeLists.txt`:

```CMake
# Example: Require at least nRF Connect SDK 2.5.0
set(MINIMUM_SDK_VERSION "2.5.0")

# The SDK version is usually defined by Zephyr in zephyr_version.h or CMake
if(DEFINED ZEPHYR_BASE)
  include(${ZEPHYR_BASE}/cmake/zephyr_version.cmake)
endif()

if(NOT DEFINED ZEPHYR_VERSION_STRING)
  message(FATAL_ERROR "Could not determine Zephyr SDK version.")
endif()

# Compare versions
if(ZEPHYR_VERSION_STRING VERSION_LESS MINIMUM_SDK_VERSION)
  message(FATAL_ERROR
    "Your nRF Connect SDK / Zephyr version (${ZEPHYR_VERSION_STRING}) "
    "is too old. Please upgrade to at least ${MINIMUM_SDK_VERSION}.")
endif()
```

For nRF Connect SDK version:

```CMake
file(READ "${ZEPHYR_BASE}/../nrf/VERSION" NRF_VERSION_CONTENTS)
string(REGEX MATCH "VERSION_STRING *= *\"([0-9.]+)\"" _ ${NRF_VERSION_CONTENTS})
set(NRF_CONNECT_SDK_VERSION ${CMAKE_MATCH_1})

if(NRF_CONNECT_SDK_VERSION VERSION_LESS "2.5.0")
  message(FATAL_ERROR
    "nRF Connect SDK version ${NRF_CONNECT_SDK_VERSION} is too old. "
    "Please upgrade to at least 2.5.0.")
endif()
```

- [ ] Implement GPIO emulation for testing / grading.

- [ ] Migrate to GitHub for Gradescope integration?

- [ ] add `ble-lib.*` to this repo

- [ ] change `NOMINAL_BATTERY_VOLT_MV` from 3700 -> 3000
