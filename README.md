# PolarFOC
CMake library for Field Oriented Control on STM32 devices (update this later)

## Usage

### Adding to Project

This project requires [EmbeddedLib](https://github.com/awesomeyooner/EmbeddedLib), please `git clone`.

To include this library, please `git clone` or `git submodule add` this repo into the `\lib` folder of your project

Then add this to link the library

```cmake
add_library(shared_library INTERFACE)

target_include_directories(shared_library INTERFACE
    ${CMAKE_SOURCE_DIR}/cube/Core/Inc
    ${CMAKE_SOURCE_DIR}/cube/USB_DEVICE/App
    ${CMAKE_SOURCE_DIR}/cube/USB_DEVICE/Target
    ${CMAKE_SOURCE_DIR}/cube/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/cube/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/cube/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_SOURCE_DIR}/cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/cube/Drivers/CMSIS/Include
)

target_compile_definitions(shared_library INTERFACE
    STM32F446xx
)

add_subdirectory(lib/PolarFOC)
target_link_libraries(PolarFOC PRIVATE shared_library)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    shared_library

    PolarFOC
)
```