# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Accept command line parameters for adapter strings
if(DEFINED ADAPTER_USB_MANUFACTURER_OVERRIDE)
    add_compile_definitions("ADAPTER_USB_MANUFACTURER_OVERRIDE=\"${ADAPTER_USB_MANUFACTURER_OVERRIDE}\"")
endif()

if(DEFINED ADAPTER_USB_PRODUCT_OVERRIDE)
    add_compile_definitions("ADAPTER_USB_PRODUCT_OVERRIDE=\"${ADAPTER_USB_PRODUCT_OVERRIDE}\"")
endif()

if(DEFINED ADAPTER_USB_CDC_STRING_OVERRIDE)
    add_compile_definitions("ADAPTER_USB_CDC_STRING_OVERRIDE=\"${ADAPTER_USB_CDC_STRING_OVERRIDE}\"")
elseif(DEFINED ADAPTER_USB_MANUFACTURER_OVERRIDE AND DEFINED ADAPTER_USB_PRODUCT_OVERRIDE)
    # Otherwise, compute it by concatenating the manufacturer and product overrides
    add_compile_definitions("ADAPTER_USB_CDC_STRING_OVERRIDE=\"${ADAPTER_USB_MANUFACTURER_OVERRIDE} ${ADAPTER_USB_PRODUCT_OVERRIDE}\"")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(usb2uart)
