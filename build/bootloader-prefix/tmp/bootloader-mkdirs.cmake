# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/ffsk/esp/esp-idf/components/bootloader/subproject"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/tmp"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/src/bootloader-stamp"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/src"
  "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ffsk/esp/ESP32_S3_Projects/sm4rtedDUMBED/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
