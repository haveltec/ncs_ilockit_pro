#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ilockit_pro)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)

# I LOCK IT Bibliotheken
target_include_directories(app PRIVATE src)

target_sources(app PRIVATE ../../libs/bluetooth/services/ili/ili.c)
target_include_directories(app PRIVATE ../../libs/bluetooth/services/ili)

target_include_directories(app PRIVATE ../../libs/parser)
target_sources(app PRIVATE ../../libs/parser/ili_parser.c)

target_include_directories(app PRIVATE ../../libs/aes)
target_sources(app PRIVATE ../../libs/aes/aes.c)

target_include_directories(app PRIVATE ../../libs/button)
target_sources(app PRIVATE ../../libs/button/ili_button.c)

target_include_directories(app PRIVATE ../../libs/motorcontroller)
target_sources(app PRIVATE ../../libs/motorcontroller/ili_motorcontroller.c)

target_include_directories(app PRIVATE ../../libs/accelerometer)
target_sources(app PRIVATE ../../libs/accelerometer/ili_accelerometer.c)

target_include_directories(app PRIVATE ../../libs/battery)
target_sources(app PRIVATE ../../libs/battery/ili_battery.c)

target_include_directories(app PRIVATE ../../libs/uicr)
target_sources(app PRIVATE ../../libs/uicr/ili_uicr.c)

target_include_directories(app PRIVATE ../../libs/ble)
target_sources(app PRIVATE ../../libs/ble/ili_client.c)

target_include_directories(app PRIVATE ../../libs/piezo)
target_sources(app PRIVATE ../../libs/piezo/ili_piezo.c)

target_include_directories(app PRIVATE ../../libs/led)
target_sources(app PRIVATE ../../libs/led/ili_led.c)

target_include_directories(app PRIVATE ../../libs/can)
target_sources(app PRIVATE ../../libs/can/ili_can.c)


# NORDIC SDK APP END
