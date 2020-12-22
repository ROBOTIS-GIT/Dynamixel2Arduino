# Dynamixel2Arduino [![Build Status](https://travis-ci.org/ROBOTIS-GIT/Dynamixel2Arduino.svg?branch=master)](https://travis-ci.org/ROBOTIS-GIT/Dynamixel2Arduino/branches)

## Serial and Direction Pin definitions by board
 - The examples defines GPIO pins based on the use with DYNAMIXEL Shields.
 - When running DYNAMIXEL without DYNAMIXEL Shields on OpenCM9.04, OpenCR or custom boards, you might need to change the Serial and DYNAMIXEL Direction Pin.
 - We provide the information below to make it easier to define Serial and Direction pins for specific hardware.

    |Board Name|Serial|Direction Pin|Note|
    |:-:|:-:|:-:|:-:|
    |OpenCM9.04|Serial1|28|because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.|
    |OpenCM485EXP|Serial3|22||
    |OpenCR|Serial3|84|For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it. ([Reference link](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78))|


## How to add new DYNAMIXEL model.
 - For the convenience of the user, Dynamixel2Arduino API hardcodes some information in the control table and stores it in flash.
 - To do this, you need to add code to some files. In this regard, please refer to [PR#3](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/pull/3) and [PR#7](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/pull/7)

## How to create custom PortHandler Class
 - Please refer to [port_handler.h](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/blob/master/src/utility/port_handler.h)
 - Create a new class by inheriting PortHandler as public. (Like SerialPortHandler and USBSerialPortHandler)

## TODO
 - Separation of protocol codes (protocol, packet handler)