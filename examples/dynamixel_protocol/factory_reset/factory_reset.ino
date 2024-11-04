/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

#define TIMEOUT 10    //default communication timeout 10ms
const uint8_t DXL_ID = 1;
// MX and AX servos use DYNAMIXEL Protocol 1.0 by default.
// to use MX and AX servos with this example, change the following line to: const float DXL_PROTOCOL_VERSION = 1.0;
const float DXL_PROTOCOL_VERSION = 2.0;

bool ret = false;
uint8_t option = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  // put your main code here, to run repeatedly:
  ret = false;

  // DYNAMIXEL protocol version 2.0 instruction
  if(DXL_PROTOCOL_VERSION == 2.0) {
    DEBUG_SERIAL.println("Select Reset Option:");
    DEBUG_SERIAL.println("[1] Reset all value");
    DEBUG_SERIAL.println("[2] Reset all value except ID");
    DEBUG_SERIAL.println("[3] Reset all value except ID and Baudrate");

    DEBUG_SERIAL.read();
    while(DEBUG_SERIAL.available()==0);
    option = DEBUG_SERIAL.read();

    switch(option) {
      case '1': // reset all value
        // if DXL_ID is Broadcast ID, this instruction will not work
        ret = dxl.factoryReset(DXL_ID, 0xFF, TIMEOUT);
        break;
      case '2': // reset all value except ID
        ret = dxl.factoryReset(DXL_ID, 0x01, TIMEOUT);
        break;
      case '3': // reset all value except ID and Baudrate
        ret = dxl.factoryReset(DXL_ID, 0x02, TIMEOUT);
        break;
      default:
        break;
    }
  }
  //DYNAMIXEL protocol version 1.0 instruction
  else {
    // if DXL_ID is Broadcast ID, this instruction will not work
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Proceed Factory Reset? [y/n]");

    DEBUG_SERIAL.read();
    while(DEBUG_SERIAL.available()==0);
    option = DEBUG_SERIAL.read();

    switch(option) {
      case 'y':
      case 'Y':
        ret = dxl.factoryReset(DXL_ID, 0xFF, TIMEOUT);
        break;
      default:
        break;
    }
  }

  if(ret) {
    DEBUG_SERIAL.println("factory reset succeeded!");
  } else {
    DEBUG_SERIAL.println("factory reset failed!");
  }

  delay(1000);
}
