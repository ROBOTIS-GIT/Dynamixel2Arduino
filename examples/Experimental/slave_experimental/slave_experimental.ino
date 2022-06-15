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
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

bool temp = false; 

// Tested: OpenRB-150 
// 

// Create a port object for DYNAMIXEL communication.
// The list of available classes is as follows.
// 1) DYNAMIXEL::SerialPortHandler     (HardwareSerial only)
//     -note: If you do not want to use half duplex communication, do not enter the second parameter.
// 2) DYNAMIXEL::USBSerialPortHandler  (Only USB port on each board)

// What protocol rule to communicate 
const float DXL_PROTOCOL_VER = 2.0;
// Model Number Address is natively defined by Slave::addDefaultControlItem()
const uint16_t DXL_MODEL_NUM = 0x5005; // Modify it to what you want.

//   DYNAMIXEL::SerialPortHandler dxl_port(DXL_SERIAL, DXL_DIR_PIN);  // For Hardware Serial Only 
DYNAMIXEL::USBSerialPortHandler dxl_port(DEBUG_SERIAL); // USB Only. 
DYNAMIXEL::Slave dxl_slave(dxl_port, DXL_MODEL_NUM);

////////////// Adding Custom Control Table Examples ////////////////////////
// Declare the address of the Slave control item you want 
// to register and the variable (size is also important)
// to store its data.
// const uint16_t ADDR_CONTROL_ITEM_LED = 10;
// const uint16_t ADDR_CONTROL_ITEM_ANALOG = 20;
// note: 'int' is not supported because its size varies by system architecture.
// uint8_t control_item_led;
// int16_t control_item_analog;
///////////////////////////////////////////////////////////////////////////

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl_slave.begin();

  // Hardware Serial Speed setting for communication (not necessary for USB)
  // dxl_port.begin(1000000);

  dxl_slave.setPortProtocolVersion(DXL_PROTOCOL_VER); 
  // Your Firmware Version. 
  dxl_slave.setFirmwareVersion(1); 
  // Controller ID 
  dxl_slave.setID(200);
}

void loop() {
  // put your main code here, to run repeatedly:

  // If there is a packet from the master, process it.
  if(dxl_slave.processPacket() == false){
    DEBUG_SERIAL.print("Last lib err code: ");
    DEBUG_SERIAL.print(dxl_slave.getLastLibErrCode());
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print("Last status packet err code: ");
    DEBUG_SERIAL.println(dxl_slave.getLastStatusPacketError());
    DEBUG_SERIAL.println();
  }

}