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

#ifdef ARDUINO_AVR_UNO
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); //RX,TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
#elif ARDUINO_AVR_MEGA2560
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL Serial1
  const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
#elif CommXEL_W
  #define DXL_SERIAL    Serial2
  #define DEBUG_SERIAL  Serial
  const uint8_t RS485_DIR_PIN = 15;
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
#endif

/*
  A structure that contains the information needed for the parameters of the 'bulkRead packet'.

  typedef struct ParamForBulkReadInst{
    uint8_t id_count;
    XelInfoForBulkReadParam_t xel[DXL_MAX_NODE]; //refer to below.
  } ParamForBulkReadInst_t;

  typedef struct XelInfoForBulkReadParam{
    uint8_t id;
    uint16_t addr;
    uint16_t length;
  } XelInfoForBulkReadParam_t;
*/
ParamForBulkReadInst_t bulk_read_param;

/*
  A structure that contains the information needed for the parameters of the 'bulkWrite packet'.

  typedef struct ParamForBulkWriteInst{
    uint8_t id_count;
    XelInfoForBulkWriteParam_t xel[DXL_MAX_NODE]; //refer to below. 
  } ParamForBulkWriteInst_t;

  typedef struct XelInfoForBulkWriteParam{
    uint8_t id;
    uint16_t addr;
    uint16_t length;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForBulkWriteParam_t;
*/
ParamForBulkWriteInst_t bulk_write_param;

/*
  A structure used to receive data from multiple XELs.

  typedef struct RecvInfoFromStatusInst{
    uint8_t id_count;
    XelInfoForStatusInst_t xel[DXL_MAX_NODE]; //refer to below.
  } RecvInfoFromStatusInst_t;

  typedef struct XelInfoForStatusInst{
    uint8_t id;
    uint16_t length;
    uint8_t error;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForStatusInst_t;
*/
RecvInfoFromStatusInst_t read_result;

Dynamixel2Arduino dxl(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.scan();
 
  // fill the members of structure for bulkWrite
  bulk_write_param.xel[0].id = 1;
  bulk_write_param.xel[1].id = 3;
  bulk_write_param.xel[0].addr = 116; //Goal Position on X serise
  bulk_write_param.xel[1].addr = 104; //Goal Velocity on X serise
  bulk_write_param.xel[0].length = 4;
  bulk_write_param.xel[1].length = 4;
  bulk_write_param.id_count = 2;

  // fill the members of structure for bulkRead
  bulk_read_param.xel[0].id = 1;
  bulk_read_param.xel[1].id = 3;
  bulk_read_param.xel[0].addr = 132; //Present Position on X serise
  bulk_read_param.xel[1].addr = 128; //Present Velocity on X serise
  bulk_read_param.xel[0].length = 4;
  bulk_read_param.xel[1].length = 4;  
  bulk_read_param.id_count = 2;

  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_POSITION);
  dxl.torqueOn(1);

  dxl.torqueOff(3);
  dxl.setOperatingMode(3, OP_VELOCITY);
  dxl.torqueOn(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t position = 0, velocity = 0;
  int32_t recv_position = 0, recv_velocity = 0;

  // set value to data buffer for bulkWrite
  position = position >= 4095 ? 0 : position+409;
  memcpy(bulk_write_param.xel[0].data, &position, sizeof(position));
  velocity = velocity >= 200 ? -200 : velocity+10;
  memcpy(bulk_write_param.xel[1].data, &velocity, sizeof(velocity));

  // send command using bulkWrite
  dxl.bulkWrite(bulk_write_param);
  delay(100);

  // Print the read data using bulkRead
  dxl.bulkRead(bulk_read_param, read_result);
  DEBUG_SERIAL.println("======= Bulk Read ========");
  memcpy(&recv_position, read_result.xel[0].data, read_result.xel[0].length);
  memcpy(&recv_velocity, read_result.xel[1].data, read_result.xel[1].length);
  DEBUG_SERIAL.print("ID: ");DEBUG_SERIAL.print(read_result.xel[0].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Position: ");DEBUG_SERIAL.print(recv_position);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");DEBUG_SERIAL.print(read_result.xel[0].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");DEBUG_SERIAL.print(read_result.xel[0].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("ID: ");DEBUG_SERIAL.print(read_result.xel[1].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Velocity: ");DEBUG_SERIAL.print(recv_velocity);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");DEBUG_SERIAL.print(read_result.xel[1].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");DEBUG_SERIAL.print(read_result.xel[1].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
  delay(100);
}


