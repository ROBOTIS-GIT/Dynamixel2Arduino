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

// Author: Ashe Kim

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
#define DXL_MOVING_STATUS_THRESHOLD  10

/* FastsyncRead
  DYNAMIXEL PROTOCOL 1.0 does NOT support Sync Read feature.
  Structures containing the necessary information to process the 'fastSyncRead' packet.

  struct XELInfoSyncRead_t {
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed));

  struct InfoSyncReadInst_t {
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed));
*/

/* syncWrite
  DYNAMIXEL PROTOCOL 1.0 supports Control Table address up to 255.
  Structures containing the necessary information to process the 'syncWrite' packet.

  struct XELInfoSyncWrite_t {
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed));

  struct InfoSyncWriteInst_t {
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed));
*/

const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t BROADCAST_ID = 254;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 132; //Present position
const uint16_t SR_ADDR_LEN = 4;
const uint16_t SW_START_ADDR = 116; //Goal position
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

int32_t dxl_goal_position = 1024;
char keyboard;

void setup() {
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to fastSyncRead using external user packet buffer
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;


  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  // Insert a initial Position to the syncWrite Packet
  for(i = 0; i < DXL_ID_CNT; i++){
    sw_data[i].goal_position = dxl_goal_position;
  }
  
  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

void loop() {
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  if (Serial.available()) {
    keyboard = Serial.read();
    if (keyboard == 'u') { // position increase
      dxl_goal_position = dxl_goal_position + 256;
    }
    else if (keyboard == 'd') { // position decrease
      dxl_goal_position = dxl_goal_position - 256;
    }
    else if (keyboard == 'r') { // position reset
      dxl_goal_position = 1024;
    }
  }

  // Insert a new Goal Position to the syncWrite Packet
  for(i = 0; i < DXL_ID_CNT; i++){
    sw_data[i].goal_position = dxl_goal_position;
  }

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);

  // Build a syncWrite Packet and transmit to DYNAMIXEL
  if(dxl.syncWrite(&sw_infos) == true){
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for(i=0; i<sw_infos.xel_count; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(sw_data[i].goal_position);
    }   
  }else{
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(300);

  // Transmit predefined fastSyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = dxl.fastSyncRead(&sr_infos);
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[fastSyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(i=0; i<recv_cnt; i++) {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Present Position: ");
      DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  } else {
    DEBUG_SERIAL.print("[fastSyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(1000);
}
