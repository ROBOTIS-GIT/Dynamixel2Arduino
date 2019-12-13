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
#elif ARDUINO_OpenCM904 // Official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif ARDUINO_OpenCR // Official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

/* InfoSyncBulkInst_t
  A structure that contains the information needed for the parameters of the 'Sync/Bulk Read/Write packet'.

typedef struct InfoSyncBulkInst{
  uint8_t* p_packet_buf;
  uint16_t packet_buf_capacity;
  uint16_t packet_length;
  uint8_t id_cnt;
  bool is_complete_packet;
} InfoSyncBulkInst_t;
*/

DYNAMIXEL::InfoSyncBulkInst_t sr_present_pos;
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint8_t DXL_ID_CNT = 2;
const uint16_t SR_START_ADDR = 126;
const uint16_t SR_ADDR_LEN = 10; //2+4+4
const uint16_t SW_START_ADDR = 104; //Goal velocity
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} sr_data_t;
uint8_t id_list[DXL_ID_CNT] = {1, 2};
sr_data_t present_values[DXL_ID_CNT];
uint8_t err_list[DXL_ID_CNT] = {0, };
int32_t goal_velocity[DXL_ID_CNT] = {100, 200};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  uint8_t i;
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(id_list[i]);
    dxl.setOperatingMode(id_list[i], OP_VELOCITY);
  }
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOn(id_list[i]);
  }

  // Generate SyncRead packet using user buffer
  sr_present_pos.p_packet_buf = user_pkt_buf;
  sr_present_pos.packet_buf_capacity = user_pkt_buf_cap;
  dxl.beginSetupSyncRead(SR_START_ADDR, SR_ADDR_LEN, &sr_present_pos);
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.addSyncReadID(id_list[i], &sr_present_pos);
  }
  dxl.endSetupSyncRead(&sr_present_pos);

  // Generate SyncRead packet using class default buffer
  dxl.beginSetupSyncWrite(SW_START_ADDR, SW_ADDR_LEN);
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.addSyncWriteData(id_list[i], (uint8_t*)&goal_velocity[i]);
  }
  dxl.endSetupSyncWrite();
  dxl.doSyncWrite();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t recv_cnt;
  
  recv_cnt = dxl.doSyncRead((uint8_t*)&present_values, sizeof(present_values), err_list, sizeof(err_list), &sr_present_pos);
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(uint16_t i=0; i<recv_cnt; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(id_list[i]);
      DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(err_list[i]);
      DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(present_values[i].present_current);
      DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(present_values[i].present_velocity);
      DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(present_values[i].present_position);
    }
    DEBUG_SERIAL.println();
  }else{
    DEBUG_SERIAL.print("\t Fail!!   ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
    DEBUG_SERIAL.println();
  }
  delay(500);
}


