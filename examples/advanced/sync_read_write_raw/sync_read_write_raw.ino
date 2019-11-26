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
const uint16_t recv_buf_cap = 128;
uint8_t recv_buf[recv_buf_cap];
int32_t goal_vel = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.scan();
  
  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_VELOCITY);
  dxl.torqueOn(1);

  dxl.torqueOff(3);
  dxl.setOperatingMode(3, OP_VELOCITY);
  dxl.torqueOn(3);

  // Using user buffer
  sr_present_pos.p_packet_buf = user_pkt_buf;
  sr_present_pos.packet_buf_capacity = user_pkt_buf_cap;

  dxl.beginSyncRead(132, 4, &sr_present_pos);
  dxl.addSyncReadID(1, &sr_present_pos);
  dxl.addSyncReadID(3, &sr_present_pos, true);

  // Using class default buffer
  dxl.beginSyncWrite(104, 4);
  goal_vel = 100;
  dxl.addSyncWriteData(1, (uint8_t*)&goal_vel);
  goal_vel = 200;
  dxl.addSyncWriteData(3, (uint8_t*)&goal_vel);
  dxl.sendSyncWrite();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t recv_len;
  
  recv_len = dxl.sendSyncRead(recv_buf, recv_buf_cap, &sr_present_pos);
  if(recv_len > 0){
    DEBUG_SERIAL.print("\t Receive Len : ");
    DEBUG_SERIAL.print(recv_len);
    DEBUG_SERIAL.println();
    for(uint16_t i=0; i<recv_len; i++){
      DEBUG_SERIAL.print(recv_buf[i], HEX);DEBUG_SERIAL.print(" ");
    }
    DEBUG_SERIAL.println();
  }else{
    DEBUG_SERIAL.print("\t Fail!!   ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
    DEBUG_SERIAL.println();
  }
  delay(500);
}


