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
 

const uint8_t DXL_1_ID = 1;
const uint8_t DXL_2_ID = 2;
const uint8_t DXL_ID_CNT = 2;
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

struct br_data_xel_1{
  int16_t present_current;
  int32_t present_velocity;
} __attribute__((packed));
struct br_data_xel_2{
  int32_t present_position;
} __attribute__((packed));
struct bw_data_xel_1{
  int32_t goal_velocity;
} __attribute__((packed));
struct bw_data_xel_2{
  int32_t goal_position;
} __attribute__((packed));

struct br_data_xel_1 br_data_xel_1;
struct br_data_xel_2 br_data_xel_2;
struct bw_data_xel_1 bw_data_xel_1;
struct bw_data_xel_2 bw_data_xel_2;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DYNAMIXEL::BulkRead<DXL_ID_CNT> dxl_br(dxl);
DYNAMIXEL::BulkWrite<DXL_ID_CNT> dxl_bw(dxl);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  
  dxl.torqueOff(DXL_1_ID);
  dxl.torqueOff(DXL_2_ID);
  dxl.setOperatingMode(DXL_1_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL_2_ID, OP_POSITION);
  dxl.torqueOn(DXL_1_ID);
  dxl.torqueOn(DXL_2_ID);
  
  // BulkRead using user external packet buffer.
  dxl_br.setPacketBuffer(user_pkt_buf, user_pkt_buf_cap);
  dxl_br.addParam(DXL_1_ID, 126, br_data_xel_1); // Start Addr: Present Current of X serise.
  dxl_br.addParam(DXL_2_ID, 132, br_data_xel_2); // Start Addr: Present Position of X serise.

  // BulkWrite using the internal packet buffer.
  bw_data_xel_1.goal_velocity = 0;
  bw_data_xel_2.goal_position = 0;
  dxl_bw.addParam(DXL_1_ID, 104, bw_data_xel_1); // Start Addr: Goal Velocity of X serise. 
  dxl_bw.addParam(DXL_2_ID, 116, bw_data_xel_2); // Start Addr: Goal Position of X serise.
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t recv_cnt;

  DEBUG_SERIAL.print("\n>>>>>> Bulk Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);

  // Update parameter data for BulkWrite
  bw_data_xel_1.goal_velocity+=5;
  if(bw_data_xel_1.goal_velocity >= 200){
    bw_data_xel_1.goal_velocity = 0;
  }
  bw_data_xel_2.goal_position+=255;
  if(bw_data_xel_2.goal_position >= 1023){
    bw_data_xel_2.goal_position = 0;
  }
  dxl_bw.updateParamData();

  // Send bulkWrite pakcet
  if(dxl_bw.sendPacket() == true){
    DEBUG_SERIAL.println("[BulkWrite] Success");

    DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.println(DXL_1_ID);
    DEBUG_SERIAL.print("\t Start Address: ");DEBUG_SERIAL.println(dxl_bw.getStartAddr(DXL_1_ID));
    DEBUG_SERIAL.print("\t Address Length: ");DEBUG_SERIAL.println(dxl_bw.getAddrLength(DXL_1_ID));
    DEBUG_SERIAL.print("\t Goal Velocity: ");DEBUG_SERIAL.println(bw_data_xel_1.goal_velocity);

    DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.println(DXL_2_ID);
    DEBUG_SERIAL.print("\t Start Address: ");DEBUG_SERIAL.println(dxl_bw.getStartAddr(DXL_2_ID));
    DEBUG_SERIAL.print("\t Address Length: ");DEBUG_SERIAL.println(dxl_bw.getAddrLength(DXL_2_ID));
    DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(bw_data_xel_2.goal_position);
  }else{
    DEBUG_SERIAL.print("[BulkWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  // Send bulkRead packet & Receive data
  recv_cnt = dxl_br.sendPacket();
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[BulkRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);

    DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(DXL_1_ID);
    DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(dxl_br.getError(DXL_1_ID));
    DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(br_data_xel_1.present_current);
    DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(br_data_xel_1.present_velocity);

    DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(DXL_2_ID);
    DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(dxl_br.getError(DXL_2_ID));   
    DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(br_data_xel_2.present_position);
  }else{
    DEBUG_SERIAL.print("[BulkRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}

