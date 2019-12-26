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



const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 126;
const uint16_t SR_ADDR_LEN = 10; //2+4+4
const uint16_t SW_START_ADDR = 104; //Goal velocity
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT];
sw_data_t sw_data[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
DYNAMIXEL::SyncRead<SR_START_ADDR, sr_data_t, DXL_ID_CNT> dxl_sr(dxl);
DYNAMIXEL::SyncWrite<SW_START_ADDR, sw_data_t, DXL_ID_CNT> dxl_sw(dxl);


void setup() {
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_VELOCITY);
    dxl.torqueOn(DXL_ID_LIST[i]);
  }

  // SyncRead using user external buffer.
  dxl_sr.setPacketBuffer(user_pkt_buf, user_pkt_buf_cap);
  for(i=0; i<DXL_ID_CNT; i++){
    dxl_sr.addParam(DXL_ID_LIST[i], sr_data[i]);
  }

  // SyncWrite using the internal buffer.
  sw_data[0].goal_velocity = 0;
  sw_data[1].goal_velocity = 100;
  for(i=0; i<DXL_ID_CNT; i++){
    dxl_sw.addParam(DXL_ID_LIST[i], sw_data[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t i, id, recv_cnt;
  
  // Update parameter data for SyncWrite
  for(i=0; i<DXL_ID_CNT; i++){
    sw_data[i].goal_velocity+=5;
    if(sw_data[i].goal_velocity >= 200){
      sw_data[i].goal_velocity = 0;
    }
  }
  dxl_sw.updateParamData();

  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);
  // Send syncWrite
  if(dxl_sw.sendPacket() == true){
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for(i=0; i<DXL_ID_CNT; i++){
      id = dxl_sw.getIDByIndex(i);
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print("\t Goal Velocity: ");DEBUG_SERIAL.println(dxl_sw.getDataPtr(id)->goal_velocity);
    }
  }else{
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  // Send syncRead & Receive data
  recv_cnt = dxl_sr.sendPacket();
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(i=0; i<recv_cnt; i++){
      id = dxl_sr.getIDByIndex(i);
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(dxl_sr.getIDByIndex(i));
      DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(dxl_sr.getError(id));
      DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(dxl_sr.getDataPtr(id)->present_current);
      DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(dxl_sr.getDataPtr(id)->present_velocity);
      DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(dxl_sr.getDataPtr(id)->present_position);
    }
  }else{
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}




