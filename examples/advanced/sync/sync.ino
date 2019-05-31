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

#define DXL_CNT 2
uint8_t id_list[DXL_CNT] = {1, 3};
uint32_t data_list[DXL_CNT];
uint32_t recv_data_list[DXL_CNT];
bool led_state_list[DXL_CNT];  

Dynamixel2Arduino dxl(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.scan();

  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_POSITION);
  dxl.torqueOn(1);

  dxl.torqueOff(3);
  dxl.setOperatingMode(3, OP_POSITION);
  dxl.torqueOn(3);
}

void loop() {
  // put your main code here, to run repeatedly:

  // set value to data buffer for syncWrite
  if(data_list[0] >= 4095){
    data_list[0] = 0;
  }else{
    data_list[0] += 5;
  }
  data_list[1] = data_list[0];

  led_state_list[0] = !led_state_list[0];
  led_state_list[1] = !led_state_list[1];

  /** 
   * parameter1 : Address of DYNAMIXEL control table item to write. In example, (116) is address of "Goal Postion" in protocol2.0.
   *             (For details, please refer to control table information of each DYNAMIXEL in http://emanual.robotis.com)
   * parameter2 : Length of DYNAMIXEL control table item to write.
   * parameter3 : List of DYNAMIXEL IDs to write.
   * parameter4 : Number of DYNAMIXEL to write.
   * parameter5 : buffer where the data to be transferred is stored.
   * parameter6 : Max size of buffer where the data to be transferred is stored.
  */
  dxl.syncWrite(116, 4, id_list, DXL_CNT, (uint8_t*)data_list, sizeof(data_list));
  delay(100);

  // parameter1(65) is address of "LED" in protocol2.0 
  //  (For details, please refer to control table information of each DYNAMIXEL in http://emanual.robotis.com)
  dxl.syncWrite(65, 1, id_list, DXL_CNT, (uint8_t*)led_state_list, sizeof(led_state_list));
  delay(100);

  /** 
   * parameter1 : Address of DYNAMIXEL control table item to read. In example, (132) is address of "Present Postion" item in protocol2.0.
   *             (For details, please refer to control table information of each DYNAMIXEL in http://emanual.robotis.com)
   * parameter2 : Length of DYNAMIXEL control table item to read.
   * parameter3 : List of DYNAMIXEL IDs to read.
   * parameter4 : Number of DYNAMIXEL to read.
   * parameter5 : Buffer to store the read data.
   * parameter6 : Max size of buffer to store the read data.
   * parameter7 : Timeout(unit: milliseconds, default: 100ms).
  */
  dxl.syncRead(132, 4, id_list, DXL_CNT, (uint8_t*)recv_data_list, sizeof(recv_data_list));
  delay(100);

  // Print the read data using syncRead
  DEBUG_SERIAL.print("Present Position : ");      
  DEBUG_SERIAL.print(recv_data_list[0]);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(recv_data_list[1]);
  DEBUG_SERIAL.println();
}


