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

// Example Environment
//
// - Controller: CM-550 (Main Controller for ROBOTIS Engineer Kit2 )
// - Library: DYNAMIXEL2Arduino 
// - Description: Arduino your CM-550 !  (Read / Write to CM-550 at Arduino IDE)
// - NOTE: The experimental codes may not gurantee full technical support. Note that the given code is subject to change / delete without any notice. 
// Author: David Park

#include <Dynamixel2Arduino.h>

#define CM550_SERIAL Serial2  // OpenCM9.04 UART2. 
Dynamixel2Arduino cm550(CM550_SERIAL); 

const int DXL_DIR_PIN = 28; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#define DXL_SERIAL   Serial1  // OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define DEBUG_SERIAL Serial   // USB

// CM550 Control Table Address 
#define ID_ADDR         7
#define ID_ADDR_LEN     1

#define ROLL_ADDR       102
#define ROLL_ADDR_LEN   2

#define PITCH_ADDR      104
#define PITCH_ADDR_LEN  2

#define YAW_ADDR        106
#define YAW_ADDR_LEN    2

#define LED_ADDR        91 // RED_LED 91, Green_LED 92, Blue_LED 93
#define LED_ADDR_LEN    1

#define TIMEOUT 10

uint8_t returned_id = 0;

int16_t returned_roll = 0;
int16_t returned_pitch = 0;
int16_t returned_yaw = 0;

const uint8_t CM550_ID = 200;
const uint8_t DXL_ID = 1;

const float DXL_PROTOCOL_VERSION = 2.0;

uint8_t turn_on = 1;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // Use Serial to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  cm550.begin(57600);
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  cm550.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  if(cm550.ping(CM550_ID) == true){
    DEBUG_SERIAL.print("CM550 ping succeeded!");
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.println(cm550.getModelNumber(CM550_ID));
  }else{
    DEBUG_SERIAL.print("ping failed!, err code: ");
    DEBUG_SERIAL.println(cm550.getLastLibErrCode());
  }
  delay(100);

  if(dxl.ping(DXL_ID) == true){
    DEBUG_SERIAL.print("DXL ping succeeded!");
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.println(dxl.getModelNumber(DXL_ID));
  }else{
    DEBUG_SERIAL.print("ping failed!, err code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  delay(100);

    // Read DYNAMIXEL ID
  cm550.read(CM550_ID, ID_ADDR, ID_ADDR_LEN, (uint8_t*)&returned_id, sizeof(returned_id), TIMEOUT);
  DEBUG_SERIAL.print("ID : ");
  DEBUG_SERIAL.println(returned_id);
  delay(100);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 50);
}

void loop() {
  // put your main code here, to run repeatedly:
  // This will controls the UART2 (Serial2)
  UART2_READ_WRITE_CM550();
  // This will controls the DXL ports (Serial1, OpenCM9.04 without OpenCM485 Expansion Board)
  DXL_PORT_DXL_POSITION();
}

void UART2_READ_WRITE_CM550(){

  // Read CM-550 Roll Data 
  cm550.read(CM550_ID, ROLL_ADDR, ROLL_ADDR_LEN, (uint8_t*)&returned_roll, sizeof(returned_roll), TIMEOUT);
  DEBUG_SERIAL.print("ROLL : ");
  DEBUG_SERIAL.println(returned_roll);

  // Read CM-550 PITCH Data 
  cm550.read(CM550_ID, PITCH_ADDR, PITCH_ADDR_LEN, (uint8_t*)&returned_pitch, sizeof(returned_pitch), TIMEOUT);
  DEBUG_SERIAL.print("PITCH : ");
  DEBUG_SERIAL.println(returned_pitch);

  // Read CM-550 YAW Data 
  cm550.read(CM550_ID, YAW_ADDR, YAW_ADDR_LEN, (uint8_t*)&returned_yaw, sizeof(returned_yaw), TIMEOUT);
  DEBUG_SERIAL.print("YAW : ");
  DEBUG_SERIAL.println(returned_yaw);
  
  delay(1000);  
  
  // DEBUG_SERIAL.println("Write `1` to Red LED");
  if (cm550.write(CM550_ID, LED_ADDR, (uint8_t*)&turn_on, LED_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Success to Turn On Red LED");
  else
    DEBUG_SERIAL.println("Failed to Turn On Red LED");
  delay(200);

  if (cm550.write(CM550_ID, LED_ADDR + 1, (uint8_t*)&turn_on, LED_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Success to Turn On Green LED");
  else
    DEBUG_SERIAL.println("Failed to Turn On Green LED");
  delay(200);

  if (cm550.write(CM550_ID, LED_ADDR + 2, (uint8_t*)&turn_on, LED_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Success to Turn On Blue LED");
  else
    DEBUG_SERIAL.println("Failed to Turn On Blue LED");
  delay(200);
}

void DXL_PORT_DXL_POSITION(){

  // Set Goal Position in DEGREE value
  DEBUG_SERIAL.print("Goal Position: ");DEBUG_SERIAL.println("512");
  if (dxl.setGoalPosition(DXL_ID, 512))
    DEBUG_SERIAL.println("Goal Position Success:");
  delay(1000);

  DEBUG_SERIAL.print("Goal Position: ");DEBUG_SERIAL.println("0");
  if(dxl.setGoalPosition(DXL_ID, 0))
    DEBUG_SERIAL.println("Goal Position Success:");
  delay(1000);
}