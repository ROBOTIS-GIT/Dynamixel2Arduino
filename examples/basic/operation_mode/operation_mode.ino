#include <Dynamixel2Arduino.h>

/** 
 * Operating Mode
 *  1. OP_POSITION                (Position Mode in protocol2.0, Joint Mode in protocol1.0)
 *  2. OP_VELOCITY                (Velocity Mode in protocol2.0, Speed Mode in protocol1.0)
 *  3. OP_PWM                     (PWM Mode in protocol2.0)
 *  4. OP_EXTENDED_POSITION       (Extended Position Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0)
 *  5. OP_CURRENT                 (Current Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0)
 *  6. OP_CURRENT_BASED_POSITION  (Current Based Postion Mode in protocol2.0 (except MX28, XL430))
 */


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

const uint8_t DXL_ID = 1;

Dynamixel2Arduino dxl(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Position Mode in protocol2.0, Joint Mode in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 512)){
    delay(1000);
    DEBUG_SERIAL.print("Present Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }

  // Velocity Mode in protocol2.0, Speed Mode in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalVelocity(DXL_ID, 128)){
    delay(1000);
    DEBUG_SERIAL.print("Present Velocity : ");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID)); DEBUG_SERIAL.println();
  }

  // Extended Position Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 4096)){
    delay(1000);
    DEBUG_SERIAL.print("Present Extended Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }

  // PWM Mode in protocol2.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPWM(DXL_ID, 200)){
    delay(1000);
    DEBUG_SERIAL.print("Present PWM : ");
    DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID)); DEBUG_SERIAL.println();
  }
  
  // Current Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalCurrent(DXL_ID, 256)){
    delay(1000);
    DEBUG_SERIAL.print("Present Current : ");
    DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID)); DEBUG_SERIAL.println();
  }
  
  // Current Based Postion Mode in protocol2.0 (except MX28, XL430)
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  if(dxl.setGoalPosition(DXL_ID, 8192)){
    delay(1000);
    DEBUG_SERIAL.print("Present Current Based Position : ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID)); DEBUG_SERIAL.println();
  }
}
