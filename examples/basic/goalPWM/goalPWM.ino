#include <Dynamixel2Arduino.h>

#ifdef ARDUINO_AVR_UNO
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(10, 11); //RX,TX
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
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(DXL_ID);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
   
  // Please refer to e-Manual(http://emanual.robotis.com) for available range of value. 
  // Set goalPWM using RAW unit
  dxl.setGoalPWM(DXL_ID, 300);
  delay(1000);
  // Print present PWM
  DEBUG_SERIAL.print("Present PWM(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID));
  delay(1000);

  // using RATIO unit (-100.0 ~ 100.0)
  dxl.setGoalPWM(DXL_ID, -40.8, UNIT_RATIO);
  delay(1000);
  DEBUG_SERIAL.print("Present PWM(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID, UNIT_RATIO));
  delay(1000);
}
