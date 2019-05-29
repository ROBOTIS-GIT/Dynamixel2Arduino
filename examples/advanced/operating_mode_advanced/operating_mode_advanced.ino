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
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.scan();
}

void loop() {
  // put your main code here, to run repeatedly:
  static float value = 0;
  static bool led_state, flag_op_changed = true;
  static uint8_t op_mode = OP_POSITION;

  switch(op_mode)
  {
    case OP_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 5;
      }     
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;
    
    case OP_EXTENDED_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Extended Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;
      
    case OP_CURRENT_BASED_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Current Based Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;

    case OP_VELOCITY:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalVelocity(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Velocity : ");
        DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
      }    
      break;

    case OP_PWM:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPWM(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present PWM : ");
        DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID));
      }    
      break;

    case OP_CURRENT:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalCurrent(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Current : ");
        DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID));
      }
      break;

    default:
      op_mode = OP_POSITION;
      break;
  }

  if(millis() - pre_time_op_mode >= (uint32_t)60*1000){
    pre_time_op_mode = millis();
    op_mode++;
    flag_op_changed = true;
  }

  if(millis() - pre_time_led >= 500) {
    pre_time_led = millis();
    led_state == true ? dxl.ledOn(DXL_ID) : dxl.ledOff(DXL_ID);
    led_state = !led_state;
  }
}