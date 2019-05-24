#include <Dynamixel2Arduino.h>


enum TimerType{
  TIMER_SET_COMMAND = 0,
  TIMER_GET_COMMAND,
  TIMER_CHANGE_OP_MODE,
  TIMER_SET_LED,

  TIMER_MAX
};

#ifdef ARDUINO_AVR_UNO
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(10, 11); //RX,TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
#elif ARDUINO_AVR_MEGA2560
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL Serial1
#elif CommXEL_W
  #define DXL_SERIAL    Serial2
  #define DEBUG_SERIAL  Serial
  #define RS485_DIR_PIN 15
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
#endif

#ifndef RS485_DIR_PIN
  const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
#endif

const uint8_t DXL_ID = 1;

Dynamixel2Arduino dynamixel(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dynamixel.begin(1000000);
  dynamixel.scan();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time[TIMER_MAX] = {0, };
  static float value = 0;
  static bool led_state, flag_op_changed = true;
  static uint8_t op_mode = OP_POSITION;

  switch(op_mode)
  {
    case OP_POSITION:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalPosition(DXL_ID, value);
        value += 5;
      }     
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present Position : ");
        DEBUG_SERIAL.println(dynamixel.getPresentPosition(DXL_ID));
      }
      break;
    
    case OP_EXTENDED_POSITION:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present Extended Position : ");
        DEBUG_SERIAL.println(dynamixel.getPresentPosition(DXL_ID));
      }
      break;
    case OP_CURRENT_BASED_POSITION:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present Current Based Position : ");
        DEBUG_SERIAL.println(dynamixel.getPresentPosition(DXL_ID));
      }
      break;

    case OP_VELOCITY:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalVelocity(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present Velocity : ");
        DEBUG_SERIAL.println(dynamixel.getPresentVelocity(DXL_ID));
      }    
      break;

    case OP_PWM:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalPWM(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present PWM : ");
        DEBUG_SERIAL.println(dynamixel.getPresentPWM(DXL_ID));
      }    
      break;

    case OP_CURRENT:
      if(flag_op_changed){
        value = 0;
        dynamixel.torqueOff(DXL_ID);
        if(dynamixel.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel.setGoalCurrent(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        DEBUG_SERIAL.print("Present Current : ");
        DEBUG_SERIAL.println(dynamixel.getPresentCurrent(DXL_ID));
      }
      break;

    default:
      op_mode = OP_POSITION;
      break;
  }

  if(millis() - pre_time[TIMER_CHANGE_OP_MODE] >= (uint32_t)60*1000){
    pre_time[TIMER_CHANGE_OP_MODE] = millis();
    op_mode++;
    flag_op_changed = true;
  }

  if(millis() - pre_time[TIMER_SET_LED] >= 500) {
    pre_time[TIMER_SET_LED] = millis();
    led_state == true ? dynamixel.ledOn(DXL_ID) : dynamixel.ledOff(DXL_ID);
    led_state = !led_state;
  }
}