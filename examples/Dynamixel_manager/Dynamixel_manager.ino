#include <Dynamixel2Arduino.h>


enum TimerType{
  TIMER_SET_COMMAND = 0,
  TIMER_GET_COMMAND,
  TIMER_CHANGE_OP_MODE,
  TIMER_SET_LED,

  TIMER_MAX
};

const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
const uint8_t DXL_ID = 1;

Dynamixel2Arduino dynamixel_manager(Serial1, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dynamixel_manager.begin(1000000);
  dynamixel_manager.scan();
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
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalPosition(DXL_ID, value);
        value += 5;
      }     
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present Position : ");
        Serial.println(dynamixel_manager.getPresentPosition(DXL_ID));
      }
      break;
    
    case OP_EXTENDED_POSITION:
      if(flag_op_changed){
        value = 0;
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present Extended Position : ");
        Serial.println(dynamixel_manager.getPresentPosition(DXL_ID));
      }
      break;
    case OP_CURRENT_BASED_POSITION:
      if(flag_op_changed){
        value = 0;
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present Current Based Position : ");
        Serial.println(dynamixel_manager.getPresentPosition(DXL_ID));
      }
      break;

    case OP_VELOCITY:
      if(flag_op_changed){
        value = 0;
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalVelocity(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present Velocity : ");
        Serial.println(dynamixel_manager.getPresentVelocity(DXL_ID));
      }    
      break;

    case OP_PWM:
      if(flag_op_changed){
        value = 0;
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalPWM(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present PWM : ");
        Serial.println(dynamixel_manager.getPresentPWM(DXL_ID));
      }    
      break;

    case OP_CURRENT:
      if(flag_op_changed){
        value = 0;
        dynamixel_manager.torqueOff(DXL_ID);
        if(dynamixel_manager.setOperatingMode(DXL_ID, op_mode) == false){
          op_mode++;
          break;
        }
        dynamixel_manager.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time[TIMER_SET_COMMAND] >= 100) {    
        pre_time[TIMER_SET_COMMAND] = millis();
        dynamixel_manager.setGoalCurrent(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time[TIMER_GET_COMMAND] >= 50) {
        pre_time[TIMER_GET_COMMAND] = millis();
        Serial.print("Present Current : ");
        Serial.println(dynamixel_manager.getPresentCurrent(DXL_ID));
      }
      break;

    default:
      op_mode = OP_POSITION;
      break;
  }

  if(millis() - pre_time[TIMER_CHANGE_OP_MODE] >= (int32_t)10*1000){
    pre_time[TIMER_CHANGE_OP_MODE] = millis();
    op_mode++;
    flag_op_changed = true;
  }

  if(millis() - pre_time[TIMER_SET_LED] >= 500) {
    pre_time[TIMER_SET_LED] = millis();
    led_state == true ? dynamixel_manager.ledOn(DXL_ID) : dynamixel_manager.ledOff(DXL_ID);
    led_state = !led_state;
  }
}