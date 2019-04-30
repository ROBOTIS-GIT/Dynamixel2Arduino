#include <Dynamixel2Arduino.h>


const uint8_t RS485_DIR_PIN = 2; //DYNAMIXEL Shield
const uint8_t DXL_ID = 1;

Dynamixel2Arduino dynamixel_manager(Serial1, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dynamixel_manager.begin(1000000);
  dynamixel_manager.scan();
  dynamixel_manager.torqueOn(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time[4] = {0, };
  static int32_t value = 0;
  static bool led_state = false;

  if(millis() - pre_time[0] >= 500) {
    pre_time[0] = millis();
    led_state == true ? dynamixel_manager.ledOn(DXL_ID) : dynamixel_manager.ledOff(DXL_ID);
    led_state = !led_state;
  }

  if(millis() - pre_time[1] >= 1500) {
    pre_time[1] = millis();
    dynamixel_manager.writeControlTableItem(GOAL_POSITION, DXL_ID, value == 0 ? 4095 : 0);
  }

  if(millis() - pre_time[2] >= 20) {
    pre_time[2] = millis();
    Serial.print("Present Position : ");
    Serial.println(dynamixel_manager.readControlTableItem(PRESENT_POSITION, DXL_ID));
  }

}