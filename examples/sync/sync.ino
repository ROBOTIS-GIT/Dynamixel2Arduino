#include <Dynamixel2Arduino.h>

enum TimerType{
  TIMER_SYNC_WRITE = 0,
  TIMER_SYNC_READ,
  TIMER_SYNC_LED,

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

#define DXL_CNT 2
uint8_t id_list[DXL_CNT] = {1, 3};

Dynamixel2Arduino dynamixel(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dynamixel.begin(1000000);
  dynamixel.scan();

  for(uint8_t i = 0; i < DXL_CNT; i++)
  {
    dynamixel.torqueOff(id_list[i]);
    dynamixel.setOperatingMode(id_list[i], OP_POSITION);
    dynamixel.torqueOn(id_list[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i;
  static uint32_t pre_time[TIMER_MAX];
  static uint32_t data_list[DXL_CNT];
  static uint32_t recv_data_list[DXL_CNT];
  static bool led_state_list[DXL_CNT];  

  if(millis() - pre_time[TIMER_SYNC_WRITE] >= 100) {    
    pre_time[TIMER_SYNC_WRITE] = millis();

    dynamixel.syncWrite(116, 4, id_list, (uint8_t*)data_list, DXL_CNT);

    for(i = 0; i < DXL_CNT; i++)
      data_list[i] = data_list[i] >= 4095 ? 0 : data_list[i]+5;
  }     

  if(millis() - pre_time[TIMER_SYNC_READ] >= 50) {
    pre_time[TIMER_SYNC_READ] = millis();

    dynamixel.syncRead(132, 4, id_list, DXL_CNT,
      (uint8_t*)recv_data_list, sizeof(recv_data_list));

    DEBUG_SERIAL.print("Present Position : ");      
    for(i = 0; i < DXL_CNT; i++)
    {
      DEBUG_SERIAL.print(recv_data_list[i]);
      DEBUG_SERIAL.print(" ");
    }
    DEBUG_SERIAL.println();
  }

  if(millis() - pre_time[TIMER_SYNC_LED] >= 500) {
    pre_time[TIMER_SYNC_LED] = millis();

    dynamixel.syncWrite(65, 1, id_list, (uint8_t*)led_state_list, 2);
    
    for(i = 0; i < DXL_CNT; i++)
      led_state_list[i] = !led_state_list[i];
  }
}


