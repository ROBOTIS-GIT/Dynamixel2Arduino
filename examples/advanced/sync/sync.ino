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

#define DXL_CNT 2
uint8_t id_list[DXL_CNT] = {1, 3};

Dynamixel2Arduino dynamixel(DXL_SERIAL, RS485_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dynamixel.begin(1000000);
  dynamixel.scan();

  dynamixel.torqueOff(1);
  dynamixel.setOperatingMode(1, OP_POSITION);
  dynamixel.torqueOn(1);

  dynamixel.torqueOff(3);
  dynamixel.setOperatingMode(3, OP_POSITION);
  dynamixel.torqueOn(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i;
  static uint32_t pre_time_sync_write, pre_time_sync_read, pre_time_led;
  static uint32_t data_list[DXL_CNT];
  static uint32_t recv_data_list[DXL_CNT];
  static bool led_state_list[DXL_CNT];  

  if(millis() - pre_time_sync_write >= 100) {    
    pre_time_sync_write = millis();

    if(data_list[0] >= 4095){
      data_list[0] = 0;
    }else{
      data_list[0] += 5;
    }
    data_list[1] = data_list[0];

    dynamixel.syncWrite(116, 4, id_list, DXL_CNT, (uint8_t*)data_list, sizeof(data_list));
  }     

  if(millis() - pre_time_sync_read >= 50) {
    pre_time_sync_read = millis();

    dynamixel.syncRead(132, 4, id_list, DXL_CNT, (uint8_t*)recv_data_list, sizeof(recv_data_list));

    DEBUG_SERIAL.print("Present Position : ");      
    DEBUG_SERIAL.print(recv_data_list[0]);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(recv_data_list[1]);
    DEBUG_SERIAL.println();
  }

  if(millis() - pre_time_led >= 500) {
    pre_time_led = millis();

    led_state_list[0] = !led_state_list[0];
    led_state_list[1] = !led_state_list[1];

    dynamixel.syncWrite(65, 1, id_list, DXL_CNT, (uint8_t*)led_state_list, sizeof(led_state_list));
  }
}


