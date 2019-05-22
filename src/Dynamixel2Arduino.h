/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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


#ifndef DYNAMIXEL_2_ARDUINO_H_
#define DYNAMIXEL_2_ARDUINO_H_


#include "utility/master.h"
#include "utility/slave.h"
#include "actuator.h"

namespace DYNAMIXEL{

enum Functions{
  SET_ID,
  SET_BAUD_RATE,

  MULTI_PROTOCOL,

  DIRECTION_CTRL,
  PROFILE_MODE_CTRL,
  TIME_BASED_PROFILE_CTRL,
  OP_MODE_CTRL,
  
  SET_POSITION,
  SET_EXTENDED_POSITION,
  SET_CURRENT_BASED_POSITION,
  GET_POSITION,

  SET_VELOCITY,
  GET_VELOCITY,

  SET_PWM,
  GET_PWM,

  SET_CURRENT,
  GET_CURRENT,

  SET_ACCELERATION,
  SET_POSITION_MOVING_SPEED,
  GET_POSITION_MOVING_SPEED,

  POSITION_PID_GAIN,
  VELOCITY_PI_GAIN,
  FEED_FORWARD_GAIN,
  CW_CCW_COMPLIANCE,
  // COMPLIANCE_MARGIN_CTRL,
  // COMPLIANCE_SLOPE_CTRL,

  GET_HARDWARE_ERROR_STATUS,

  LAST_DUMMY_FUNC = 0xFF
};

}

enum ParamUnit{
  UNIT_RAW = 0,
  UNIT_RATIO,
  UNIT_RPM,
  UNIT_RADIAN,
  UNIT_DEGREE,
  UNIT_MILLI_AMPERE
};

class Dynamixel2Arduino : public DYNAMIXEL::Master{

  public:
    Dynamixel2Arduino(HardwareSerial& port, int dir_pin = -1);
    
    /* For Master configuration */
    void begin(unsigned long baud);
    unsigned long getPortBaud() const;
    
    /* Commands for Slave access & control */
    bool scan(); //broadcast ping
    bool ping(uint8_t id = DXL_BROADCAST_ID);
    uint16_t getModelNumber(uint8_t id);

    bool setID(uint8_t id, uint8_t new_id);
    bool setProtocol(uint8_t id, float version);

    bool torqueOn(uint8_t id);
    bool torqueOff(uint8_t id);
    bool setTorqueEnable(uint8_t id, bool enable);

    bool ledOn(uint8_t id);
    bool ledOff(uint8_t id);
    bool setLedState(uint8_t id, bool state);
    
    bool setGoalVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentVelocity(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setGoalPosition(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentPosition(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setGoalCurrent(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentCurrent(uint8_t id, uint8_t unit = UNIT_RAW);    

    bool setGoalPWM(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentPWM(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setDirectionToNormal(uint8_t id);
    bool setDirectionToReverse(uint8_t id);
    bool setDirection(uint8_t id, bool dir);

    bool setProfileBase(uint8_t id, uint8_t base);
    bool setProfileToVelocityBased(uint8_t id);
    bool setProfileToTimeBased(uint8_t id);

    bool setProfileVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    bool setProfileAcceleration(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    bool setPositionPIDGain(uint8_t id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);
    bool setVelocityPIGain(uint8_t id, uint16_t p_gain, uint16_t i_gain);
    bool setFeedForwardGain(uint8_t id, uint16_t fisrt_gain, uint16_t second_gain);

    bool setCompliance(uint8_t id, 
      uint8_t cw_margin, uint8_t ccw_margin,
      uint8_t cw_slope, uint8_t ccw_slope);

    uint8_t getHardwareError(uint8_t id);



    int32_t readControlTableItem(uint8_t item_idx, 
      uint8_t id, uint32_t timeout = 100);
    bool writeControlTableItem(uint8_t item_idx, 
      uint8_t id, int32_t data, uint32_t timeout = 100);


  private:
    typedef struct IdAndModelNum{
      uint16_t model_num;
      uint8_t id;
    } IdAndModelNum_t;

    DYNAMIXEL::SerialPortHandler dxl_port_;
    
    DYNAMIXEL::param_t  param_;
    DYNAMIXEL::status_t resp_;
    IdAndModelNum_t registered_dxl_[DXLCMD_MAX_NODE];
    uint8_t         registered_dxl_cnt_;
    uint32_t        err_code_;

    uint16_t getModelNumberFromTable(uint8_t id);
};




#endif /* DYNAMIXEL_2_ARDUINO_H_ */