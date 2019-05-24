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

/**
 * @Dynamixel2Arduino.h
 * @author Kei
 * @date 24 May 2019
 * @brief This file defines a class for controlling the DYNAMIXEL Actuators in Arduino boards.
 *
 * Supported actors are hardcoded, defined in actuator.cpp and Dynamixel2Arduino.cpp, and can be enabled or disabled in config.h.
 * Therefore, in order to support the new Actuator, hard coding must be done.
 * 
 * @see 
 * @see 
 */


#ifndef DYNAMIXEL_2_ARDUINO_H_
#define DYNAMIXEL_2_ARDUINO_H_


#include "utility/master.h"
#include "utility/slave.h"
#include "actuator.h"

namespace DYNAMIXEL{

enum Functions{
  SET_ID,
  SET_BAUD_RATE,

  SET_PROTOCOL,
 
  SET_POSITION,
  GET_POSITION,

  SET_VELOCITY,
  GET_VELOCITY,

  SET_PWM,
  GET_PWM,

  SET_CURRENT,
  GET_CURRENT,

  DIRECTION_CTRL,
//  PROFILE_MODE_CTRL,
//  TIME_BASED_PROFILE_CTRL,

  POSITION_PID_GAIN,
  VELOCITY_PI_GAIN,
  FEED_FORWARD_GAIN,

  LAST_DUMMY_FUNC = 0xFF
};

}

enum OperatingMode{
  OP_POSITION = 0,
  OP_EXTENDED_POSITION,
  OP_CURRENT_BASED_POSITION,
  OP_VELOCITY,
  OP_PWM,
  OP_CURRENT,

  UNKNOWN_OP
};

enum ParamUnit{
  UNIT_RAW = 0,
  UNIT_RATIO,
  UNIT_RPM,
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
    bool setBaudrate(uint8_t id, uint32_t baudrate);

    bool torqueOn(uint8_t id);
    bool torqueOff(uint8_t id);

    bool ledOn(uint8_t id);
    bool ledOff(uint8_t id);

    /**
     * @brief It is API for controlling operating mode of DYNAMIXEL.
     * @code
     * Dynamixel2Arduino dynamixel_manager(Serial1, 2);
     * dynamixel_manager.setGoalPosition(1, 512);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param mode The operating mode you want.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setOperatingMode(uint8_t id, uint8_t mode);

    /**
     * @brief It is API for controlling position(joint) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * Dynamixel2Arduino dynamixel_manager(Serial1, 2);
     * dynamixel_manager.setGoalPosition(1, 512);
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param value The value you want to set.
     * @param unit The unit of the value you entered.
     * @return It returns true(1) on success, false(0) on failure.
     */
    bool setGoalPosition(uint8_t id, float value, uint8_t unit = UNIT_RAW);

    /**
     * @brief It is API for getting present position (joint) of DYNAMIXEL.
     * You can use the @unit parameter to specify the unit of @value.
     * @code
     * Dynamixel2Arduino dynamixel_manager(Serial1, 2);
     * Serial.print(dynamixel_manager.getPresentPosition(1));
     * @endcode
     * @param id DYNAMIXEL Actuator's ID.
     * @param The unit you want to return (the function converts the raw value to the unit you specified and returns it)
     *    Only support UNIT_RAW, UNIT_DEGREE.
     * @return It returns the data readed from DXL control table item.
     * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastErrorCode().
     */    
    float getPresentPosition(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setGoalVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentVelocity(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setGoalPWM(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentPWM(uint8_t id, uint8_t unit = UNIT_RAW);

    bool setGoalCurrent(uint8_t id, float value, uint8_t unit = UNIT_RAW);
    float getPresentCurrent(uint8_t id, uint8_t unit = UNIT_RAW);    



    int32_t readControlTableItem(uint16_t model_num,
      uint8_t item_idx, uint8_t id, uint32_t timeout = 100);

    int32_t readControlTableItem(uint8_t item_idx, 
      uint8_t id, uint32_t timeout = 100);

    bool writeControlTableItem(uint8_t item_idx, 
      uint8_t id, int32_t data, uint32_t timeout = 100);

    bool writeControlTableItem(uint16_t model_num, 
      uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout = 100);


#if 0 //TODO
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

    uint8_t getHardwareError(uint8_t id);
#endif     

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

    bool setTorqueEnable(uint8_t id, bool enable);
    bool setLedState(uint8_t id, bool state);

    float readForRangeDepandancyFunc(uint8_t func_idx, uint8_t id, uint8_t unit);
    bool writeForRangeDepandancyFunc(uint8_t func_idx, uint8_t id, float value, uint8_t unit);
};




#endif /* DYNAMIXEL_2_ARDUINO_H_ */