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

#include "Dynamixel2Arduino.h"
#include "actuator.h"

using namespace DYNAMIXEL;

typedef struct ModelDependancyFuncInfo{
  uint8_t func_idx;
  uint8_t item_idx;
  uint8_t default_unit;
  int32_t min_value;
  int32_t max_value;
  float unit_value;
} ModelDependancyFuncInfo_t;


Dynamixel2Arduino::Dynamixel2Arduino(HardwareSerial& port, int dir_pin)
  : Master(), dxl_port_(port, dir_pin)
{
  setPort(dxl_port_);
}

/* For Master configuration */
void Dynamixel2Arduino::begin(unsigned long baud)
{
  dxl_port_.begin(baud);
}

unsigned long Dynamixel2Arduino::getPortBaud() const
{
  return dxl_port_.getBaud();
}

bool Dynamixel2Arduino::scan()
{
  bool ret = true;

  ret = ping();

  return ret;
}

bool Dynamixel2Arduino::ping(uint8_t id)
{
  bool ret = false;
  dxl_return_t dxl_ret = DXL_RET_OK;    
  uint8_t i;
  uint8_t id_i;
  uint8_t dxl_cnt;

  if (id == DXL_BROADCAST_ID){
    registered_dxl_cnt_ = 0;

    dxl_ret = Master::ping(DXL_BROADCAST_ID, &resp_.ping, 3*254);  
    dxl_cnt = resp_.ping.id_count;

    for (i=0; i<dxl_cnt && registered_dxl_cnt_<DXLCMD_MAX_NODE; i++){
      id_i = resp_.ping.p_node[i]->id;

      registered_dxl_[registered_dxl_cnt_].id  = id_i;
      if(getPortProtocolVersion() == 2.0){
        registered_dxl_[registered_dxl_cnt_].model_num = resp_.ping.p_node[i]->model_number;
      }else{
        registered_dxl_[registered_dxl_cnt_].model_num = getModelNumber(id_i);
      }
      registered_dxl_cnt_++;
    }

    if (registered_dxl_cnt_ != 0){
      ret = true;
    }
  }else{
    dxl_ret = Master::ping(id, &resp_.ping, 20);

    if (dxl_ret == DXL_RET_RX_RESP && resp_.ping.id_count > 0){        
      if (registered_dxl_cnt_ < DXLCMD_MAX_NODE){
        for (i=0; i<registered_dxl_cnt_; i++){
          if (registered_dxl_[i].id == id){
            ret = true;
            break;
          }  
        }
        if (i == registered_dxl_cnt_){
          registered_dxl_[i].id = id;
          if(getPortProtocolVersion() == 2.0){
            registered_dxl_[i].model_num = resp_.ping.p_node[0]->model_number;
          }else{
            registered_dxl_[i].model_num = getModelNumber(id);
          }
          registered_dxl_cnt_++;      
          ret = true;
        }
      }
      else
      {
        err_code_ = 0xFF; //DXL 노드 최대 수 초과
      }
      
      err_code_ = 0;     
    }
  }

  return ret;  
}

uint16_t Dynamixel2Arduino::getModelNumber(uint8_t id)
{
  uint16_t model_num = 0xFFFF;

  (void) read(id, COMMON_MODEL_NUMBER_ADDR, COMMON_MODEL_NUMBER_ADDR_LENGTH,
   (uint8_t*)&model_num, sizeof(model_num), 20);

  return model_num;
}

/* Commands for Slave */
bool Dynamixel2Arduino::torqueOn(uint8_t id)
{
  return setTorqueEnable(id, true);
}

bool Dynamixel2Arduino::torqueOff(uint8_t id)
{
  return setTorqueEnable(id, false);
}

bool Dynamixel2Arduino::setTorqueEnable(uint8_t id, bool enable)
{
  return writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, enable);
}

bool Dynamixel2Arduino::ledOn(uint8_t id)
{
  return setLedState(id, true);
}

bool Dynamixel2Arduino::ledOff(uint8_t id)
{
  return setLedState(id, false);
}

bool Dynamixel2Arduino::setLedState(uint8_t id, bool state)
{
  return writeControlTableItem(ControlTableItem::LED, id, state);
}

int32_t Dynamixel2Arduino::readControlTableItem(uint8_t item_idx, uint8_t id, uint32_t timeout)
{
  int32_t ret = 0;
  ControlTableItemInfo_t item_info;
  uint16_t model_num = getModelNumberFromTable(id);

  item_info = getControlTableItemInfo(model_num, item_idx);

  if(item_info.addr_length == 0)
    return 0;

  read(id, item_info.addr, item_info.addr_length, (uint8_t*)&ret, sizeof(ret), timeout);

  return ret;
}

bool Dynamixel2Arduino::writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  ControlTableItemInfo_t item_info;
  uint16_t model_num = getModelNumberFromTable(id);

  item_info = getControlTableItemInfo(model_num, item_idx);

  if(item_info.addr_length == 0)
    return false;

  return write(id, item_info.addr, (uint8_t*)&data, item_info.addr_length, timeout);  
}

uint16_t Dynamixel2Arduino::getModelNumberFromTable(uint8_t id)
{
  uint16_t model_num = UNREGISTERED_MODEL;
  uint32_t i;

  for(i = 0; i < DXLCMD_MAX_NODE; i++)
  {
    if(registered_dxl_[i].id == id){
      model_num = registered_dxl_[i].model_num;
      break;
    }
  }

  if(i == DXLCMD_MAX_NODE && registered_dxl_cnt_ < DXLCMD_MAX_NODE){
    if(ping(id) == true){
      for(i = 0; i < DXLCMD_MAX_NODE; i++)
      {
        if(registered_dxl_[i].id == id){
          model_num = registered_dxl_[i].model_num;
          break;
        }
      }
    }
  }

  return model_num;
}





// TODO
#if 0
bool Dynamixel2Arduino::setGoalVelocity(uint8_t id, float value, uint8_t unit)
{
  bool ret = false;
  //float min_value, max_value = 0.0;
  int32_t data;
  uint8_t item_idx = 0;

  switch(unit)
  {
  case UNIT_RAW:
    data = (int32_t)value;
    break;

  default:
    return ret;
  }

  ret = writeControlTableItem(item_idx, id, data);

  return ret;
}

const ModelDependancyFuncInfo_t dependancy_ctable_1_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_AX \
 || ENABLE_ACTUATOR_DX \
 || ENABLE_ACTUATOR_RX \
 || ENABLE_ACTUATOR_EX)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 1023, 0.29},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 1023, 0.29},

  {SET_VELOCITY, MOVING_SPEED, UNIT_RATIO, 0, 2047, 0.1},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RATIO, 0, 2047, 0.1},  

  {OP_MODE_CTRL, CW_ANGLE_LIMIT, UNIT_RAW, 1, 1023, 1},

//  {SET_POSITION_MOVING_SPEED, MOVING_SPEED, UNIT_RPM, 0, 1023, 0.111},
//  {GET_POSITION_MOVING_SPEED, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.111},

  {CW_CCW_COMPLIANCE, CW_COMPLIANCE_MARGIN, UNIT_RAW, 0, 255, 1},
//  {COMPLIANCE_MARGIN_CTRL, CW_COMPLIANCE_MARGIN, UNIT_RAW, 0, 255, 1},
//  {COMPLIANCE_SLOPE_CTRL, CW_COMPLIANCE_SLOPE, UNIT_RAW, 0, 254, 1},

#endif 
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependancyFuncInfo_t dependancy_xl320[] PROGMEM = {
#if (ENABLE_ACTUATOR_XL320)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 1023, 0.29},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 1023, 0.29},

  {SET_VELOCITY, MOVING_SPEED, UNIT_RATIO, 0, 2047, 0.1},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RATIO, 0, 2047, 0.1},  

  {OP_MODE_CTRL, CONTROL_MODE, UNIT_RAW, 1, 2, 1},

  // {SET_POSITION_MOVING_SPEED, MOVING_SPEED, UNIT_RPM, 0, 1023, 0.111},
  // {GET_POSITION_MOVING_SPEED, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.111},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependancyFuncInfo_t dependancy_ex[] PROGMEM = {
#if (ENABLE_ACTUATOR_EX)
  {COMPLIANCE_MARGIN_CTRL, CW_COMPLIANCE_MARGIN, UNIT_RAW, 0, 255, 1},
  {COMPLIANCE_SLOPE_CTRL, CW_COMPLIANCE_SLOPE, UNIT_RAW, 0, 254, 1},
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 4095, 0.06},

#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};






const ModelDependancyFuncInfo_t dependancy_ctable_1_1_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W \
 || ENABLE_ACTUATOR_MX28 \
 || ENABLE_ACTUATOR_MX64 \
 || ENABLE_ACTUATOR_MX106)

#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};



const ModelDependancyFuncInfo_t dependancy_mx_1_0[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W || ENABLE_ACTUATOR_MX28 \
 || ENABLE_ACTUATOR_MX64 || ENABLE_ACTUATOR_MX106)
  {SET_ID, ID, UNIT_RAW, 0, 253, 1},

  // 1:1M, 3:500000, 4:400000, 7:250000, 9:200000, 16:115200, 34:57600, 103:19200, 207:9600
  {SET_BAUD_RATE, BAUD_RATE, UNIT_RAW, 0, 207, 1},

  {OP_MODE_CTRL, CW_ANGLE_LIMIT, UNIT_RAW, 0, 1023, 1},

  {POSITION_PID_GAIN, D_GAIN, UNIT_RAW, 0, 254, 1},

  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 4095, 0.088},


  {SET_EXTENDED_POSITION, GOAL_POSITION, UNIT_DEGREE, -28672, 28672, 0.088},


  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
 #endif
};


const ModelDependancyFuncInfo_t dependancy_ctable_xl320[] PROGMEM = {
#if (ENABLE_ACTUATOR_XL320)
  {SET_ID, ID, UNIT_RAW, 0, 253, 1},

  {SET_BAUD_RATE, BAUD_RATE, UNIT_RAW, 0, 3, 1},

  {OP_MODE_CTRL, CONTROL_MODE, UNIT_RAW, 1, 2, 1},
  

  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
#endif
};

const ModelDependancyFuncInfo_t dependancy_ctable_2_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX28_PROTOCOL2 \
  || ENABLE_ACTUATOR_MX64_PROTOCOL2 \
  || ENABLE_ACTUATOR_MX106_PROTOCOL2 \
  || ENABLE_ACTUATOR_XL430 \
  || ENABLE_ACTUATOR_XM430 || ENABLE_ACTUATOR_XH430 \
  || ENABLE_ACTUATOR_XM540 || ENABLE_ACTUATOR_XH540)

  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 4095, 0.088},
  {SET_EXTENDED_POSITION, GOAL_POSITION, UNIT_DEGREE, -1048575, 1048575, 0.088},
  {SET_CURRENT_BASED_POSITION, GOAL_POSITION, UNIT_DEGREE, -1048575, 1048575, 0.088},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483648, 2147483647, 0.088},

  {SET_VELOCITY, MOVING_SPEED, UNIT_RATIO, 0, 2047, 0.1},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RATIO, 0, 2047, 0.1},  

  // Each has its own dependency.
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, 0, 0, 0},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, 0, 0, 0},  

  // Bit0:Direction
  {DIRECTION_CTRL, DRIVE_MODE, UNIT_RAW, 0, 4, 1},
  // Bit2:Profile
  {PROFILE_MODE_CTRL, DRIVE_MODE, UNIT_RAW, 0, 4, 1},
  // 0:Current, 1:Velocity, 3:Position, 4:Extended Position, 5:Current Based Position, 16:PWM
  {OP_MODE_CTRL, OPERATING_MODE, UNIT_RAW, 0, 16, 1},
  
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
#endif
};




const ModelDependancyFuncInfo_t dependancy_ctable_pro_model[] PROGMEM = {
#if (0) //PRO
  {SET_ID, ID, UNIT_RAW, 0, 253, 1},
  {SET_BAUD_RATE, BAUD_RATE, UNIT_RAW, 0, 3, 1},

  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
#endif
};

#endif