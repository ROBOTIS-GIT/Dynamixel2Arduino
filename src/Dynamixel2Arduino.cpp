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

typedef struct ModelDependencyFuncItemAndRangeInfo{
  uint8_t func_idx;
  uint8_t item_idx;
  uint8_t unit_type;
  int32_t min_value;
  int32_t max_value;
  float unit_value;
} ModelDependencyFuncItemAndRangeInfo_t;

typedef struct ItemAndRangeInfo{
  uint8_t item_idx;
  uint8_t unit_type;
  int32_t min_value;
  int32_t max_value;
  float unit_value;
} ItemAndRangeInfo_t;

static ItemAndRangeInfo_t getModelDependencyFuncInfo(uint16_t model_num, uint8_t func_num);
static float f_map(float x, float in_min, float in_max, float out_min, float out_max);
static bool checkAndconvertWriteData(float in_data, int32_t &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info);
static bool checkAndconvertReadData(int32_t in_data, float &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info);


Dynamixel2Arduino::Dynamixel2Arduino(HardwareSerial& port, int dir_pin)
  : Master(), dxl_port_(port, dir_pin)
{
  setPort(dxl_port_);
  begin();
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

bool Dynamixel2Arduino::setID(uint8_t id, uint8_t new_id)
{
  return writeControlTableItem(ControlTableItem::ID, id, new_id);
}


bool Dynamixel2Arduino::setProtocol(uint8_t id, float version)
{
  uint8_t ver_idx;

  if(version == DXL_PACKET_VER_1_0){
    ver_idx = 1;
  }else if(version == DXL_PACKET_VER_2_0){
    ver_idx = 2;
  }else{
    return false;
  }

  return writeControlTableItem(ControlTableItem::PROTOCOL_VERSION, id, ver_idx);
}

bool Dynamixel2Arduino::setBaudrate(uint8_t id, uint32_t baudrate)
{
  uint16_t model_num = getModelNumberFromTable(id);
  uint8_t baud_idx = 0;

  switch(model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
    case DX113:
    case DX116:
    case DX117:
    case RX10:
    case RX24F:
    case RX28:
    case RX64:
    case EX106:    
    case MX12W:
    case MX28:
    case MX64:
    case MX106:
      // baud_idx = round(2000000.0/(float)baudrate) - 1;
      // if(baud_idx > 254)
      //   return false;
      switch(baudrate)
      {
        case 9600:
          baud_idx = 207;
          break;
        case 57600:
          baud_idx = 34;
          break;
        case 115200:
          baud_idx = 16;
          break;
        case 1000000:
          baud_idx = 1;
          break;
        default:
          return false;                    
      }        
      break;

    case XL320:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        default:
          return false;                    
      }    
      break;

    case MX28_2:
    case MX64_2:
    case MX106_2:
    case XL430_W250:
    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
      switch(baudrate)
      {
        case 9600:
          baud_idx = 0;
          break;
        case 57600:
          baud_idx = 1;
          break;
        case 115200:
          baud_idx = 2;
          break;
        case 1000000:
          baud_idx = 3;
          break;
        case 2000000:
          baud_idx = 4;
          break;
        case 3000000:
          baud_idx = 5;
          break;
        case 4000000:
          baud_idx = 6;
          break;
        case 4500000:
          baud_idx = 7;
          break;    
        default:
          return false;          
      }
      break;            

    default:
      return false;
      break;
  }

  return writeControlTableItem(ControlTableItem::BAUD_RATE, id, baud_idx);
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



bool Dynamixel2Arduino::setOperatingMode(uint8_t id, uint8_t mode)
{
  bool ret = false;
  uint16_t model_num = getModelNumberFromTable(id);

  switch(model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
    case DX113:
    case DX116:
    case DX117:
    case RX10:
    case RX24F:
    case RX28:
    case RX64:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 1023);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }
      break;

    case EX106:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }
      break;

    case XL320:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 2);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 1);
      }
      break;
    
    case MX12W:
    case MX28:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }else if(mode == OP_EXTENDED_POSITION){
        if(writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }
      break;   

    case MX64:
    case MX106:
      if(mode == OP_POSITION){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_VELOCITY){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
      }else if(mode == OP_EXTENDED_POSITION){
        if(writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0)
          || writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
          ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 1);
      }
      break;

    case MX28_2:
    case XL430_W250:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_PWM){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 16);
      }
      break;

    case MX64_2:
    case MX106_2:
    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
      if(mode == OP_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 3);
      }else if(mode == OP_VELOCITY){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 1);
      }else if(mode == OP_EXTENDED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 4);
      }else if(mode == OP_CURRENT){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 0);
      }else if(mode == OP_PWM){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 16);
      }else if(mode == OP_CURRENT_BASED_POSITION){
        ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 5);
      }
      break;            

    default:
      break;
  }

  return ret;
}



bool Dynamixel2Arduino::setGoalPosition(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_DEGREE)
    return false;

  return writeForRangeDependencyFunc(SET_POSITION, id, value, unit);
}

float Dynamixel2Arduino::getPresentPosition(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_DEGREE)
    return 0.0;

  return readForRangeDependencyFunc(GET_POSITION, id, unit);
}

bool Dynamixel2Arduino::setGoalVelocity(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO && unit != UNIT_RPM)
    return false;

  return writeForRangeDependencyFunc(SET_VELOCITY, id, value, unit);
}

float Dynamixel2Arduino::getPresentVelocity(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO && unit != UNIT_RPM)
    return 0.0;

  return readForRangeDependencyFunc(GET_VELOCITY, id, unit);
}

bool Dynamixel2Arduino::setGoalPWM(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO)
    return false;

  return writeForRangeDependencyFunc(SET_PWM, id, value, unit);
}

float Dynamixel2Arduino::getPresentPWM(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO)
    return 0.0;

  return readForRangeDependencyFunc(GET_PWM, id, unit);
}

bool Dynamixel2Arduino::setGoalCurrent(uint8_t id, float value, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO && unit != UNIT_MILLI_AMPERE)
    return false;

  return writeForRangeDependencyFunc(SET_CURRENT, id, value, unit);
}

float Dynamixel2Arduino::getPresentCurrent(uint8_t id, uint8_t unit)
{
  if(unit != UNIT_RAW && unit != UNIT_RATIO && unit != UNIT_MILLI_AMPERE)
    return 0.0;

  return readForRangeDependencyFunc(GET_CURRENT, id, unit);
}



int32_t Dynamixel2Arduino::readControlTableItem(uint8_t item_idx, uint8_t id, uint32_t timeout)
{
  uint16_t model_num = getModelNumberFromTable(id);

  return readControlTableItem(model_num, item_idx, id, timeout);
}

bool Dynamixel2Arduino::writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  uint16_t model_num = getModelNumberFromTable(id);

  return writeControlTableItem(model_num, item_idx, id, data, timeout);
}




/* Private Member Function */

int32_t Dynamixel2Arduino::readControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, uint32_t timeout)
{
  int32_t recv_len, ret = 0;
  ControlTableItemInfo_t item_info;

  item_info = getControlTableItemInfo(model_num, item_idx);

  if(item_info.addr_length == 0)
    return 0;

  recv_len = read(id, item_info.addr, item_info.addr_length, (uint8_t*)&ret, sizeof(ret), timeout);

  if(recv_len == 1){
    int8_t t_data = (int8_t)ret;
    ret = (int32_t)t_data;
  }else if(recv_len == 2){
    int16_t t_data = (int16_t)ret;
    ret = (int32_t)t_data;
  }

  return ret;
}

bool Dynamixel2Arduino::writeControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  ControlTableItemInfo_t item_info;

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

float Dynamixel2Arduino::readForRangeDependencyFunc(uint8_t func_idx, uint8_t id, uint8_t unit)
{
  float ret = 0;
  int32_t ret_data = 0;
  uint16_t model_num = getModelNumberFromTable(id);
  ItemAndRangeInfo_t item_info = getModelDependencyFuncInfo(model_num, func_idx);

  if(item_info.item_idx == LAST_DUMMY_ITEM)
    return false;

  ret_data = readControlTableItem(model_num, item_info.item_idx, id);
  checkAndconvertReadData(ret_data, ret, unit, item_info);

  return ret;
}

bool Dynamixel2Arduino::writeForRangeDependencyFunc(uint8_t func_idx, uint8_t id, float value, uint8_t unit)
{
  bool ret = false;
  int32_t data = 0;
  uint16_t model_num = getModelNumberFromTable(id);
  ItemAndRangeInfo_t item_info = getModelDependencyFuncInfo(model_num, func_idx);

  if(item_info.item_idx == LAST_DUMMY_ITEM)
    return false;

  if(checkAndconvertWriteData(value, data, unit, item_info) == false)
    return false;

  ret = writeControlTableItem(model_num, item_info.item_idx, id, data);

  return ret;
}





/* Const structure & Static Function */

const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_1_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_AX \
 || ENABLE_ACTUATOR_DX \
 || ENABLE_ACTUATOR_RX \
 || ENABLE_ACTUATOR_EX \
 || ENABLE_ACTUATOR_XL320)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 1023, 0.29},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 1023, 0.29},

  {SET_VELOCITY, MOVING_SPEED, UNIT_RATIO, 0, 2047, 0.1},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RATIO, 0, 2047, 0.1},  
#endif 
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_ex[] PROGMEM = {
#if (ENABLE_ACTUATOR_EX)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 4095, 0.06},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 4095, 0.06},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_1_1_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W \
 || ENABLE_ACTUATOR_MX28 \
 || ENABLE_ACTUATOR_MX64 \
 || ENABLE_ACTUATOR_MX106)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -28672, 28672, 0.088},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -28672, 28672, 0.088},

  {SET_VELOCITY, MOVING_SPEED, UNIT_RPM, 0, 2047, 0.114},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.114},  
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_mx12[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W)
  {SET_VELOCITY, MOVING_SPEED, UNIT_RPM, 0, 2047, 0.916},
  {GET_VELOCITY, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.916},  
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_mx64_mx106[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX64 \
 || ENABLE_ACTUATOR_MX106)
  {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, 0, 2047, 4.5},
  {GET_CURRENT, CURRENT, UNIT_MILLI_AMPERE, 0, 4095, 4.5},  
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_2_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX28_PROTOCOL2 \
  || ENABLE_ACTUATOR_MX64_PROTOCOL2 \
  || ENABLE_ACTUATOR_MX106_PROTOCOL2 \
  || ENABLE_ACTUATOR_XL430 \
  || ENABLE_ACTUATOR_XM430 || ENABLE_ACTUATOR_XH430 \
  || ENABLE_ACTUATOR_XM540 || ENABLE_ACTUATOR_XH540)
  {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -1048575, 1048575, 0.088},
  {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483648 , 2147483647, 0.088},

  {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
  {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},  

  {SET_PWM, GOAL_PWM, UNIT_RAW, -885, 885, 1},
  {GET_PWM, PRESENT_PWM, UNIT_RAW, -885, 885, 1},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_mx64_2[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX64_PROTOCOL2)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 3.36},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 3.36},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_mx106_2[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX106_PROTOCOL2)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 3.36},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 3.36},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_xm430_w210_w350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XM430)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 2.69},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 2.69},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_xh430_w210_w350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XH430)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -648, 648, 2.69},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -648, 648, 2.69},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_xh430_v210_v350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XH430)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -689, 689, 1.34},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -689, 689, 1.34},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

const ModelDependencyFuncItemAndRangeInfo_t dependency_xm540_xh540[] PROGMEM = {
#if (ENABLE_ACTUATOR_XM540 || ENABLE_ACTUATOR_XH540)
  {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},
  {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},
#endif
  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};


const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_pro_model[] PROGMEM = {
#if (0) //PRO
  {SET_ID, ID, UNIT_RAW, 0, 253, 1},
  {SET_BAUD_RATE, BAUD_RATE, UNIT_RAW, 0, 3, 1},

  {LAST_DUMMY_FUNC, LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
#endif
};


static ItemAndRangeInfo_t getModelDependencyFuncInfo(uint16_t model_num, uint8_t func_num)
{
  uint8_t func_idx, i = 0;
  const ModelDependencyFuncItemAndRangeInfo_t *p_common_ctable = nullptr;
  const ModelDependencyFuncItemAndRangeInfo_t *p_dep_ctable = nullptr;
  ItemAndRangeInfo_t item_info;
  memset(&item_info, 0, sizeof(item_info));
  item_info.item_idx = LAST_DUMMY_ITEM;

  switch(model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
    case DX113:
    case DX116:
    case DX117:
    case RX10:
    case RX24F:
    case RX28:
    case RX64:
    case XL320:
      p_common_ctable = dependency_ctable_1_0_common;
      break;

    case EX106:
      p_common_ctable = dependency_ctable_1_0_common;
      p_dep_ctable = dependency_ex;
      break;

    case MX12W:
      p_common_ctable = dependency_ctable_1_1_common;
      p_dep_ctable = dependency_mx12;
      break;

    case MX28:
      p_common_ctable = dependency_ctable_1_1_common;
      break;

    case MX64:
    case MX106:
      p_common_ctable = dependency_ctable_1_1_common;
      p_dep_ctable = dependency_mx64_mx106;
      break;              

    case MX28_2:
    case XL430_W250:
      p_common_ctable = dependency_ctable_2_0_common;
      break;
    case MX64_2:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_mx64_2;
      break;    
    case MX106_2:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_mx106_2;
      break;        
    case XM430_W210:
    case XM430_W350:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_xm430_w210_w350;
      break;
    case XH430_V210:
    case XH430_V350:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_xh430_v210_v350;
      break;
    case XH430_W210:
    case XH430_W350:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_xh430_w210_w350;
      break;    
    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
      p_common_ctable = dependency_ctable_2_0_common;
      p_dep_ctable = dependency_xm540_xh540;
      break;            

    default:
      break;
  }

  if(p_common_ctable == nullptr){
    return item_info;
  }

  do{
    func_idx = pgm_read_byte(&p_common_ctable[i].func_idx);
    if(func_idx == func_num) {
      item_info.item_idx = pgm_read_byte(&p_common_ctable[i].item_idx);
      item_info.min_value = (int32_t)pgm_read_dword(&p_common_ctable[i].min_value);
      item_info.max_value = (int32_t)pgm_read_dword(&p_common_ctable[i].max_value);
      item_info.unit_type = pgm_read_byte(&p_common_ctable[i].unit_type);
      item_info.unit_value = (float)pgm_read_dword(&p_common_ctable[i].unit_value);
      break;
    }
    i++;
  }while(func_idx != LAST_DUMMY_FUNC);

  if(p_dep_ctable == nullptr) {
    return item_info;
  }

  i = 0;
  do{
    func_idx = pgm_read_byte(&p_dep_ctable[i].func_idx);
    if(func_idx == func_num) {
      item_info.item_idx = pgm_read_byte(&p_dep_ctable[i].item_idx);
      item_info.min_value = (int32_t)pgm_read_dword(&p_dep_ctable[i].min_value);
      item_info.max_value = (int32_t)pgm_read_dword(&p_dep_ctable[i].max_value);
      item_info.unit_type = pgm_read_byte(&p_dep_ctable[i].unit_type);
      item_info.unit_value = (float)pgm_read_dword(&p_dep_ctable[i].unit_value);
      break;
    }
    i++;
  }while(func_idx != LAST_DUMMY_FUNC);

  return item_info;  
}

static bool checkAndconvertWriteData(float in_data, int32_t &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info)
{
  float data_f = 0.0;
  int32_t data = 0;  

  switch(unit)
  {
  case UNIT_RAW:
    data = (int32_t)in_data;
    if(data < item_info.min_value || data > item_info.max_value)
      return false;
    break;

  case UNIT_RATIO:
    data_f = f_map(in_data, -100.0, 100.0,
     (float)item_info.min_value, (float)item_info.max_value);
    data = (int32_t)data_f;
    break;

  case UNIT_RPM:
  case UNIT_DEGREE:
  case UNIT_MILLI_AMPERE:
    if(unit != item_info.unit_type)
      return false;
    data = round(in_data/item_info.unit_value);
    if(data < item_info.min_value || data > item_info.max_value)
      return false;
    break;

  default:
    return false;
  }

  out_data = data;

  return true;
}

static bool checkAndconvertReadData(int32_t in_data, float &out_data, uint8_t unit, ItemAndRangeInfo_t &item_info)
{
  float data = 0;  

  switch(unit)
  {
  case UNIT_RAW:
    data = (float)in_data;
    break;

  case UNIT_RATIO:
    data = (float)f_map(in_data, 
     (float)item_info.min_value, (float)item_info.max_value, -100.0, 100.0);
    break;    

  case UNIT_RPM:
  case UNIT_DEGREE:
  case UNIT_MILLI_AMPERE:
    if(unit != item_info.unit_type)
      return false;
    data = (float)in_data*item_info.unit_value;
    break;

  default:
    return false;
  }

  out_data = data;

  return true;
}

static float f_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


