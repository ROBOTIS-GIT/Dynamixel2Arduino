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


Dynamixel2Arduino::Dynamixel2Arduino(HardwareSerial& port, int dir_pin)
  : Master(), dxl_port_(port, dir_pin)
{
  setPort(&dxl_port_);
}

/* For Master configuration */
void Dynamixel2Arduino::begin(unsigned long baud)
{
  (void)baud;
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
  dxl_return_t dxl_ret;    
  uint8_t i;
  uint8_t id_i;
  uint8_t dxl_cnt;

  if (id == DXL_BROADCAST_ID){
    registered_dxl_cnt_ = 0;

    dxl_ret = Master::ping(DXL_BROADCAST_ID, &resp_.ping, 500);  
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

bool Dynamixel2Arduino::ledOn(uint8_t id)
{
  return setLedState(id, true);
}

bool Dynamixel2Arduino::ledOff(uint8_t id)
{
  return setLedState(id, false);
}

bool Dynamixel2Arduino::setTorqueEnable(uint8_t id, bool enable)
{
  return writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, enable);
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

  read(id, item_info.addr, item_info.addr_length, (uint8_t*)&ret, sizeof(ret), timeout);

  return ret;
}

bool Dynamixel2Arduino::writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout)
{
  ControlTableItemInfo_t item_info;
  uint16_t model_num = getModelNumberFromTable(id);

  item_info = getControlTableItemInfo(model_num, item_idx);

  return write(id, item_info.addr, (uint8_t*)&data, item_info.addr_length, timeout);  
}

uint16_t Dynamixel2Arduino::getModelNumberFromTable(uint8_t id)
{
  uint16_t model_num = 0xFFFF;
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