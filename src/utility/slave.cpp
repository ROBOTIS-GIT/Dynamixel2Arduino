#include "config.h"
#include "slave.h"

using namespace DYNAMIXEL;


enum DefaultControlTableItemAddr{
  ADDR_MODEL_NUMBER    = 0,
  ADDR_FIRMWARE_VER    = 6,
  ADDR_ID              = 7,
  ADDR_PROTOCOL_VER    = 9
};

static bool isAddrInRange(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length);
static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length, uint16_t other_start_addr, uint16_t other_length);

static lib_err_code_t dxlMakePacketStatus1_0(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlMakePacketStatus2_0(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlTxPacketStatus(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length);
static lib_err_code_t dxlMakePacketStatus(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length );


Slave::Slave(PortHandler &port, const uint16_t model_num, float protocol_ver)
: model_num_(model_num), 
  firmware_ver_(1), id_(1),
  user_write_callback_(nullptr), user_read_callback_(nullptr),
  last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  setPort(port);
  protocol_ver = protocol_ver == 1.0 ? 1.0:2.0;
  protocol_ver_idx_ = protocol_ver == 1.0 ? 1:2;
  dxlInit(&packet_, protocol_ver, DXLMode::DXL_MODE_SLAVE);
  this->setID(id_);
  memset(&control_table_, 0, sizeof(control_table_));
  this->addDefaultControlItem();
}

Slave::Slave(const uint16_t model_num, float protocol_ver)
: model_num_(model_num), 
  firmware_ver_(1), id_(1),
  user_write_callback_(nullptr), user_read_callback_(nullptr),
  last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  protocol_ver = protocol_ver == 1.0 ? 1.0:2.0;
  protocol_ver_idx_ = protocol_ver == 1.0 ? 1:2;
  dxlInit(&packet_, protocol_ver, DXLMode::DXL_MODE_SLAVE);
  this->setID(id_);
  memset(&control_table_, 0, sizeof(control_table_));
  this->addDefaultControlItem();
}

void Slave::setWriteCallbackFunc(userCallbackFunc callback_func, void* callback_arg)
{
  user_write_callback_ = callback_func;
  user_write_callbakc_arg_ = callback_arg;
}

void Slave::setReadCallbackFunc(userCallbackFunc callback_func, void* callback_arg)
{
  user_read_callback_ = callback_func;
  user_read_callbakc_arg_ = callback_arg;
}

bool Slave::setPort(PortHandler &port)
{
  bool ret = setDxlPort(&packet_, &port);

  p_port_ = &port;

  return ret;
}

bool Slave::setPortProtocolVersion(float version)
{
  bool ret = false;

  ret = dxlSetProtocolVersion(&packet_, version);
  if(ret == true){
    protocol_ver_idx_ = version == 1.0 ? 1:2;
  }

  return ret;
}

bool Slave::setPortProtocolVersionUsingIndex(uint8_t version_idx)
{
  bool ret = false;
  float version_float;

  if(version_idx == 1){
    version_float = 1.0;
  }else if(version_idx == 2){
    version_float = 2.0;
  }else{
    return false;
  }

  ret = dxlSetProtocolVersion(&packet_, version_float);
  if(ret == true){
    protocol_ver_idx_ = version_idx;
  }

  return ret;
}

float Slave::getPortProtocolVersion() const
{
  float version_float = 0.0;

  if(protocol_ver_idx_ == 1){
    version_float = 1.0;
  }else if(protocol_ver_idx_ == 2){
    version_float = 2.0;
  }

  return version_float;
}

uint8_t Slave::getPortProtocolVersionIndex() const
{
  return protocol_ver_idx_;
}

uint16_t Slave::getModelNumber() const
{
  return model_num_;
}

bool Slave::setID(uint8_t id)
{
  if(getPortProtocolVersion() == 1.0){
    if(id > 253){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ID;
      return false;
    }
  }else{
    if(id > 252){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ID;
      return false;
    }
  }

  id_ = id;
  dxlSetId(&packet_, id);

  return true;
}

uint8_t Slave::getID() const
{
  return id_;
}

void Slave::setFirmwareVersion(uint8_t version)
{
  firmware_ver_ = version;
}

uint8_t Slave::getFirmwareVersion() const
{
  return firmware_ver_;
}

bool Slave::processPacket()
{
  bool ret = false;
  last_lib_err_code_ = dxlRxPacket(&packet_);

  if(last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_INST){
    if(packet_.rx.id == packet_.id){
      ret = processInst(packet_.rx.cmd);
    }
  }else if(last_lib_err_code_ == DXL_LIB_PROCEEDING){
    ret = true;
  }

  return ret;
}


uint8_t Slave::getNumCanBeRegistered() const
{
  return CONTROL_ITEM_MAX-registered_item_cnt_;
}

bool Slave::isEnoughSpaceInControlTable(uint16_t start_addr, uint16_t length)
{
  uint16_t available_start_addr = control_table_[registered_item_cnt_].start_addr + control_table_[registered_item_cnt_].length;

  if(start_addr > CONTROL_ITEM_ADDR_LIMIT){
    last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ADDR;
    return false;
  }

  if(length == 0 || length > CONTROL_ITEM_ADDR_LIMIT - available_start_addr){
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return false;
  }

  return true;
}


uint8_t Slave::addControlItem(uint16_t start_addr, uint8_t* p_data, uint16_t length)
{
  if(registered_item_cnt_ >= CONTROL_ITEM_MAX){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return last_lib_err_code_;
  }

  if(p_data == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return last_lib_err_code_;
  }

  if(isEnoughSpaceInControlTable(start_addr, length) == false){
    return last_lib_err_code_;
  }

  for(uint16_t i=0; i < registered_item_cnt_; i++){
    if(isAddrInOtherItem(start_addr, length, control_table_[i].start_addr, control_table_[i].length)){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ADDR;
      return last_lib_err_code_;
    }
  }

  control_table_[registered_item_cnt_].start_addr = start_addr;
  control_table_[registered_item_cnt_].length = length;
  control_table_[registered_item_cnt_].p_data = p_data;

  registered_item_cnt_++;

  last_lib_err_code_ = DXL_LIB_OK;
  
  return last_lib_err_code_;
}

uint8_t Slave::addControlItem(uint16_t start_addr, bool &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, float &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, double &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}




uint8_t Slave::getLastStatusPacketError() const
{
  return last_status_packet_error_;
}

lib_err_code_t Slave::getLastLibErrCode() const
{
  return last_lib_err_code_;
}




bool Slave::processInstPing()
{
  bool ret = false;
  uint16_t param_length = 0;
  uint8_t *p_param_data;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  p_param_data = &packet_.tx.data[PKT_STATUS_PARAM_IDX];
  if(packet_.packet_ver == DXL_PACKET_VER_2_0){
    p_param_data[param_length++] = (uint8_t)(model_num_ >> 0);
    p_param_data[param_length++] = (uint8_t)(model_num_ >> 8);
    p_param_data[param_length++] = (uint8_t)firmware_ver_;
  }

  last_lib_err_code_ = dxlTxPacketStatus(packet_, 0, p_param_data, param_length);

  if(last_lib_err_code_ == DXL_LIB_OK)
    ret = true;

  return ret;
}


bool Slave::processInstRead()
{
  bool ret = false;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t process_ret = DXL_ERR_NONE;
  uint8_t *p_param_data;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if(packet_.packet_ver == DXL_PACKET_VER_1_0 )
  {
    if( packet_.rx.param_length != 2){
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }

    addr   = packet_.rx.p_param[0];
    length = packet_.rx.p_param[1];

    if( length > 0xFF - 2){
      dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }
  }else{
    if( packet_.rx.param_length != 4){
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    } 

    addr   = ((uint16_t)packet_.rx.p_param[1]<<8) | (uint16_t)packet_.rx.p_param[0];
    length = ((uint16_t)packet_.rx.p_param[3]<<8) | (uint16_t)packet_.rx.p_param[2];
  }

  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  memset(packet_.tx.data, 0, sizeof(packet_.tx.data));
  p_param_data = &packet_.tx.data[PKT_STATUS_PARAM_IDX];

  uint16_t item_start_addr, item_addr_length;
  for(uint8_t i=0; i < registered_item_cnt_; i++){
    item_start_addr = control_table_[i].start_addr;
    item_addr_length = control_table_[i].length;
    if(item_addr_length != 0
      && control_table_[i].p_data != nullptr
      && isAddrInRange(item_start_addr, item_addr_length, addr, length)==true){
      
      if(item_start_addr == ADDR_ID){
        setID(dxlGetId(&packet_));
      }else if(item_start_addr == ADDR_PROTOCOL_VER){         
        setPortProtocolVersion(dxlGetProtocolVersion(&packet_));
      }  
      
      if(user_read_callback_ != nullptr){
        user_read_callback_(item_start_addr, process_ret, user_read_callbakc_arg_);
        if(process_ret != DXL_ERR_NONE){
          break;
        }
      }
      memcpy(&p_param_data[item_start_addr-addr], control_table_[i].p_data, item_addr_length);
    }
  }

  last_lib_err_code_ = dxlTxPacketStatus(packet_, process_ret, p_param_data, length);
  if(last_lib_err_code_ == DXL_LIB_OK){
    ret = true;
  }

  return ret;
}


bool Slave::processInstWrite()
{
  bool ret = false;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t  *p_data;
  uint8_t process_ret = DXL_ERR_NONE;

  if(packet_.rx.id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }
    
  if(packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    addr   =  packet_.rx.p_param[0];
    p_data = &packet_.rx.p_param[1];

    if(packet_.rx.param_length > 1 ){
      length = packet_.rx.param_length - 1;
    }else{
      dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }

    if( length > 0xFF - 2 ){
      dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }
  }else{
    addr   = ((uint16_t)packet_.rx.p_param[1]<<8) | (uint16_t)packet_.rx.p_param[0];
    p_data = &packet_.rx.p_param[2];

    if(packet_.rx.param_length > 2 ){
      length = packet_.rx.param_length - 2;
    }else{
      dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
      last_lib_err_code_ = DXL_LIB_ERROR_LENGTH;
      return false;
    }    
  }

  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(packet_, DXL_ERR_DATA_LENGTH, nullptr, 0);
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  uint16_t item_start_addr, item_addr_length;
  for(uint8_t i=0; i < registered_item_cnt_; i++){
    item_start_addr = control_table_[i].start_addr;
    item_addr_length = control_table_[i].length;
    if(item_addr_length != 0
      && control_table_[i].p_data != nullptr
      && isAddrInRange(item_start_addr, item_addr_length, addr, length)==true){
      if(item_start_addr != ADDR_MODEL_NUMBER
        && item_start_addr != ADDR_FIRMWARE_VER){

        memcpy(control_table_[i].p_data, &p_data[item_start_addr-addr], item_addr_length);
        if(user_write_callback_ != nullptr){
          user_write_callback_(item_start_addr, process_ret, user_write_callbakc_arg_);
          if(process_ret != DXL_ERR_NONE){
            break;
          }
        }
      }
    }
  }

  last_lib_err_code_ = dxlTxPacketStatus(packet_, process_ret, nullptr, 0);
  if(last_lib_err_code_ == DXL_LIB_OK){
    ret = true;
  }

  return ret;
}

bool Slave::addDefaultControlItem()
{
  if(addControlItem(ADDR_MODEL_NUMBER, (uint16_t&)model_num_) != DXL_LIB_OK
    || addControlItem(ADDR_FIRMWARE_VER, firmware_ver_) != DXL_LIB_OK
    || addControlItem(ADDR_ID, id_) != DXL_LIB_OK
    || addControlItem(ADDR_PROTOCOL_VER, protocol_ver_idx_) != DXL_LIB_OK){
    return false;
  }   

  return true;
}

bool Slave::processInst(uint8_t inst_idx)
{
  bool ret = false;

  switch(inst_idx)
  {
    case INST_PING:
      ret = processInstPing();
      break;

    case INST_READ:
      ret = processInstRead();
      break;

    case INST_WRITE:
      ret = processInstWrite();
      break;

    default:
      last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
      break;  
  }

  return ret;
}







static bool isAddrInRange(uint16_t addr, uint16_t length,
  uint16_t range_addr, uint16_t range_length)
{
  return (addr >= range_addr && addr+length <= range_addr+range_length) ? true:false;
}

static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length,
  uint16_t other_start_addr, uint16_t other_length)
{
  bool ret = false;
  uint16_t addr_end = start_addr + length;
  uint16_t other_addr_end = other_start_addr + other_length;

  // The start address of the item is in the range of another item address.
  if (start_addr >= other_start_addr && start_addr < other_addr_end){ 
    ret = true;
  // The last address of an item is in the range of another item address.    
  }else if (addr_end > other_start_addr && addr_end <= other_addr_end){ 
    ret = true;
  }  

  return ret;
}




static lib_err_code_t dxlTxPacketStatus(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  ret = dxlMakePacketStatus(packet, error, p_data, length);
  if(ret == DXL_LIB_OK) {
    dxlTxWrite(&packet, packet.tx.data, packet.tx.packet_length);
  }

  return ret;
}

static lib_err_code_t dxlMakePacketStatus(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  if(packet.packet_ver == DXL_PACKET_VER_1_0){
    ret = dxlMakePacketStatus1_0(packet, error, p_data, length);
  }else{
    ret = dxlMakePacketStatus2_0(packet, error, p_data, length);
  }

  return ret;
}

static lib_err_code_t dxlMakePacketStatus1_0(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint8_t  check_sum;


  if(length > DXL_BUF_LENGTH){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  if(length > 0xFF){
    return DXL_LIB_ERROR_LENGTH;
  }

  check_sum = 0;
  packet_length = length + 2; // param_length + Instruction + CheckSum

  packet.tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  packet.tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  packet.tx.data[PKT_1_0_ID_IDX]    = packet.id;
  packet.tx.data[PKT_1_0_ERROR_IDX] = error;

  check_sum += packet.id;
  check_sum += packet_length;
  check_sum += error;

  for (i=0; i<length; i++)
  {
    packet.tx.data[PKT_1_0_STATUS_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  packet.tx.data[PKT_1_0_LEN_IDX] = packet_length;
  packet.tx.data[PKT_1_0_ERROR_IDX + packet_length - 1] = ~(check_sum);

  return ret;
}

static lib_err_code_t dxlMakePacketStatus2_0(dxl_t &packet, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint16_t stuff_length;
  uint16_t crc;

  if(length > DXL_BUF_LENGTH){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  packet_length = length + 4; // param_length + Instruction + Error + CRC_L + CRC_H

  packet.tx.data[PKT_HDR_1_IDX] = 0xFF;
  packet.tx.data[PKT_HDR_2_IDX] = 0xFF;
  packet.tx.data[PKT_HDR_3_IDX] = 0xFD;
  packet.tx.data[PKT_RSV_IDX]   = 0x00;
  packet.tx.data[PKT_ID_IDX]    = packet.id;
  packet.tx.data[PKT_INST_IDX]  = INST_STATUS;
  packet.tx.data[PKT_ERROR_IDX] = error;

  for (i=0; i<length; i++)
  {
    packet.tx.data[PKT_STATUS_PARAM_IDX + i] = p_data[i];
  }

  stuff_length = dxlAddStuffing(&packet, &packet.tx.data[PKT_INST_IDX], length + 2); // + instruction + error
  packet_length += stuff_length;

  packet.tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  packet.tx.data[PKT_LEN_H_IDX] = packet_length >> 8;

  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, packet.tx.data[i]);
  }

  packet.tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  packet.tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;

  packet.tx.packet_length = packet_length + 7;

  return ret;
}