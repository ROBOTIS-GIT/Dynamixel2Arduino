#include "master.h"


using namespace DYNAMIXEL;


Master::Master(DXLPortHandler &port, float protocol_ver)
: protocol_ver_idx_(2),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK)
{
  setPort(port);
  setPortProtocolVersion(protocol_ver);
  
  p_packet_buf_ = new uint8_t[DXL_BUF_LENGTH];
  if(p_packet_buf_ != nullptr){
    packet_buf_capacity_ = DXL_BUF_LENGTH;
    is_buf_malloced_ = true;
  }
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;  
}


Master::Master(float protocol_ver)
: protocol_ver_idx_(2),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK)
{
  setPortProtocolVersion(protocol_ver);

  p_packet_buf_ = new uint8_t[DXL_BUF_LENGTH];
  if(p_packet_buf_ != nullptr){
    packet_buf_capacity_ = DXL_BUF_LENGTH;
    is_buf_malloced_ = true;
  }
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;  
}


bool 
Master::setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
{
  if(p_packet_buf_ == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }
  if(packet_buf_capacity_ == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return false;
  }

  if(is_buf_malloced_ == true){
    delete p_packet_buf_;
  }
  p_packet_buf_ = p_buf;
  packet_buf_capacity_ = buf_capacity;

  return true;
}


uint8_t* 
Master::getPacketBuffer() const
{
  return p_packet_buf_;
}


uint16_t 
Master::getPacketBufferCapacity() const
{
  return packet_buf_capacity_;
}


// Refer to http://emanual.robotis.com/#protocol
bool 
Master::setPortProtocolVersion(float version)
{
  uint8_t version_idx;

  if(version == 2.0){
    version_idx = 2;
  }else if(version == 1.0){
    version_idx = 1;
  }else{
    last_lib_err_ = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
    return false;
  }

  return setPortProtocolVersionUsingIndex(version_idx);
}


bool 
Master::setPortProtocolVersionUsingIndex(uint8_t version_idx)
{
  if(version_idx != 2 && version_idx != 1){
    return false;
  }
  protocol_ver_idx_ = version_idx;

  return true;
}


float 
Master::getPortProtocolVersion() const
{
  return (float)protocol_ver_idx_;
}


bool 
Master::setPort(DXLPortHandler *p_port)
{
  if(p_port == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }

  p_port_ = p_port;

  return true;
}


bool 
Master::setPort(DXLPortHandler &port)
{
  p_port_ = &port;

  return true;
}


DXLPortHandler* 
Master::getPort() const
{
  return p_port_;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#ping
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#ping
uint8_t 
Master::ping(uint8_t id, uint8_t *p_recv_buf, uint8_t recv_buf_capacity, uint32_t timeout_ms)
{
  uint8_t ret_id_cnt = 0;
  uint32_t pre_time_ms;
  uint8_t rx_param[3];

  // Parameter exception handling
  if(p_recv_buf == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return 0;
  }else if(recv_buf_capacity == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return 0;
  }

  // Send Ping Instruction
  if(txInstPacket(id, DXL_INST_PING, nullptr, 0) == true){
    // Receive Status Packet
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(rx_param, 3, timeout_ms) != nullptr){
        if(info_rx_packet_.id == id){
          p_recv_buf[ret_id_cnt++] = info_rx_packet_.id;
        }
      }
    }else{
      pre_time_ms = millis();
      while(ret_id_cnt < recv_buf_capacity)
      {
        if(rxStatusPacket(rx_param, 3, 3) != nullptr){
          p_recv_buf[ret_id_cnt++] = info_rx_packet_.id;
        }

        if (millis()-pre_time_ms >= timeout_ms) {
          last_lib_err_ = DXL_LIB_ERROR_TIMEOUT;
          break;
        }
      }
    }
  }

  return ret_id_cnt;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#read
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#read
int32_t 
Master::read(uint8_t id, uint16_t addr, uint16_t addr_length,
 uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout_ms)
{
  int32_t ret_param_len = -1;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;
  uint8_t tx_param[4];
  
  // Parameter exception handling
  if(p_recv_buf == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(addr_length == 0) {
    err = DXL_LIB_ERROR_ADDR_LENGTH;
  }else if(recv_buf_capacity < addr_length){
    err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
  }else if (id == DXL_BROADCAST_ID) {
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return -1;
  }
    
  // Send Read Instruction
  if(protocol_ver_idx_ == 2){
    tx_param[param_len++] = addr >> 0;
    tx_param[param_len++] = addr >> 8;
    tx_param[param_len++] = addr_length >> 0;
    tx_param[param_len++] = addr_length >> 8;    
  }else if(protocol_ver_idx_ == 1){
    tx_param[param_len++] = addr;
    tx_param[param_len++] = addr_length;
  }
  if(txInstPacket(id, DXL_INST_READ, (uint8_t*)&tx_param, param_len) == true){
    if(rxStatusPacket(p_recv_buf, recv_buf_capacity, timeout_ms) != nullptr){
      ret_param_len = (int32_t)info_rx_packet_.recv_param_len;
    }
  }

  return ret_param_len;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#write
bool 
Master::write(uint8_t id, uint16_t addr, 
  const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms)
{
  bool ret = false;
  
  // Send Write Instruction
  if(writeNoResp(id, addr, p_data, data_length) == true){
    // Receive Status Packet
    if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
      ret = true;
    }
  }

  return ret;
}


bool 
Master::writeNoResp(uint8_t id, uint16_t addr, const uint8_t *p_data, uint16_t data_length)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;

  // Parameter exception handling
  if(p_data == nullptr || p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(data_length == 0) {
    err = DXL_LIB_ERROR_INVAILD_DATA_LENGTH;
  }else if(id == DXL_BROADCAST_ID) {
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Write Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 2;
  }else if(protocol_ver_idx_ == 1){
    param_len = 1;
  }
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    DXL_INST_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&addr, param_len);
  param_len = data_length;
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)p_data, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);
    ret = true;
  }

  last_lib_err_ = err;

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#reg-write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#reg-write
bool
Master::regWrite(uint8_t id, uint16_t addr, 
  const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;

  // Parameter exception handling
  if(p_data == nullptr || p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(data_length == 0) {
    err = DXL_LIB_ERROR_INVAILD_DATA_LENGTH;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Write Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 2;
  }else if(protocol_ver_idx_ == 1){
    param_len = 1;
  }
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    DXL_INST_REG_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&addr, param_len);
  param_len = data_length;
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)p_data, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);

    // Receive Status Packet
    if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
      ret = true;
    }
  }

  last_lib_err_ = err;

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#action
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#action
bool
Master::action(uint8_t id, uint32_t timeout_ms)
{
  bool ret = false;

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_ACTION, nullptr, 0) == true){
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        ret = true;
      }
    }else{
      ret = true;
    }
  }

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#factory-reset
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#factory-reset
bool 
Master::factoryReset(uint8_t id, uint8_t option, uint32_t timeout_ms)
{
  bool ret = false;
  uint8_t param_len = 0;

  // Parameter exception handling
  if (id == DXL_BROADCAST_ID){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }
  
  // Send FactoryReset Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 1;
  }
  if(txInstPacket(id, DXL_INST_FACTORY_RESET, (uint8_t*)&option, param_len) == true){
    if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
      ret = true;
    }
  } 

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#reboot
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#reboot
bool 
Master::reboot(uint8_t id, uint32_t timeout_ms)
{ 
  bool ret = false;

  // Parameter exception handling
  if (id == DXL_BROADCAST_ID){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_REBOOT, nullptr, 0) == true){
    if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
      ret = true;
    }
  }

  return ret;
}


// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#clear
bool
Master::clear(uint8_t id, uint8_t option, uint32_t ex_option, uint32_t timeout_ms)
{
  bool ret = false;
  uint8_t tx_param[5];

  // Parameter exception handling
  if(protocol_ver_idx_ != 2){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }else if(id == DXL_BROADCAST_ID){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  // http://emanual.robotis.com/docs/en/dxl/protocol2/#clear
  tx_param[0] = option;
  if(option == 0x01){ 
    tx_param[1] = 0x44;
    tx_param[2] = 0x58;
    tx_param[3] = 0x4C;
    tx_param[4] = 0x22;
  }else{
    memcpy(&tx_param[1], &ex_option, 4);
  }

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_CLEAR, tx_param, 5) == true){
    if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
      ret = true;
    }
  }

  return ret;
}






#if 0 //TODO
// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-read
bool
Master::syncRead(uint8_t *p_param, uint16_t param_len,
  uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout_ms)
{

}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#sync-write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-write
bool
Master::syncWrite(uint8_t *p_param, uint16_t param_len)
{

}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#bulk-read
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#bulk-read
bool
Master::bulkRead(uint8_t *p_param, uint16_t param_len,
  uint8_t *p_recv_buf, uint16_t recv_buf_capacity)
{

}


// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#bulk-write
bool
Master::bulkWrite(uint8_t *p_param, uint16_t param_len)
{

}

#endif


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#error
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#error
uint8_t 
Master::getLastStatusPacketError() const
{
  return info_rx_packet_.err_idx;
}


DXLLibErrorCode_t 
Master::getLastLibErrCode() const
{
  return last_lib_err_;
}


bool 
Master::txInstPacket(uint8_t id, uint8_t inst_idx, uint8_t *p_param, uint16_t param_len)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_len > 0 && p_param == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(inst_idx == DXL_INST_STATUS){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Instruction Packet
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    inst_idx, 0, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, p_param, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);
    ret = true;
  }

  last_lib_err_ = err;

  return ret;
}


const InfoToParseDXLPacket_t* 
Master::rxStatusPacket(uint8_t* p_param_buf, uint16_t param_buf_cap, uint32_t timeout_ms)
{
  InfoToParseDXLPacket_t *p_ret = nullptr;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint32_t pre_time_ms;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_buf_cap > 0 && p_param_buf == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return nullptr;
  }

  // Receive Status Packet
  begin_parse_dxl_packet(&info_rx_packet_, protocol_ver_idx_, p_param_buf, param_buf_cap);
  pre_time_ms = millis();
  while(1) 
  {
    if(p_port_->available() > 0){
      err = parse_dxl_packet(&info_rx_packet_, p_port_->read());
      if(err == DXL_LIB_OK){
        if((protocol_ver_idx_ == 2 && info_rx_packet_.inst_idx == DXL_INST_STATUS)
        || protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0){
            p_ret = &info_rx_packet_;
          }
          break;
        }
      }else if(err != DXL_LIB_PROCEEDING){
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout_ms) {
      err = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }

  last_lib_err_ = err;

  return p_ret;
}