#include "master.h"


using namespace DYNAMIXEL;

Master::Master(PortHandler &port, float protocol_ver)
  : last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  setPort(port);
  dxlInit(&packet_, protocol_ver);
}

Master::Master(float protocol_ver)
  : last_status_packet_error_(0), last_lib_err_code_(DXL_LIB_OK)
{
  dxlInit(&packet_, protocol_ver);
}

bool Master::setPortProtocolVersion(float version)
{
  return dxlSetProtocolVersion(&packet_, version);
}

float Master::getPortProtocolVersion()
{
  return dxlGetProtocolVersion(&packet_);
}

bool Master::setPort(PortHandler &port)
{
  bool ret = setDxlPort(&port);

  p_port_ = &port;

  return ret;
}

bool Master::ping(uint8_t id, status_ping_t *p_resp, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_ms, pre_time_us;

  if(p_resp == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return false;    
  }

  if (p_port_->getOpenState() != true) {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }
  
  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_PING, NULL, 0);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;

  p_resp->id_count = 0;
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK 
        && packet_.rx.type == RX_PACKET_TYPE_STATUS 
        && p_resp->id_count < DXL_MAX_NODE) {
      packet_.rx_time = micros() - pre_time_us;
      pre_time_ms     = millis();

      p_resp->node[p_resp->id_count].id = packet_.rx.id;
      if(getPortProtocolVersion() == DXL_PACKET_VER_2_0) {
        p_resp->node[p_resp->id_count].model_number     = packet_.rx.p_param[0]<<0;
        p_resp->node[p_resp->id_count].model_number    |= packet_.rx.p_param[1]<<8;
        p_resp->node[p_resp->id_count].firmware_version = packet_.rx.p_param[2];
      }

      p_resp->id_count++;

      if (id != DXL_BROADCAST_ID) {
        last_lib_err_code_ = DXL_LIB_OK;
        ret = true;
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout) {
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      if (p_resp->id_count > 0)
        ret = true;
      break;
    }
  }

  return ret;
}

int32_t Master::read(uint8_t id, uint16_t addr, uint16_t addr_length,
 uint8_t *p_recv_buf, uint16_t recv_buf_length, uint32_t timeout)
{
  uint32_t pre_time_us, pre_time_ms;
  int32_t i, recv_param_len = -1;
  uint8_t tx_param[4];

  if (id == DXL_BROADCAST_ID) {
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return -1;
  }

  if(addr_length == 0) {
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return -1;
  }
    
  if (p_port_->getOpenState() != true) {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return -1;
  }
    
  // Send Read Instruction 
  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ) {
    tx_param[0] = addr;
    tx_param[1] = addr_length;
  }else{
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;
    tx_param[2] = addr_length >> 0;
    tx_param[3] = addr_length >> 8;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_READ, tx_param, 4);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();

  // Receive Status Packet  
  while(1) {
    last_lib_err_code_ = dxlRxPacket(&packet_);

    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      recv_param_len = packet_.rx.param_length;
      if(recv_param_len > recv_buf_length) {
        recv_param_len = recv_buf_length;
      }

      for (i=0; i<recv_param_len; i++)
      {
        p_recv_buf[i] = packet_.rx.p_param[i];
      }
      last_status_packet_error_ = packet_.rx.error;

      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout) {
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }

  return recv_param_len;
}

bool Master::write(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t data_length, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_us, pre_time_ms;
  
  if(writeNoResp(id, addr, p_data, data_length) != false){
    return ret;
  }
    
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    } else if (last_lib_err_code_ != DXL_LIB_PROCEEDING) {
      break;
    }

    if (millis()-pre_time_ms >= timeout)
    {
      last_status_packet_error_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }

  return ret;
}

bool Master::writeNoResp(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t data_length)
{
  bool ret = false;
  uint32_t i, pre_time_us;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if(p_data == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return false;    
  }

  if (id == DXL_BROADCAST_ID) {
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if(data_length == 0) {
    last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return false;
  }

  if (p_port_->getOpenState() != true)
  {
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return ret;
  }  

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 )
  {
    if ((size_t)(PKT_1_0_INST_PARAM_IDX + 1 + data_length + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = addr;
    for (i=0; i<data_length; i++)
    {
      p_tx_data[tx_length++] = p_data[i];
    }
  }
  else
  {
    if ((size_t)(PKT_INST_PARAM_IDX + 2 + data_length + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    p_tx_data[tx_length++] = addr >> 0;
    p_tx_data[tx_length++] = addr >> 8;
    for (i=0; i<data_length; i++)
    {
      p_tx_data[tx_length++] = p_data[i];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_WRITE, p_tx_data, tx_length);
  packet_.tx_time = micros() - pre_time_us;

  return ret;
}

bool Master::factoryReset(uint8_t id, uint8_t option, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];

  if (id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }
  
  tx_param[0] = option;

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_RESET, tx_param, 1);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }
  return ret;
}

bool Master::reboot(uint8_t id, uint32_t timeout)
{ 
  bool ret = false;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];

  if (id == DXL_BROADCAST_ID){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, id, INST_REBOOT, tx_param, 1);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
      last_status_packet_error_ = packet_.rx.error;
      ret = true;
      break;
    }else if (last_lib_err_code_ != DXL_LIB_PROCEEDING){
      break;
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      break;
    }
  }
  return ret;
}



int32_t Master::syncRead(uint16_t addr, uint16_t addr_len,
  uint8_t *id_list, uint8_t id_cnt, 
  uint8_t *recv_buf, uint16_t recv_buf_size, uint32_t timeout)
{
  uint8_t i, id_idx = 0;
  int32_t recv_len = 0;  
  uint32_t pre_time_us, pre_time_ms;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if(id_list == nullptr || recv_buf == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return -1;
  }
    
  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return -1;
  }

  if(id_cnt*addr_len > recv_buf_size
     || (size_t)(PKT_INST_PARAM_IDX + 4 + id_cnt + 2) > sizeof(packet_.tx.data)){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return -1;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return -1;
  }

  p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
  p_tx_data[tx_length++] = addr >> 0;
  p_tx_data[tx_length++] = addr >> 8;
  p_tx_data[tx_length++] = addr_len >> 0;
  p_tx_data[tx_length++] = addr_len >> 8;

  for( i=0; i<id_cnt; i++)
  {
    p_tx_data[tx_length++] = id_list[i];
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_READ, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;    

  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;

      while(id_idx < id_cnt && id_list[id_idx] < packet_.rx.id){
        for (i=0; i<addr_len; i++)
        {
          recv_buf[id_idx*addr_len + i] = 0;
        }
        id_idx++;
      }

      if(id_list[id_idx] == packet_.rx.id){
        for (i=0; i<addr_len; i++)
        {
          recv_buf[id_idx*addr_len + i] = packet_.rx.p_param[i];
        }
        id_idx++;
      }

      recv_len += packet_.rx.param_length;

      if(id_idx >= id_cnt){
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      return -1;
    }
  }

  return recv_len;
}


bool Master::syncWrite(uint16_t addr, uint16_t addr_len,
  uint8_t *id_list, uint8_t id_cnt, 
  uint8_t *data_list, uint16_t data_list_size)
{
  bool ret = false;
  uint32_t pre_time_us, i, j = 0;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;
  
  if(id_list == nullptr || data_list == nullptr){
    last_lib_err_code_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }

  if (addr_len*id_cnt > data_list_size){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  if (getPortProtocolVersion() == DXL_PACKET_VER_1_0 ){
    if(addr > 0xFF){
      last_lib_err_code_ = DXL_LIB_ERROR_INVAILD_ADDR;
      return false;
    }
    if(addr_len > 0xFF){
      last_lib_err_code_ = DXL_LIB_ERROR_ADDR_LENGTH;
      return false;
    }
    if ((size_t)(PKT_1_0_INST_PARAM_IDX + 2 + addr_len*id_cnt + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = (uint8_t)addr;
    p_tx_data[tx_length++] = (uint8_t)addr_len;
  }else{
    if ((size_t)(PKT_INST_PARAM_IDX + 4 + addr_len*id_cnt + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    p_tx_data[tx_length++] = addr >> 0;
    p_tx_data[tx_length++] = addr >> 8;
    p_tx_data[tx_length++] = addr_len >> 0;
    p_tx_data[tx_length++] = addr_len >> 8;
  }

  for(i=0; i<id_cnt; i++)
  {
    p_tx_data[tx_length++] = id_list[i];
    for(j=0; j<addr_len; j++)
    {
      p_tx_data[tx_length++] = data_list[i*addr_len + j];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;
  ret = true;

  return ret;
}



#if 1
bool Master::syncRead(param_sync_read_t *p_param, status_read_t *p_resp, uint32_t timeout)
{
  bool ret = false;
  uint32_t i, pre_time_us, pre_time_ms;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }

  if(p_param->id_count > DXL_MAX_NODE
     || (size_t)(PKT_INST_PARAM_IDX + p_param->id_count * 5 + 2) > sizeof(packet_.tx.data)){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];

  p_tx_data[tx_length++] = p_param->addr >> 0;
  p_tx_data[tx_length++] = p_param->addr >> 8;
  p_tx_data[tx_length++] = p_param->length >> 0;
  p_tx_data[tx_length++] = p_param->length >> 8;

  for( i=0; i<p_param->id_count; i++)
  {
    p_tx_data[tx_length++] = p_param->id_tbl[i];
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_READ, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;

  p_resp->id_count = 0;
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;
    
      p_resp->node[p_resp->id_count].id     = packet_.rx.id;
      p_resp->node[p_resp->id_count].error  = packet_.rx.error;
      p_resp->node[p_resp->id_count].length = packet_.rx.param_length;

      for (i=0; i<packet_.rx.param_length; i++)
      {
        p_resp->node[p_resp->id_count].data[i] = packet_.rx.p_param[i];
      }

      p_resp->id_count++;

      if (p_resp->id_count >= p_param->id_count){
        ret = true;
        break;
      }
    }

    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      return false;
    }
  }
  
  return ret;
}


bool Master::syncWrite(param_sync_write_t *p_param)
{
  bool ret = false;
  uint32_t i, j, pre_time_us;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    if(p_param->id_count > DXL_MAX_NODE
       || (size_t)(PKT_1_0_INST_PARAM_IDX + 2 + p_param->length + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = p_param->addr;
    p_tx_data[tx_length++] = p_param->length;
  }else{
    if(p_param->id_count > DXL_MAX_NODE
       || (size_t)(PKT_INST_PARAM_IDX + 4 + p_param->length + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    p_tx_data[tx_length++] = p_param->addr >> 0;
    p_tx_data[tx_length++] = p_param->addr >> 8;
    p_tx_data[tx_length++] = p_param->length >> 0;
    p_tx_data[tx_length++] = p_param->length >> 8;
  }

  for( i=0; i<p_param->id_count; i++)
  {
    p_tx_data[tx_length++] = p_param->node[i].id;
    for (j=0; j<p_param->length; j++)
    {
      p_tx_data[tx_length++] = p_param->node[i].data[j];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;
  ret = true;

  return ret;
}

bool Master::bulkRead(param_bulk_read_t *p_param, status_read_t *p_resp, uint32_t timeout)
{
  bool ret = false;
  uint32_t i, pre_time_us, pre_time_ms;
  uint16_t tx_length = 0;
  uint8_t *p_tx_data;

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  if(getPortProtocolVersion() == DXL_PACKET_VER_1_0){
    if(p_param->id_count > DXL_MAX_NODE
       || (size_t)(PKT_1_0_INST_PARAM_IDX + 1 + p_param->id_count * 3 + 1) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_1_0_INST_PARAM_IDX];
    p_tx_data[tx_length++] = 0x00;
    for( i=0; i<p_param->id_count; i++)
    {
      p_tx_data[tx_length++] = p_param->length[i];
      p_tx_data[tx_length++] = p_param->id_tbl[i];
      p_tx_data[tx_length++] = p_param->addr[i];
    }
  }else{
    if(p_param->id_count > DXL_MAX_NODE
       || (size_t)(PKT_INST_PARAM_IDX + p_param->id_count * 5 + 2) > sizeof(packet_.tx.data)){
      last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      return false;
    }
    p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
    for( i=0; i<p_param->id_count; i++)
    {
      p_tx_data[tx_length++] = p_param->id_tbl[i];
      p_tx_data[tx_length++] = p_param->addr[i] >> 0;
      p_tx_data[tx_length++] = p_param->addr[i] >> 8;
      p_tx_data[tx_length++] = p_param->length[i] >> 0;
      p_tx_data[tx_length++] = p_param->length[i] >> 8;
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_READ, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;

  p_resp->id_count = 0;
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_lib_err_code_ = dxlRxPacket(&packet_);
    if (last_lib_err_code_ == DXL_LIB_OK && packet_.rx.type == RX_PACKET_TYPE_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;

      p_resp->node[p_resp->id_count].id     = packet_.rx.id;
      p_resp->node[p_resp->id_count].error  = packet_.rx.error;
      p_resp->node[p_resp->id_count].length = packet_.rx.param_length;

      for (i=0; i<packet_.rx.param_length; i++)
      {
        p_resp->node[p_resp->id_count].data[i] = packet_.rx.p_param[i];
      }

      p_resp->id_count++;

      if (p_resp->id_count >= p_param->id_count){
        ret = true;
        break;
      }
    }
      
    if (millis()-pre_time_ms >= timeout){
      last_lib_err_code_ = DXL_LIB_ERROR_TIMEOUT;
      return false;
    }
  } 
  return ret;
}

bool Master::bulkWrite(param_bulk_write_t *p_param)
{
  bool ret = false;
  uint32_t i, j, pre_time_us;
  uint16_t tx_length = 0, total_data_length = 0;
  uint8_t *p_tx_data;

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 ){
    last_lib_err_code_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }

  for( i=0; i<p_param->id_count; i++)
  {
    total_data_length += p_param->node[i].length;
  }

  if(p_param->id_count > DXL_MAX_NODE
     || (size_t)(PKT_INST_PARAM_IDX + p_param->id_count * 5 + total_data_length +2) > sizeof(packet_.tx.data)){
    last_lib_err_code_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return false;
  }

  if (p_port_->getOpenState() != true){
    last_lib_err_code_ = DXL_LIB_ERROR_PORT_NOT_OPEN;
    return false;
  }

  p_tx_data = &packet_.tx.data[PKT_INST_PARAM_IDX];
  for( i=0; i<p_param->id_count; i++)
  {
    p_tx_data[tx_length++] = p_param->node[i].id;
    p_tx_data[tx_length++] = p_param->node[i].addr >> 0;
    p_tx_data[tx_length++] = p_param->node[i].addr >> 8;
    p_tx_data[tx_length++] = p_param->node[i].length >> 0;
    p_tx_data[tx_length++] = p_param->node[i].length >> 8;
    for (j=0; j<p_param->node[i].length; j++)
    {
      p_tx_data[tx_length++] = p_param->node[i].data[j];
    }
  }

  pre_time_us = micros();
  last_lib_err_code_ = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_WRITE, p_tx_data, tx_length);
  if(last_lib_err_code_ != DXL_LIB_OK)
    return false;
  packet_.tx_time = micros() - pre_time_us;  
  ret = true;

  return ret;
}

#endif


uint8_t Master::getLastStatusPacketError() const
{
  return last_status_packet_error_;
}

lib_err_code_t Master::getLastLibErrCode() const
{
  return last_lib_err_code_;
}

