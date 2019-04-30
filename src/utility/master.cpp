#include "master.h"


using namespace DYNAMIXEL;

Master::Master(PortHandler *port, float protocol_ver)
  : p_port_(port), last_status_error_(0), last_dxl_return_(DXL_RET_OK)
{
  dxlInit(&packet_, protocol_ver);
}

Master::Master(float protocol_ver)
  : p_port_(nullptr), last_status_error_(0), last_dxl_return_(DXL_RET_OK)
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

bool Master::setPort(PortHandler *port)
{
  return setDxlPort(port);
}

dxl_return_t Master::ping(uint8_t id, status_ping_t *p_resp, uint32_t timeout)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_ms;
  uint32_t pre_time_us;
  uint32_t mem_addr;
  uint8_t  *p_mem = (uint8_t *)p_resp->mem;

  p_resp->id_count = 0;
  p_resp->p_node[0] = (ping_node_t *)&p_mem[0];

  if (p_port_->getOpenState() == true) {
    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, id, INST_PING, NULL, 0);
    packet_.tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      ret = dxlRxPacket(&packet_);
      if (ret == DXL_RET_RX_STATUS && p_resp->id_count < DXLCMD_MAX_NODE) {
        packet_.rx_time = micros() - pre_time_us;
        pre_time_ms     = millis();

        p_resp->p_node[p_resp->id_count]->id = packet_.rx.id;
        if(getPortProtocolVersion() == 2.0) {
          p_resp->p_node[p_resp->id_count]->model_number     = packet_.rx.p_param[0]<<0;
          p_resp->p_node[p_resp->id_count]->model_number    |= packet_.rx.p_param[1]<<8;
          p_resp->p_node[p_resp->id_count]->firmware_version = packet_.rx.p_param[2];
        }

        p_resp->id_count++;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(ping_node_t);
        if (mem_addr%4) {
          mem_addr += 4 - (mem_addr%4);
        }

        p_resp->p_node[p_resp->id_count] = (ping_node_t *)&p_mem[mem_addr];

        if (id != DXL_BROADCAST_ID) {
          ret = DXL_RET_RX_RESP;
          break;
        }
      }

      if (millis()-pre_time_ms >= timeout) {
        if (p_resp->id_count > 0) {
          ret = DXL_RET_RX_RESP;
        } else {
          ret = DXL_RET_ERROR_TIMEOUT;
        } 
        break;
      }
    }
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
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
    last_dxl_return_ = DXL_RET_ERROR_NOT_SUPPORT_BROADCAST;
    return recv_param_len;
  }

  if(addr_length == 0) {
    last_dxl_return_ = DXL_RET_ERROR_ADDR_LENGTH;
    return recv_param_len;
  }
    
  if (p_port_->getOpenState() != true) {
    last_dxl_return_ = DXL_RET_NOT_OPEN;
    return recv_param_len;
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
  last_dxl_return_ = dxlTxPacketInst(&packet_, id, INST_READ, tx_param, 4);
  packet_.tx_time = micros() - pre_time_us;

  pre_time_ms = millis();
  pre_time_us = micros();

  // Receive Status Packet  
  while(1) {
    last_dxl_return_ = dxlRxPacket(&packet_);

    if (last_dxl_return_ == DXL_RET_RX_STATUS){
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
      last_status_error_ = packet_.rx.error;

      break;
    }else if (last_dxl_return_ != DXL_RET_EMPTY){
      break;
    }else{
// #ifdef _USE_HW_RTOS
//         osThreadYield();
// #endif
    }

    if (millis()-pre_time_ms >= timeout) {
      last_status_error_ = DXL_RET_ERROR_TIMEOUT;
      break;
    }
  }

  //Serial.print("read dxl ret : ");Serial.println(getLastDxlReturn());

  return recv_param_len;
}

bool Master::write(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t data_length, uint32_t timeout)
{
  bool ret = false;
  uint32_t pre_time_us, pre_time_ms;
  
  if(writeNoResp(id, addr, p_data, data_length) != false){
    Serial.print("write dxl ret : ");Serial.println(getLastDxlReturn());
    return ret;
  }
    
  pre_time_ms = millis();
  pre_time_us = micros();
  while(1)
  {
    last_dxl_return_ = dxlRxPacket(&packet_);
    if (last_dxl_return_ == DXL_RET_RX_STATUS) {
      pre_time_ms = millis();
      packet_.rx_time = micros() - pre_time_us;

      last_status_error_ = packet_.rx.error;

      last_dxl_return_ = DXL_RET_RX_RESP;
      ret = true;
      break;
    } else if (last_dxl_return_ != DXL_RET_EMPTY) {
      break;
    }

    if (millis()-pre_time_ms >= timeout)
    {
      last_status_error_ = DXL_RET_ERROR_TIMEOUT;
      break;
    }
  }

  //Serial.print("write dxl ret : ");Serial.println(getLastDxlReturn());

  return ret;
}

bool Master::writeNoResp(uint8_t id, uint16_t addr, uint8_t *p_data, uint16_t data_length)
{
  bool ret = false;
  uint32_t pre_time_us;
  uint8_t  tx_param[2 + DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE];
  uint16_t tx_length;
  uint32_t i;

  if (id == DXL_BROADCAST_ID) {
    last_dxl_return_ = DXL_RET_ERROR_NOT_SUPPORT_BROADCAST;
    return ret;
  }

  if(data_length == 0) {
    last_dxl_return_ = DXL_RET_ERROR_ADDR_LENGTH;
    return ret;
  }

  if (data_length > DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE){
    last_dxl_return_ = DXL_RET_ERROR_LENGTH;
    return ret;
  }

  if (p_port_->getOpenState() != true)
  {
    last_dxl_return_ = DXL_RET_NOT_OPEN;
    return ret;
  }  

  if (packet_.packet_ver == DXL_PACKET_VER_1_0 )
  {
    tx_param[0] = addr;
    for (i=0; i<data_length; i++)
    {
      tx_param[1 + i] = p_data[i];
    }
    tx_length = 1 + data_length;
  }
  else
  {
    tx_param[0] = addr >> 0;
    tx_param[1] = addr >> 8;

    for (i=0; i<data_length; i++)
    {
      tx_param[2 + i] = p_data[i];
    }

    tx_length = 2 + data_length;
  }

  pre_time_us = micros();
  last_dxl_return_ = dxlTxPacketInst(&packet_, id, INST_WRITE, tx_param, tx_length);
  packet_.tx_time = micros() - pre_time_us;

  return ret;
}

dxl_return_t Master::factoryReset(uint8_t id, uint8_t option, uint32_t timeout)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;
  uint8_t tx_param[1];

  if (p_port_->getOpenState() == true)
  {
    tx_param[0] = option;

    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, id, INST_RESET, tx_param, 1);
    packet_.tx_time = micros() - pre_time_us;

    if (id == DXL_BROADCAST_ID)
    {
      return ret;
    }

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      ret = dxlRxPacket(&packet_);
      if (ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        packet_.rx_time = micros() - pre_time_us;

        last_status_error_ = packet_.rx.error;

        ret = DXL_RET_RX_RESP;
        break;
      }
      else if (ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}

dxl_return_t Master::reboot(uint8_t id, uint32_t timeout)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  if (p_port_->getOpenState() == true)
  {
    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, id, INST_REBOOT, NULL, 0);
    packet_.tx_time = micros() - pre_time_us;

    if (id == DXL_BROADCAST_ID)
    {
      return ret;
    }

    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      ret = dxlRxPacket(&packet_);
      if (ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        packet_.rx_time = micros() - pre_time_us;

        last_status_error_ = packet_.rx.error;

        ret = DXL_RET_RX_RESP;
        break;
      }
      else if (ret != DXL_RET_EMPTY)
      {
        break;
      }


      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}

dxl_return_t Master::syncRead(param_sync_read_t *p_param, status_read_t *p_resp, uint32_t timeout)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE];
  uint32_t mem_addr;
  uint8_t  *p_mem = (uint8_t *)p_resp->mem;
  uint32_t i;


  p_resp->id_count = 0;
  p_resp->p_node[0] = (read_node_t *)&p_mem[0];


  if (p_port_->getOpenState() == true)
  {
    tx_param[0] = p_param->addr >> 0;
    tx_param[1] = p_param->addr >> 8;
    tx_param[2] = p_param->length >> 0;
    tx_param[3] = p_param->length >> 8;

    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[4+i] = p_param->id_tbl[i];
    }

    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_READ, tx_param, 4 + p_param->id_count);
    packet_.tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      ret = dxlRxPacket(&packet_);
      if (ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        packet_.rx_time = micros() - pre_time_us;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(read_node_t);
        if (mem_addr%4)
        {
          mem_addr += 4 - (mem_addr%4);
        }
        p_resp->p_node[p_resp->id_count]->p_data = &p_mem[mem_addr];

        p_resp->p_node[p_resp->id_count]->id     = packet_.rx.id;
        p_resp->p_node[p_resp->id_count]->error  = packet_.rx.error;
        p_resp->p_node[p_resp->id_count]->length = packet_.rx.param_length;

        for (i=0; i<packet_.rx.param_length; i++)
        {
          p_resp->p_node[p_resp->id_count]->p_data[i] = packet_.rx.p_param[i];
        }

        p_resp->id_count++;

        mem_addr += packet_.rx.param_length;
        p_resp->p_node[p_resp->id_count] = (read_node_t *)&p_mem[mem_addr];

        if (p_resp->id_count >= p_param->id_count)
        {
          ret = DXL_RET_RX_RESP;
          break;
        }
      }
      else
      {
// #ifdef _USE_HW_RTOS
//         osThreadYield();
// #endif
      }

      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}

dxl_return_t Master::syncWrite(param_sync_write_t *p_param)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE];
  uint32_t i;
  uint32_t j;
  uint32_t data_index;

  if (p_port_->getOpenState() == true)
  {

    if (p_param->length > DXLCMD_MAX_NODE * DXLCMD_MAX_NODE_BUFFER_SIZE)
    {
      return DXL_RET_ERROR_LENGTH;
    }

    if (packet_.packet_ver == DXL_PACKET_VER_1_0 )
    {
      tx_param[0] = p_param->addr;
      tx_param[1] = p_param->length;

      data_index = 2;
      for( i=0; i<p_param->id_count; i++)
      {
        tx_param[data_index++] = p_param->node[i].id;
        for (j=0; j<p_param->length; j++)
        {
          tx_param[data_index++] = p_param->node[i].data[j];
        }
      }
    }
    else
    {
      tx_param[0] = p_param->addr >> 0;
      tx_param[1] = p_param->addr >> 8;
      tx_param[2] = p_param->length >> 0;
      tx_param[3] = p_param->length >> 8;

      data_index = 4;
      for( i=0; i<p_param->id_count; i++)
      {
        tx_param[data_index++] = p_param->node[i].id;
        for (j=0; j<p_param->length; j++)
        {
          tx_param[data_index++] = p_param->node[i].data[j];
        }
      }
    }

    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_SYNC_WRITE, tx_param, data_index);
    packet_.tx_time = micros() - pre_time_us;
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}

dxl_return_t Master::bulkRead(param_bulk_read_t *p_param, status_read_t *p_resp, uint32_t timeout)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;
  uint32_t pre_time_ms;

  uint8_t tx_param[DXLCMD_MAX_NODE * 5];
  uint32_t mem_addr;
  uint8_t *p_mem = (uint8_t *)p_resp->mem;
  uint32_t i;
  uint16_t tx_length;

  p_resp->id_count = 0;
  p_resp->p_node[0] = (read_node_t *)&p_mem[0];


  if (p_port_->getOpenState() == true)
  {
    tx_length = 0;
    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[tx_length+0] = p_param->id_tbl[i];
      tx_param[tx_length+1] = p_param->addr[i] >> 0;
      tx_param[tx_length+2] = p_param->addr[i] >> 8;
      tx_param[tx_length+3] = p_param->length[i] >> 0;
      tx_param[tx_length+4] = p_param->length[i] >> 8;
      tx_length += 5;
    }

    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_READ, tx_param, tx_length);
    packet_.tx_time = micros() - pre_time_us;

    mem_addr = 0;
    pre_time_ms = millis();
    pre_time_us = micros();
    while(1)
    {
      ret = dxlRxPacket(&packet_);
      if (ret == DXL_RET_RX_STATUS)
      {
        pre_time_ms = millis();
        packet_.rx_time = micros() - pre_time_us;

        mem_addr += sizeof(read_node_t);
        p_resp->p_node[p_resp->id_count]->p_data = &p_mem[mem_addr];

        p_resp->p_node[p_resp->id_count]->id     = packet_.rx.id;
        p_resp->p_node[p_resp->id_count]->error  = packet_.rx.error;
        p_resp->p_node[p_resp->id_count]->length = packet_.rx.param_length;

        for (i=0; i<packet_.rx.param_length; i++)
        {
          p_resp->p_node[p_resp->id_count]->p_data[i] = packet_.rx.p_param[i];
        }

        p_resp->id_count++;

        mem_addr += packet_.rx.param_length;

        //-- 주소를 4바이트로 정렬( 구조체를 직접 타입변환하여 사용하기 위해서 )
        //
        mem_addr += sizeof(read_node_t);
        if (mem_addr%4)
        {
          mem_addr += 4 - (mem_addr%4);
        }

        p_resp->p_node[p_resp->id_count] = (read_node_t *)&p_mem[mem_addr];

        if (p_resp->id_count >= p_param->id_count)
        {
          ret = DXL_RET_RX_RESP;
          break;
        }
      }
      else
      {
// #ifdef _USE_HW_RTOS
//         osThreadYield();
// #endif
      }

      if (millis()-pre_time_ms >= timeout)
      {
        break;
      }
    }
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}

dxl_return_t Master::bulkWrite(param_bulk_write_t *p_param)
{
  dxl_return_t ret = DXL_RET_OK;
  uint32_t pre_time_us;

  uint8_t tx_param[4 + DXLCMD_MAX_NODE];
  uint32_t i;
  uint32_t j;
  uint32_t data_index;
  uint32_t tx_buf_length;


  if (p_port_->getOpenState() == true)
  {
    tx_buf_length = sizeof(tx_param);
    data_index = 0;

    for( i=0; i<p_param->id_count; i++)
    {
      tx_param[data_index++] = p_param->node[i].id;
      tx_param[data_index++] = p_param->node[i].addr >> 0;
      tx_param[data_index++] = p_param->node[i].addr >> 8;
      tx_param[data_index++] = p_param->node[i].length >> 0;
      tx_param[data_index++] = p_param->node[i].length >> 8;
      for (j=0; j<p_param->node[i].length; j++)
      {
        tx_param[data_index++] = p_param->node[i].data[j];
      }

      if (data_index > tx_buf_length)
      {
        return DXL_RET_ERROR_LENGTH;
      }
    }

    pre_time_us = micros();
    ret = dxlTxPacketInst(&packet_, DXL_BROADCAST_ID, INST_BULK_WRITE, tx_param, data_index);
    packet_.tx_time = micros() - pre_time_us;
  }
  else
  {
    ret = DXL_RET_NOT_OPEN;
  }

  return ret;
}


uint8_t Master::getLastStatusError() const
{
  return last_status_error_;
}

dxl_return_t Master::getLastDxlReturn() const
{
  return last_dxl_return_;
}

