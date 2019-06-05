


#include "protocol.h"

using namespace DYNAMIXEL;

#define PACKET_2_0_STATE_IDLE       0
#define PACKET_2_0_STATE_RESERVED   1
#define PACKET_2_0_STATE_ID         2
#define PACKET_2_0_STATE_LENGTH_L   3
#define PACKET_2_0_STATE_LENGTH_H   4
#define PACKET_2_0_STATE_DATA       5
#define PACKET_2_0_STATE_CRC_L      6
#define PACKET_2_0_STATE_CRC_H      7


#define PACKET_1_0_STATE_IDLE       0
#define PACKET_1_0_STATE_ID         1
#define PACKET_1_0_STATE_LENGTH     2
#define PACKET_1_0_STATE_DATA       3
#define PACKET_1_0_STATE_CHECK_SUM  4


#define DXL_DIR_PIN_RX              0
#define DXL_DIR_PIN_TX              1

//-- Internal Variables
//

//-- External Variables
//


//-- Internal Functions
//
static lib_err_code_t dxlRxPacket1_0(dxl_t *p_packet, uint8_t data_in);
static lib_err_code_t dxlMakePacketStatus1_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlTxPacketInst1_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length );

static lib_err_code_t dxlRxPacket2_0(dxl_t *p_packet, uint8_t data_in);
static lib_err_code_t dxlTxPacketInst2_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length );
static lib_err_code_t dxlMakePacketStatus2_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );

static uint16_t dxlAddStuffing(dxl_t *p_packet, uint8_t *p_data, uint16_t length);
static uint16_t dxlRemoveStuffing(uint8_t *p_data, uint16_t length);

#if (USE_SLAVE_FUNC)
static lib_err_code_t dxlInstPing(dxl_t *p_dxl);
static lib_err_code_t dxlInstFactoryReset(dxl_t *p_dxl);
static lib_err_code_t dxlInstReboot(dxl_t *p_dxl);
static lib_err_code_t dxlInstRead(dxl_t *p_dxl);
static lib_err_code_t dxlInstWrite(dxl_t *p_dxl);
static lib_err_code_t dxlInstSyncRead(dxl_t *p_dxl);
static lib_err_code_t dxlInstSyncWrite(dxl_t *p_dxl);
static lib_err_code_t dxlInstBulkRead(dxl_t *p_dxl);
static lib_err_code_t dxlInstBulkWrite(dxl_t *p_dxl);

static lib_err_code_t dxlInstPing1_0(dxl_t *p_dxl);
static lib_err_code_t dxlInstPing2_0(dxl_t *p_dxl);

static lib_err_code_t dxlInstRead1_0(dxl_t *p_dxl);
static lib_err_code_t dxlInstRead2_0(dxl_t *p_dxl);

static lib_err_code_t dxlInstWrite1_0(dxl_t *p_dxl);
static lib_err_code_t dxlInstWrite2_0(dxl_t *p_dxl);

static lib_err_code_t dxlInstSyncWrite1_0(dxl_t *p_dxl);
static lib_err_code_t dxlInstSyncWrite2_0(dxl_t *p_dxl);
#endif

static void dxlUpdateCrc(uint16_t *p_crc_cur, uint8_t data_in);

//-- External Functions
//


static PortHandler *p_port;

bool DYNAMIXEL::setDxlPort(PortHandler *port)
{
  if(port == nullptr)
    return false;
  
  p_port = port;
  return true;
}

bool DYNAMIXEL::dxlInit(dxl_t *p_packet, float protocol_ver)
{
  p_packet->header_cnt = 0;
  p_packet->packet_ver = protocol_ver;
  p_packet->dxl_mode   = DXL_MODE_MASTER;

  p_packet->rx_state   = PACKET_2_0_STATE_IDLE;

  p_packet->process_state = DXL_PROCESS_INST;

  p_packet->id         = 200;

  p_packet->rx_timeout = (uint32_t)500*(uint32_t)1000;
  p_packet->tx_timeout = (uint32_t)500*(uint32_t)1000;

  p_packet->rx.header[0] = 0;
  p_packet->rx.header[1] = 0;
  p_packet->rx.header[2] = 0;

#if (USE_SLAVE_FUNC)
  p_packet->inst_func.ping          = (dxlInstFunc_t)dxlInstPing;
  p_packet->inst_func.read          = (dxlInstFunc_t)dxlInstRead;
  p_packet->inst_func.write         = (dxlInstFunc_t)dxlInstWrite;
  p_packet->inst_func.reg_write     = nullptr;
  p_packet->inst_func.action        = nullptr;
  p_packet->inst_func.factory_reset = (dxlInstFunc_t)dxlInstFactoryReset;
  p_packet->inst_func.reboot        = (dxlInstFunc_t)dxlInstReboot;
  p_packet->inst_func.status        = nullptr;
  p_packet->inst_func.sync_read     = (dxlInstFunc_t)dxlInstSyncRead;
  p_packet->inst_func.sync_write    = (dxlInstFunc_t)dxlInstSyncWrite;
  p_packet->inst_func.bulk_read     = (dxlInstFunc_t)dxlInstBulkRead;
  p_packet->inst_func.bulk_write    = (dxlInstFunc_t)dxlInstBulkWrite;

  p_packet->process_func.processPing          = nullptr;
  p_packet->process_func.processReboot        = nullptr;
  p_packet->process_func.processFactoryReset  = nullptr;
  p_packet->process_func.processRead          = nullptr;
  p_packet->process_func.processWrite         = nullptr;
#endif

  p_packet->read_hook_func = nullptr;

  return true;
}

bool DYNAMIXEL::dxlSetId(dxl_t *p_packet, uint8_t id)
{
  p_packet->id = id;

  return true;
}

uint8_t DYNAMIXEL::dxlGetId(dxl_t *p_packet)
{
  return p_packet->id;
}

bool DYNAMIXEL::dxlSetProtocolVersion(dxl_t *p_packet, float protocol_version)
{
  bool ret = true;

  if(protocol_version == DXL_PACKET_VER_1_0){
    p_packet->packet_ver = protocol_version;
  }else if(protocol_version == DXL_PACKET_VER_2_0){
    p_packet->packet_ver = protocol_version;
  }else{
    ret = false;
  }

  return ret;
}

float DYNAMIXEL::dxlGetProtocolVersion(dxl_t *p_packet)
{
  return p_packet->packet_ver;
}

#if (USE_SLAVE_FUNC)
void DYNAMIXEL::dxlAddInstFunc(dxl_t *p_packet, uint8_t inst, dxlInstFunc_t func)
{
  switch(inst)
  {
    case INST_PING:
      p_packet->inst_func.ping = (dxlInstFunc_t)func;
      break;

    case INST_READ:
      p_packet->inst_func.read = (dxlInstFunc_t)func;
      break;

    case INST_WRITE:
      p_packet->inst_func.write = (dxlInstFunc_t)func;
      break;

    case INST_REG_WRITE:
      p_packet->inst_func.reg_write = (dxlInstFunc_t)func;
      break;

    case INST_ACTION:
      p_packet->inst_func.action = (dxlInstFunc_t)func;
      break;

    case INST_RESET:
      p_packet->inst_func.factory_reset = (dxlInstFunc_t)func;
      break;

    case INST_REBOOT:
      p_packet->inst_func.reboot = (dxlInstFunc_t)func;
      break;

    case INST_STATUS:
      p_packet->inst_func.status = (dxlInstFunc_t)func;
      break;

    case INST_SYNC_READ:
      p_packet->inst_func.sync_read = (dxlInstFunc_t)func;
      break;

    case INST_SYNC_WRITE:
      p_packet->inst_func.sync_write = (dxlInstFunc_t)func;
      break;

    case INST_BULK_READ:
      p_packet->inst_func.bulk_read = (dxlInstFunc_t)func;
      break;

    case INST_BULK_WRITE:
      p_packet->inst_func.bulk_write = (dxlInstFunc_t)func;
      break;
  }
}

void DYNAMIXEL::dxlAddProcessPingFunc(dxl_t *p_packet, uint8_t (*func)(uint8_t *p_data, uint16_t *p_length))
{
  p_packet->process_func.processPing = func;
}

void DYNAMIXEL::dxlAddProcessRebootFunc(dxl_t *p_packet, uint8_t (*func)(void))
{
  p_packet->process_func.processReboot = func;
}

void DYNAMIXEL::dxlAddProcessFactoryResetFunc(dxl_t *p_packet, uint8_t (*func)(uint8_t mode))
{
  p_packet->process_func.processFactoryReset = func;
}

void DYNAMIXEL::dxlAddProcessReadFunc(dxl_t *p_packet, uint8_t (*func)(uint16_t addr, uint8_t *p_data, uint16_t length))
{
  p_packet->process_func.processRead = func;
}

void DYNAMIXEL::dxlAddProcessWriteFunc(dxl_t *p_packet, uint8_t (*func)(uint16_t addr, uint8_t *p_data, uint16_t length))
{
  p_packet->process_func.processWrite = func;
}


uint8_t dxlProcessPacket(dxl_t *p_packet)
{
  lib_err_code_t lib_err;


  p_packet->dxl_mode = DXL_MODE_SLAVE;

  switch (p_packet->process_state)
  {
    //-- INST
    //
    case DXL_PROCESS_INST:
      lib_err = dxlRxPacket(p_packet);

      if (p_packet->rx.type == RX_PACKET_TYPE_INST)
      {

        lib_err = dxlProcessInst(p_packet);

        if (lib_err == DXL_RET_PROCESS_BROAD_PING)
        {
          p_packet->present_id = 1;
          p_packet->process_pre_time = micros();
          p_packet->process_state = DXL_PROCESS_BROAD_PING;
        }

        if (lib_err == DXL_RET_PROCESS_BROAD_READ)
        {
          p_packet->process_pre_time = micros();
          p_packet->process_state = DXL_PROCESS_BROAD_READ;
        }
      }
      break;


    //-- BROAD_PING
    //
    case DXL_PROCESS_BROAD_PING:
      lib_err = dxlRxPacket(p_packet);

      if (p_packet->rx.type == RX_PACKET_TYPE_STATUS)
      {
        p_packet->present_id = p_packet->rx.id + 1;
      }
      else
      {
        if (micros()-p_packet->process_pre_time >= 3000)
        {
          p_packet->process_pre_time = micros();
          p_packet->present_id++;
        }
      }

      if (p_packet->present_id == p_packet->id)
      {
        dxlTxPacket(p_packet);
        p_packet->process_state = DXL_PROCESS_INST;
      }

      break;

    //-- BROAD_READ
    //
    case DXL_PROCESS_BROAD_READ:
      lib_err = dxlRxPacket(p_packet);

      if (p_packet->rx.type == RX_PACKET_TYPE_STATUS)
      {
        p_packet->process_pre_time = micros();
        if (p_packet->pre_id == p_packet->rx.id)
        {
          dxlTxPacket(p_packet);
          p_packet->process_state = DXL_PROCESS_INST;
        }
      }
      else
      {
        if (micros()-p_packet->process_pre_time >= 50000)
        {
          p_packet->process_state = DXL_PROCESS_INST;
        }
      }
      break;


    default:
      p_packet->process_state = DXL_PROCESS_INST;
      break;
  }

  p_packet->dxl_mode = DXL_MODE_MASTER;

  return p_packet->process_state;
}

lib_err_code_t DYNAMIXEL::dxlProcessInst(dxl_t *p_packet)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint8_t inst;
  dxlInstFunc_t func;

  inst = p_packet->rx.cmd;
  func = nullptr;

  switch(inst)
  {
    case INST_PING:
      func = p_packet->inst_func.ping;
      break;

    case INST_READ:
      func = p_packet->inst_func.read;
      break;

    case INST_WRITE:
      func = p_packet->inst_func.write;
      break;

    case INST_REG_WRITE:
      func = p_packet->inst_func.reg_write;
      break;

    case INST_ACTION:
      func = p_packet->inst_func.action;
      break;

    case INST_RESET:
      func = p_packet->inst_func.factory_reset;
      break;

    case INST_REBOOT:
      func = p_packet->inst_func.reboot;
      break;

    case INST_STATUS:
      func = p_packet->inst_func.status;
      break;

    case INST_SYNC_READ:
      func = p_packet->inst_func.sync_read;
      break;

    case INST_SYNC_WRITE:
      func = p_packet->inst_func.sync_write;
      break;

    case INST_BULK_READ:
      func = p_packet->inst_func.bulk_read;
      break;

    case INST_BULK_WRITE:
      func = p_packet->inst_func.bulk_write;
      break;
  }

  if (func != nullptr)
  {
    if (p_packet->rx.id != dxlGetId(p_packet) && p_packet->rx.id != DXL_BROADCAST_ID)
    {
      ret = DXL_LIB_ERROR_INVAILD_ID;
    }
    else
    {
      ret = func(p_packet);
    }
  }

  return ret;
}
#endif

void DYNAMIXEL::dxlSetRxByteISR(dxl_t *p_packet, void (*p_func)(uint8_t data))
{
  p_packet->read_hook_func = p_func;
}

uint32_t DYNAMIXEL::dxlRxAvailable()
{
  return (uint32_t)p_port->available();
}

uint8_t DYNAMIXEL::dxlRxRead()
{
  return (uint8_t)p_port->read();
}

void DYNAMIXEL::dxlTxWrite(uint8_t *p_data, uint32_t length)
{
  p_port->write(p_data, length);
}

lib_err_code_t DYNAMIXEL::dxlRxPacket(dxl_t *p_packet)
{
  uint8_t data;
  lib_err_code_t ret = DXL_LIB_PROCEEDING;

  while (dxlRxAvailable())
  {
    data = dxlRxRead();
    ret  = dxlRxPacketDataIn(p_packet, data);

    if (p_packet->dxl_mode == DXL_MODE_SLAVE)
    {
      if (p_packet->read_hook_func != nullptr)
      {
        (*p_packet->read_hook_func)(data);
      }
    }

    if (ret != DXL_LIB_PROCEEDING)
    {
      break;
    }
  }

  return ret;
}

lib_err_code_t DYNAMIXEL::dxlRxPacketDataIn(dxl_t *p_packet, uint8_t data_in)
{
  lib_err_code_t ret = DXL_LIB_PROCEEDING;

  if (p_packet->packet_ver == DXL_PACKET_VER_1_0){
    ret = dxlRxPacket1_0(p_packet, data_in);
  }else{
    ret = dxlRxPacket2_0(p_packet, data_in);
  }

  return ret;
}

lib_err_code_t dxlRxPacket1_0(dxl_t *p_packet, uint8_t data_in)
{
  lib_err_code_t ret = DXL_LIB_PROCEEDING;

  // time out
  if((micros() - p_packet->prev_time) > p_packet->rx_timeout){
    p_packet->rx_state   = PACKET_1_0_STATE_IDLE;
    p_packet->prev_time  = micros();
    p_packet->header_cnt = 0;
  }
  p_packet->prev_time  = micros();

  switch(p_packet->rx_state)
  {
    case PACKET_1_0_STATE_IDLE:

      if( p_packet->header_cnt >= 1 )
      {
        p_packet->rx.header[1] = data_in;

        if(    p_packet->rx.header[0] == 0xFF
            && p_packet->rx.header[1] == 0xFF )
        {
          p_packet->header_cnt   = 0;
          p_packet->rx.check_sum = 0;
          p_packet->rx_state = PACKET_1_0_STATE_ID;
        }
        else
        {
          p_packet->rx.header[0] = p_packet->rx.header[1];
          p_packet->rx.header[1] = 0;
        }
      }
      else
      {
        p_packet->rx.header[p_packet->header_cnt] = data_in;
        p_packet->header_cnt++;
      }
      break;

    case PACKET_1_0_STATE_ID:
      p_packet->rx.id       = data_in;
      p_packet->rx_state    = PACKET_1_0_STATE_LENGTH;
      p_packet->rx.check_sum += data_in;
      break;

    case PACKET_1_0_STATE_LENGTH:
      p_packet->rx.packet_length = data_in;
      p_packet->rx_state         = PACKET_1_0_STATE_DATA;
      p_packet->rx.index         = 0;
      p_packet->rx.check_sum    += data_in;

      if (p_packet->rx.packet_length > DXL_BUF_LENGTH-4)
      {
        p_packet->rx_state = PACKET_1_0_STATE_IDLE;
        ret = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      }
      if (p_packet->rx.packet_length < 2)
      {
        p_packet->rx_state = PACKET_1_0_STATE_IDLE;
        ret = DXL_LIB_ERROR_LENGTH;
      }
      break;

    case PACKET_1_0_STATE_DATA:
      p_packet->rx.data[p_packet->rx.index] = data_in;
      p_packet->rx.check_sum += data_in;

      p_packet->rx.index++;

      if (p_packet->rx.index >= p_packet->rx.packet_length-1)
      {
        p_packet->rx_state = PACKET_1_0_STATE_CHECK_SUM;
      }
      break;

    case PACKET_1_0_STATE_CHECK_SUM:
      p_packet->rx.check_sum          = ~(p_packet->rx.check_sum);
      p_packet->rx.check_sum_received = data_in;

      if (p_packet->rx.check_sum_received == p_packet->rx.check_sum)
      {
        p_packet->rx.cmd   = p_packet->rx.data[0];
        p_packet->rx.error = p_packet->rx.data[0];

        if (p_packet->dxl_mode == DXL_MODE_MASTER)
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 2;
          p_packet->rx.type = RX_PACKET_TYPE_STATUS;
        }
        else
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 2;
          p_packet->rx.type = RX_PACKET_TYPE_INST;
        }

        ret = DXL_LIB_OK;
      }
      else
      {
        ret = DXL_LIB_ERROR_CHECK_SUM;
      }

      p_packet->rx_state = PACKET_1_0_STATE_IDLE;
      break;

    default:
      p_packet->rx_state = PACKET_1_0_STATE_IDLE;
      break;
  }

  return ret;
}

lib_err_code_t dxlRxPacket2_0(dxl_t *p_packet, uint8_t data_in)
{
  lib_err_code_t ret = DXL_LIB_PROCEEDING;
  uint16_t stuff_length;

  // time out
  if( (micros() - p_packet->prev_time) > p_packet->rx_timeout )
  {
    p_packet->rx_state   = PACKET_2_0_STATE_IDLE;
    p_packet->header_cnt = 0;
  }
  p_packet->prev_time  = micros();


  switch(p_packet->rx_state)
  {
    case PACKET_2_0_STATE_IDLE:
      if( p_packet->header_cnt >= 2 )
      {
        p_packet->rx.header[2] = data_in;

        if(    p_packet->rx.header[0] == 0xFF
            && p_packet->rx.header[1] == 0xFF
            && p_packet->rx.header[2] == 0xFD )
        {
          p_packet->header_cnt = 0;
          p_packet->rx.crc     = 0;
          dxlUpdateCrc(&p_packet->rx.crc, 0xFF);
          dxlUpdateCrc(&p_packet->rx.crc, 0xFF);
          dxlUpdateCrc(&p_packet->rx.crc, 0xFD);
          p_packet->rx_state = PACKET_2_0_STATE_RESERVED;
        }
        else
        {
          p_packet->rx.header[0] = p_packet->rx.header[1];
          p_packet->rx.header[1] = p_packet->rx.header[2];
          p_packet->rx.header[2] = 0;
        }
      }
      else
      {
        p_packet->rx.header[p_packet->header_cnt] = data_in;
        p_packet->header_cnt++;
      }
      break;

    case PACKET_2_0_STATE_RESERVED:
      if( data_in == 0xFD )
      {
        p_packet->rx_state  = PACKET_2_0_STATE_IDLE;
        ret = DXL_LIB_ERROR_WRONG_PACKET;
      }
      else
      {
        p_packet->rx.reserved = data_in;
        p_packet->rx_state    = PACKET_2_0_STATE_ID;
      }
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_2_0_STATE_ID:
      p_packet->rx.id       = data_in;
      p_packet->rx_state    = PACKET_2_0_STATE_LENGTH_L;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_2_0_STATE_LENGTH_L:
      p_packet->rx.packet_length = data_in;
      p_packet->rx_state         = PACKET_2_0_STATE_LENGTH_H;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);
      break;

    case PACKET_2_0_STATE_LENGTH_H:
      p_packet->rx.packet_length |= data_in<<8;
      p_packet->rx_state          = PACKET_2_0_STATE_DATA;
      p_packet->rx.index          = 0;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);

      if (p_packet->rx.packet_length > DXL_BUF_LENGTH)
      {
        p_packet->rx_state = PACKET_2_0_STATE_IDLE;
        ret = DXL_LIB_ERROR_BUFFER_OVERFLOW;
      }
      if (p_packet->rx.packet_length < 3)
      {
        p_packet->rx_state = PACKET_2_0_STATE_IDLE;
        ret = DXL_LIB_ERROR_LENGTH;
      }
      break;

    case PACKET_2_0_STATE_DATA:
      p_packet->rx.data[p_packet->rx.index] = data_in;
      dxlUpdateCrc(&p_packet->rx.crc, data_in);

      p_packet->rx.index++;

      if (p_packet->rx.index >= p_packet->rx.packet_length-2)
      {
        p_packet->rx_state = PACKET_2_0_STATE_CRC_L;
      }
      break;

    case PACKET_2_0_STATE_CRC_L:
      p_packet->rx.crc_received = data_in;
      p_packet->rx_state        = PACKET_2_0_STATE_CRC_H;
      break;

    case PACKET_2_0_STATE_CRC_H:
      p_packet->rx.crc_received |= data_in<<8;

      stuff_length = dxlRemoveStuffing(p_packet->rx.data, p_packet->rx.packet_length);
      p_packet->rx.packet_length -= stuff_length;

      if (p_packet->rx.crc_received == p_packet->rx.crc)
      {
        p_packet->rx.cmd   = p_packet->rx.data[0];
        p_packet->rx.error = p_packet->rx.data[1];

        if (p_packet->rx.data[0] == INST_STATUS)
        {
          p_packet->rx.p_param      = &p_packet->rx.data[2];
          p_packet->rx.param_length = p_packet->rx.packet_length - 4;
          p_packet->rx.type = RX_PACKET_TYPE_STATUS;
        }
        else
        {
          p_packet->rx.p_param      = &p_packet->rx.data[1];
          p_packet->rx.param_length = p_packet->rx.packet_length - 3;
          p_packet->rx.type = RX_PACKET_TYPE_INST;
        }
        ret = DXL_LIB_OK;
      }
      else
      {
        ret = DXL_LIB_ERROR_CRC;
      }

      p_packet->rx_state = PACKET_2_0_STATE_IDLE;
      break;

    default:
      p_packet->rx_state = PACKET_2_0_STATE_IDLE;
      break;
  }

  return ret;
}

uint16_t dxlRemoveStuffing(uint8_t *p_data, uint16_t length)
{
  uint16_t i;
  uint16_t stuff_length;
  uint16_t index;

  index = 0;
  stuff_length = 0;
  for( i=0; i<length; i++ )
  {
    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        i++;
        stuff_length++;
      }
    }
    p_data[index++] = p_data[i];
  }

  return stuff_length;
}

uint16_t dxlAddStuffing(dxl_t *p_packet, uint8_t *p_data, uint16_t length)
{
  uint8_t *stuff_buf;
  uint16_t i;
  uint16_t index;
  uint16_t stuff_length = 0;

  stuff_buf = p_packet->rx.data;

  if (length <= 2)
  {
    return 0;
  }

  index = 0;
  stuff_length = 0;
  for( i=0; i<length-2; i++ )
  {
    stuff_buf[index++] = p_data[i];

    if( i >= 2 )
    {
      if( p_data[i-2] == 0xFF && p_data[i-1] == 0xFF  && p_data[i] == 0xFD )
      {
        stuff_buf[index++] = 0xFD;
        stuff_length++;
      }
    }
  }

  if( stuff_length > 0 )
  {
    for( i=0; i<index; i++ )
    {
      p_data[i] = stuff_buf[i];
    }
  }

  return stuff_length;
}

lib_err_code_t DYNAMIXEL::dxlTxPacket(dxl_t *p_packet)
{
  lib_err_code_t ret = DXL_LIB_OK;

  dxlTxWrite(p_packet->tx.data, p_packet->tx.packet_length);

  return ret;
}

lib_err_code_t DYNAMIXEL::dxlTxPacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  ret = dxlMakePacketStatus(p_packet, id, error, p_data, length);
  if (ret == DXL_LIB_OK) {
    dxlTxWrite(p_packet->tx.data, p_packet->tx.packet_length);
  }

  return ret;
}

lib_err_code_t DYNAMIXEL::dxlMakePacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret;

  if (p_packet->packet_ver == DXL_PACKET_VER_1_0){
    ret = dxlMakePacketStatus1_0(p_packet, id, error, p_data, length);
  }else{
    ret = dxlMakePacketStatus2_0(p_packet, id, error, p_data, length);
  }

  return ret;
}

lib_err_code_t dxlMakePacketStatus1_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
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

  p_packet->tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_ID_IDX]    = id;
  p_packet->tx.data[PKT_1_0_ERROR_IDX] = error;

  check_sum += id;
  check_sum += packet_length;
  check_sum += error;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_1_0_STATUS_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  p_packet->tx.data[PKT_1_0_LEN_IDX] = packet_length;
  p_packet->tx.data[PKT_1_0_ERROR_IDX + packet_length - 1] = ~(check_sum);

  return ret;
}

lib_err_code_t dxlMakePacketStatus2_0(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length )
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

  p_packet->tx.data[PKT_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_3_IDX] = 0xFD;
  p_packet->tx.data[PKT_RSV_IDX]   = 0x00;
  p_packet->tx.data[PKT_ID_IDX]    = id;
  p_packet->tx.data[PKT_INST_IDX]  = INST_STATUS;
  p_packet->tx.data[PKT_ERROR_IDX] = error;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_STATUS_PARAM_IDX + i] = p_data[i];
  }

  stuff_length = dxlAddStuffing(p_packet, &p_packet->tx.data[PKT_INST_IDX], length + 2); // + instruction + error
  packet_length += stuff_length;

  p_packet->tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  p_packet->tx.data[PKT_LEN_H_IDX] = packet_length >> 8;

  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, p_packet->tx.data[i]);
  }


  p_packet->tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  p_packet->tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;

  p_packet->tx.packet_length = packet_length + 7;

  return ret;
}

lib_err_code_t DYNAMIXEL::dxlTxPacketInst(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_PROCEEDING;

  if (p_packet->packet_ver == DXL_PACKET_VER_1_0)
  {
    ret = dxlTxPacketInst1_0(p_packet, id, inst_cmd, p_data, length);
  }
  else
  {
    ret = dxlTxPacketInst2_0(p_packet, id, inst_cmd, p_data, length);
  }

  return ret;
}

lib_err_code_t dxlTxPacketInst1_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
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

  p_packet->tx.data[PKT_1_0_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_1_0_ID_IDX]    = id;
  p_packet->tx.data[PKT_1_0_INST_IDX]  = inst_cmd;

  check_sum += id;
  check_sum += packet_length;
  check_sum += inst_cmd;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_1_0_INST_PARAM_IDX + i] = p_data[i];
    check_sum += p_data[i];
  }

  p_packet->tx.data[PKT_1_0_LEN_IDX] = packet_length;
  p_packet->tx.data[PKT_1_0_INST_IDX + packet_length - 1] = ~(check_sum);

  dxlTxWrite(p_packet->tx.data, packet_length + 4);

  return ret;
}

lib_err_code_t dxlTxPacketInst2_0(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length )
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t i = 0;
  uint16_t packet_length;
  uint16_t stuff_length;
  uint16_t crc;

  if(length > DXL_BUF_LENGTH){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  packet_length = length + 3; // param_length + Instruction + CRC_L + CRC_H

  p_packet->tx.data[PKT_HDR_1_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_2_IDX] = 0xFF;
  p_packet->tx.data[PKT_HDR_3_IDX] = 0xFD;
  p_packet->tx.data[PKT_RSV_IDX]   = 0x00;
  p_packet->tx.data[PKT_ID_IDX]    = id;
  p_packet->tx.data[PKT_INST_IDX]  = inst_cmd;

  for (i=0; i<length; i++)
  {
    p_packet->tx.data[PKT_INST_PARAM_IDX + i] = p_data[i];
  }

  stuff_length = dxlAddStuffing(p_packet, &p_packet->tx.data[PKT_INST_IDX], length + 1);  // + instruction
  packet_length += stuff_length;

  p_packet->tx.data[PKT_LEN_L_IDX] = packet_length >> 0;
  p_packet->tx.data[PKT_LEN_H_IDX] = packet_length >> 8;

  crc = 0;
  for (i=0; i<packet_length+7-2; i++)
  {
    dxlUpdateCrc(&crc, p_packet->tx.data[i]);
  }

  p_packet->tx.data[PKT_INST_IDX + packet_length - 2] = crc >> 0;
  p_packet->tx.data[PKT_INST_IDX + packet_length - 1] = crc >> 8;

  dxlTxWrite(p_packet->tx.data, packet_length + 7);

  return ret;
}


#if (USE_SLAVE_FUNC)
lib_err_code_t dxlInstPing(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;

  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 ){
    ret = dxlInstPing1_0(p_dxl);
  }else{
    ret = dxlInstPing2_0(p_dxl);
  }

  return ret;
}

lib_err_code_t dxlInstPing1_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t length = 0;


  if (p_dxl->process_func.processPing != nullptr){
    (*p_dxl->process_func.processPing)(p_dxl->tx.data, &length);
  }

  if (p_dxl->rx.id == DXL_BROADCAST_ID){
    ret = dxlMakePacketStatus(p_dxl, p_dxl->id, 0, p_dxl->tx.data, length);

    if (ret == DXL_LIB_OK){
      ret = DXL_RET_PROCESS_BROAD_PING;
    }
  }else{
    ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, p_dxl->tx.data, length);
  }


  return ret;
}

lib_err_code_t dxlInstPing2_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t length = 0;

  if (p_dxl->process_func.processPing != nullptr){
    (*p_dxl->process_func.processPing)(p_dxl->tx.data, &length);
  }

  if (p_dxl->rx.id == DXL_BROADCAST_ID){
    ret = dxlMakePacketStatus(p_dxl, p_dxl->id, 0, p_dxl->tx.data, length);

    if (ret == DXL_LIB_OK) {
      ret = DXL_RET_PROCESS_BROAD_PING;
    }
  }else{
    ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, p_dxl->tx.data, length);
  }


  return ret;
}

lib_err_code_t dxlInstFactoryReset(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;

  if (p_dxl->rx.id != DXL_BROADCAST_ID) {
    ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, nullptr, 0);
  }

  if (p_dxl->process_func.processFactoryReset != nullptr) {
    delay(100);
    (*p_dxl->process_func.processFactoryReset)(p_dxl->rx.p_param[0]);
  }

  return ret;
}

lib_err_code_t dxlInstReboot(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;

  if (p_dxl->rx.id != DXL_BROADCAST_ID){
    ret = dxlTxPacketStatus(p_dxl, p_dxl->id, 0, nullptr, 0);
  }


  if (p_dxl->process_func.processReboot != nullptr) {
    delay(100);
    (*p_dxl->process_func.processReboot)();
  }

  return ret;
}

lib_err_code_t dxlInstRead(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;


  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 )
  {
    ret = dxlInstRead1_0(p_dxl);
  }
  else
  {
    ret = dxlInstRead2_0(p_dxl);
  }

  return ret;
}

lib_err_code_t dxlInstRead1_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t process_ret = 0;

  if (p_dxl->rx.id == DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  
  if( p_dxl->rx.param_length != 2)
    return DXL_LIB_ERROR_LENGTH;

  addr   = p_dxl->rx.p_param[0];
  length = p_dxl->rx.p_param[1];

  if( length > 0xFF - 2 ){
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_LENGTH;
  }

  if (p_dxl->process_func.processRead != nullptr){
    process_ret = (*p_dxl->process_func.processRead)(addr, p_dxl->tx.data, length);
  }

  ret = dxlTxPacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);

  return ret;
}

lib_err_code_t dxlInstRead2_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length = 0;

  uint8_t process_ret = 0;

  if (p_dxl->rx.id == DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  
  if( p_dxl->rx.param_length != 4)
    return DXL_LIB_ERROR_LENGTH;

  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];

  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  if (p_dxl->process_func.processRead != nullptr){
    process_ret = (*p_dxl->process_func.processRead)(addr, p_dxl->tx.data, length);
  }

  ret = dxlTxPacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);

  return ret;
}

lib_err_code_t dxlInstWrite(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;

  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 ){
    ret = dxlInstWrite1_0(p_dxl);
  }else{
    ret = dxlInstWrite2_0(p_dxl);
  }

  return ret;
}

lib_err_code_t dxlInstWrite1_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t  *p_data;

  if (p_dxl->rx.id == DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;

  addr   =  p_dxl->rx.p_param[0];
  p_data = &p_dxl->rx.p_param[1];

  if (p_dxl->rx.param_length > 1 ){
    length = p_dxl->rx.param_length - 1;
  }else{
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_LENGTH;
  }

  if( length > 0xFF - 2 ){
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_LENGTH;
  }

  // TODO : Write address validation required before sending Write response
  dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_NONE, nullptr, 0);

  if (p_dxl->process_func.processWrite != nullptr){
    (*p_dxl->process_func.processWrite)(addr, p_data, length);
  }

  return ret;
}

lib_err_code_t dxlInstWrite2_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length = 0;
  uint8_t  *p_data;
  //uint8_t process_ret;

  if (p_dxl->rx.id == DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;

  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  p_data = &p_dxl->rx.p_param[2];

  if (p_dxl->rx.param_length > 2 ){
    length = p_dxl->rx.param_length - 2;
  }else{
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_LENGTH;
  }
  
  if(length > DXL_BUF_LENGTH){
    dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_DATA_LENGTH, nullptr, 0);
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  // TODO : Write address validation required before sending Write response
  dxlTxPacketStatus(p_dxl, p_dxl->id, DXL_ERR_NONE, nullptr, 0);

  if (p_dxl->process_func.processWrite != nullptr){
    (*p_dxl->process_func.processWrite)(addr, p_data, length);
  }

  return ret;
}

lib_err_code_t dxlInstSyncRead(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t i;
  uint16_t rx_id_cnt;
  uint8_t process_ret = DXL_ERR_NONE;


  if (p_dxl->rx.id != DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_INVAILD_ID;

  rx_id_cnt = p_dxl->rx.param_length - 4;

  if (p_dxl->rx.param_length < (5) || rx_id_cnt > 0xFF)
    return DXL_LIB_ERROR_LENGTH;

  addr      = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length    = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];
  p_data    = &p_dxl->rx.p_param[4];

  p_dxl->pre_id     = 0xFF;
  p_dxl->present_id = 0xFF;

  for (i=0; i<rx_id_cnt; i++)
  {
    if (p_data[i] == p_dxl->id) {
      p_dxl->present_id = p_dxl->id;
      break;
    }

    p_dxl->pre_id = p_data[i];
  }

  if (p_dxl->present_id == p_dxl->id){
    if (p_dxl->process_func.processRead != nullptr){
      process_ret = (*p_dxl->process_func.processRead)(addr, p_dxl->tx.data, length);
    }

    if (p_dxl->pre_id == 0xFF){
      ret = dxlTxPacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);
    }else{
      ret = dxlMakePacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);
      if (ret == DXL_LIB_OK) {
        ret = DXL_RET_PROCESS_BROAD_READ;
      }
    }
  }

  return ret;
}

lib_err_code_t dxlInstSyncWrite(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;


  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 )
  {
    ret = dxlInstSyncWrite1_0(p_dxl);
  }
  else
  {
    ret = dxlInstSyncWrite2_0(p_dxl);
  }

  return ret;
}

lib_err_code_t dxlInstSyncWrite1_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t remain_length;
  uint16_t index;

  if (p_dxl->rx.id != DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_INVAILD_ID;

  addr   = p_dxl->rx.p_param[0];
  length = p_dxl->rx.p_param[1];

  if (p_dxl->rx.param_length < (2+length+1))
    return DXL_LIB_ERROR_LENGTH;

  index = 2;
  while(1)
  {
    p_data = &p_dxl->rx.p_param[index];
    remain_length = p_dxl->rx.param_length - index;


    if (remain_length < (length+1))
    {
      break;
    }
    else
    {
      if (p_data[0] == p_dxl->id)
      {
        if (p_dxl->process_func.processWrite != nullptr)
        {
          (*p_dxl->process_func.processWrite)(addr, &p_data[1], length);
        }
        break;
      }

      index += length + 1;
    }
  }

  return ret;
}

lib_err_code_t dxlInstSyncWrite2_0(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t remain_length;
  uint16_t index;

  if (p_dxl->rx.id != DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_INVAILD_ID;

  addr   = (p_dxl->rx.p_param[1]<<8) | p_dxl->rx.p_param[0];
  length = (p_dxl->rx.p_param[3]<<8) | p_dxl->rx.p_param[2];

  if (p_dxl->rx.param_length < (4+length+1))
    return DXL_LIB_ERROR_LENGTH;

  index = 4;
  while(1)
  {
    p_data = &p_dxl->rx.p_param[index];
    remain_length = p_dxl->rx.param_length - index;

    if (remain_length < (length+1))
    {
      break;
    }
    else
    {
      if (p_data[0] == p_dxl->id)
      {
        if (p_dxl->process_func.processWrite != nullptr)
        {
          (*p_dxl->process_func.processWrite)(addr, &p_data[1], length);
        }
        break;
      }

      index += length + 1;
    }
  }

  return ret;
}

lib_err_code_t dxlInstBulkRead(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t i;
  uint16_t rx_id_cnt;
  uint8_t process_ret = DXL_ERR_NONE;


  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 )
    return DXL_LIB_ERROR_NOT_SUPPORTED;

  if (p_dxl->rx.id != DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_INVAILD_ID;

  rx_id_cnt = p_dxl->rx.param_length / 5;

  if (p_dxl->rx.param_length < 5 || (p_dxl->rx.param_length%5) != 0)
    return DXL_LIB_ERROR_LENGTH;

  p_dxl->pre_id     = 0xFF;
  p_dxl->present_id = 0xFF;

  addr   = 0;
  length = 0;
  for (i=0; i<rx_id_cnt; i++)
  {
    p_data = &p_dxl->rx.p_param[i*5];
    addr   = (p_data[2]<<8) | p_data[1];
    length = (p_data[4]<<8) | p_data[3];


    if (p_data[0] == p_dxl->id)
    {
      p_dxl->present_id = p_dxl->id;
      break;
    }
    p_dxl->pre_id = p_data[0];
  }


  if (p_dxl->present_id == p_dxl->id)
  {
    if (p_dxl->process_func.processRead != nullptr)
    {
      process_ret = (*p_dxl->process_func.processRead)(addr, p_dxl->tx.data, length);
    }

    if (p_dxl->pre_id == 0xFF)
    {
      ret = dxlTxPacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);
    }
    else
    {
      ret = dxlMakePacketStatus(p_dxl, p_dxl->id, process_ret, p_dxl->tx.data, length);
      if (ret == DXL_LIB_OK)
      {
        ret = DXL_RET_PROCESS_BROAD_READ;
      }
    }
  }

  return ret;
}

lib_err_code_t dxlInstBulkWrite(dxl_t *p_dxl)
{
  lib_err_code_t ret = DXL_LIB_OK;
  uint16_t addr;
  uint16_t length;
  uint8_t  *p_data;
  uint16_t index;
  uint8_t  id;

  if (p_dxl->packet_ver == DXL_PACKET_VER_1_0 )
    return DXL_LIB_ERROR_NOT_SUPPORTED;

  if (p_dxl->rx.id != DXL_BROADCAST_ID)
    return DXL_LIB_ERROR_INVAILD_ID;

  index = 0;
  while(1)
  {
    p_data = &p_dxl->rx.p_param[index];
    id     = p_data[0];
    addr   = (p_data[2]<<8) | p_data[1];
    length = (p_data[4]<<8) | p_data[3];

    index += 5;

    if (p_dxl->rx.param_length < (index + length))
    {
      break;
    }

    if (id == p_dxl->id)
    {
      if (p_dxl->process_func.processWrite != nullptr)
      {
        (*p_dxl->process_func.processWrite)(addr, &p_dxl->rx.p_param[index], length);
      }
      break;
    }
    index += length;
  }

  return ret;
}
#endif

const unsigned short crc_table[256] PROGMEM  = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

void dxlUpdateCrc(uint16_t *p_crc_cur, uint8_t data_in)
{
  uint16_t crc;
  uint16_t i;

  crc = *p_crc_cur;

  i = ((unsigned short)(crc >> 8) ^ data_in) & 0xFF;
  *p_crc_cur = (crc << 8) ^ pgm_read_word_near(crc_table + i);
}