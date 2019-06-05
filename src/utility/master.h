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

#ifndef DYNAMIXEL_MASTER_H_
#define DYNAMIXEL_MASTER_H_


#include "packet_handler.h"
#include "protocol.h"

#define DXLCMD_MAX_NODE               DXL_MAX_NODE
#define DXLCMD_MAX_NODE_BUFFER_SIZE   DXL_MAX_NODE_BUFFER_SIZE
#define DXLCMD_MAX_BUFFER             DXL_BUF_LENGTH

namespace DYNAMIXEL {

typedef struct
{
  uint8_t   id;
  uint8_t   error;
  uint16_t  model_number;
  uint8_t   firmware_version;
} ping_node_t;

typedef struct
{
  uint8_t  id;
  uint8_t  error;
  uint16_t length;
  uint8_t  *p_data;
} read_node_t;

typedef struct
{
  uint8_t        id_count;
  read_node_t   *p_node[DXLCMD_MAX_NODE];
  uint32_t       mem[DXLCMD_MAX_BUFFER/4];
} status_read_t;

typedef struct
{
  uint8_t        id_count;
  ping_node_t   *p_node[DXLCMD_MAX_NODE];
  uint32_t       mem[DXLCMD_MAX_BUFFER/4];
} status_ping_t;

//-- SyncRead
typedef struct
{
  uint8_t   id_count;
  uint8_t   id_tbl[DXLCMD_MAX_NODE];
  uint16_t  addr;
  uint8_t   length;
} param_sync_read_t;

//-- BulkRead
typedef struct
{
  uint8_t   id_count;
  uint8_t   id_tbl[DXLCMD_MAX_NODE];
  uint16_t  addr  [DXLCMD_MAX_NODE];
  uint8_t   length[DXLCMD_MAX_NODE];
} param_bulk_read_t;

//-- SyncWrite
typedef struct
{
  uint8_t  id;
  uint8_t  data[DXLCMD_MAX_NODE_BUFFER_SIZE];
} sync_write_node_t;

typedef struct
{
  uint8_t  id_count;
  uint16_t addr;
  uint8_t  length;
  sync_write_node_t node[DXLCMD_MAX_NODE];
} param_sync_write_t;

//-- BulkWrite
typedef struct
{
  uint8_t  id;
  uint16_t addr;
  uint8_t  length;
  uint8_t  data[DXLCMD_MAX_NODE_BUFFER_SIZE];
} bulk_write_node_t;

typedef struct
{
  uint8_t   id_count;
  bulk_write_node_t node[DXLCMD_MAX_NODE];
} param_bulk_write_t;

typedef union
{
  param_sync_read_t  sync_read;
  param_bulk_read_t  bulk_read;
  param_sync_write_t sync_write;
  param_bulk_write_t bulk_write;
} param_t;

typedef union
{
  status_ping_t    ping;
  status_read_t    read;
  status_read_t    sync_read;
  status_read_t    bulk_read;
} status_t;

typedef union
{
  param_t  param;
  status_t   resp;
} param_resp_t;


class Master
{
  public:
    Master(PortHandler &port, float protocol_ver = 2.0);
    Master(float protocol_ver = 2.0);

    bool setPortProtocolVersion(float version);
    float getPortProtocolVersion();

    bool setPort(PortHandler &port);

    bool ping(uint8_t id,
     status_ping_t *p_resp, uint32_t timeout);
    //bool pingBroadcast();
          
    int32_t read(uint8_t id, uint16_t addr, uint16_t addr_length,
      uint8_t *p_recv_buf, uint16_t recv_buf_length, uint32_t timeout);

    bool write(uint8_t id, uint16_t addr, 
      uint8_t *p_data, uint16_t data_length, uint32_t timeout = 100);

    bool writeNoResp(uint8_t id, uint16_t addr, 
      uint8_t *p_data, uint16_t data_length);

    //TODO: bool regWrite();
    //TODO: bool action();
    bool factoryReset(uint8_t id, uint8_t option, uint32_t timeout = 100);
    bool reboot(uint8_t id, uint32_t timeout);

    //TODO: bool clear();
    bool syncWrite(uint16_t addr, uint16_t addr_len,
      uint8_t *id_list, uint8_t id_cnt, 
      uint8_t *data_list, uint16_t data_list_size);

    int32_t syncRead(uint16_t addr, uint16_t addr_len,
      uint8_t *id_list, uint8_t id_cnt, 
      uint8_t *recv_buf, uint16_t recv_buf_size, uint32_t timeout = 100);

#if 1 //TODO
    bool syncRead(param_sync_read_t *p_param, status_read_t *p_resp, uint32_t timeout = 100);
    bool syncWrite(param_sync_write_t *p_param);

    bool bulkRead(param_bulk_read_t *p_param, status_read_t *p_resp, uint32_t timeout = 100);
    bool bulkWrite(param_bulk_write_t *p_param);
#endif

    uint8_t getLastStatusPacketError() const;
    lib_err_code_t getLastLibErrCode() const;

    void setLastLibErrCode(lib_err_code_t err_code);

  private:
    PortHandler *p_port_;
    //PacketHandler packet_handler_;
    dxl_t packet_;
    uint8_t last_status_packet_error_; 
    lib_err_code_t last_lib_err_code_;
  };
}


#endif /* DYNAMIXEL_MASTER_H_ */