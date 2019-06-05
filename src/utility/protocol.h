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
#ifndef DYNAMIXEL_PROTOCOL_H_
#define DYNAMIXEL_PROTOCOL_H_

#include <Arduino.h>
#include "port_handler.h"
#include "config.h"

#define DXL_PACKET_VER_1_0      ((float)(1.0))
#define DXL_PACKET_VER_2_0      ((float)(2.0))

#define DXL_STATE_WAIT_INST     0
#define DXL_STATE_WAIT_STATUS   1

#define DXL_TYPE_INST           0
#define DXL_TYPE_STATUS         1

#define DXL_MODE_SLAVE          0
#define DXL_MODE_MASTER         1

#define DXL_BROADCAST_ID        0xFE
#define DXL_ALL_ID              DXL_BROADCAST_ID
#define DXL_NONE_ID             0xFF

#define DXL_MAX_BUFFER          DXL_BUF_LENGTH


#define UNREGISTERED_MODEL  (uint16_t)0xFFFF
#define COMMON_MODEL_NUMBER_ADDR         0
#define COMMON_MODEL_NUMBER_ADDR_LENGTH  2


//-- 2.0 Protocol
// 
#define PKT_HDR_1_IDX           0
#define PKT_HDR_2_IDX           1
#define PKT_HDR_3_IDX           2
#define PKT_RSV_IDX             3
#define PKT_ID_IDX              4
#define PKT_LEN_L_IDX           5
#define PKT_LEN_H_IDX           6
#define PKT_INST_IDX            7
#define PKT_ERROR_IDX           8

#define PKT_INST_PARAM_IDX      8
#define PKT_STATUS_PARAM_IDX    9


//-- 1.0 Protocol
//
#define PKT_1_0_HDR_1_IDX         0
#define PKT_1_0_HDR_2_IDX         1
#define PKT_1_0_ID_IDX            2
#define PKT_1_0_LEN_IDX           3
#define PKT_1_0_INST_IDX          4
#define PKT_1_0_ERROR_IDX         4

#define PKT_1_0_INST_PARAM_IDX    5
#define PKT_1_0_STATUS_PARAM_IDX  5

#define DXL_ERR_NONE            0x00
#define DXL_ERR_RESULT_FAIL     0x01
#define DXL_ERR_INST_ERROR      0x02
#define DXL_ERR_CRC_ERROR       0x03
#define DXL_ERR_DATA_RANGE      0x04
#define DXL_ERR_DATA_LENGTH     0x05
#define DXL_ERR_DATA_LIMIT      0x06
#define DXL_ERR_ACCESS          0x07

#define DXL_PROCESS_INST        0
#define DXL_PROCESS_BROAD_PING  1
#define DXL_PROCESS_BROAD_READ  2
#define DXL_PROCESS_BROAD_WRITE 3

#define DXL_BYPASS_NONE         0
#define DXL_BYPASS_ENABLE       1
#define DXL_BYPASS_ONLY         2

#define RX_PACKET_TYPE_STATUS   0
#define RX_PACKET_TYPE_INST     1

namespace DYNAMIXEL{

  enum Instruction{
    INST_PING = 0x01,
    INST_READ = 0x02,
    INST_WRITE = 0x03,
    INST_REG_WRITE = 0x04,
    INST_ACTION = 0x05,
    INST_RESET = 0x06,
    INST_REBOOT = 0x08,
    INST_STATUS = 0x55,
    INST_SYNC_READ = 0x82,
    INST_SYNC_WRITE = 0x83,
    INST_BULK_READ = 0x92,
    INST_BULK_WRITE = 0x93
  };

  typedef enum LibErrorCode
  {
    DXL_LIB_OK = 0,
    DXL_LIB_PROCEEDING,

    DXL_LIB_ERROR_NOT_SUPPORTED,
    DXL_LIB_ERROR_TIMEOUT,
    DXL_LIB_ERROR_INVAILD_ID,
    DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST,
    DXL_LIB_ERROR_NULLPTR,
    DXL_LIB_ERROR_LENGTH,
    DXL_LIB_ERROR_ADDR_LENGTH,
    DXL_LIB_ERROR_BUFFER_OVERFLOW,
    DXL_LIB_ERROR_PORT_NOT_OPEN,
    DXL_LIB_ERROR_CHECK_SUM,
    DXL_LIB_ERROR_CRC
  } lib_err_code_t;

  typedef struct
  {
    uint8_t   header[3];
    uint8_t   reserved;
    uint8_t   id;
    uint8_t   cmd;
    uint8_t   error;
    uint8_t   type;
    uint16_t  index;
    uint16_t  packet_length;
    uint16_t  param_length;
    uint16_t  crc;
    uint16_t  crc_received;
    uint8_t   check_sum;
    uint8_t   check_sum_received;
    uint8_t   *p_param;
    uint32_t  dummy;
    uint8_t   data[DXL_MAX_BUFFER];
  } dxl_packet_t;

  typedef lib_err_code_t (*dxlInstFunc_t)(void *);

  typedef struct
  {
    dxlInstFunc_t ping;         
    dxlInstFunc_t read;         
    dxlInstFunc_t write;        
    dxlInstFunc_t reg_write;    
    dxlInstFunc_t action;       
    dxlInstFunc_t factory_reset;
    dxlInstFunc_t reboot;       
    dxlInstFunc_t status;       
    dxlInstFunc_t sync_read;    
    dxlInstFunc_t sync_write;   
    dxlInstFunc_t bulk_read;   
    dxlInstFunc_t bulk_write;   
  } dxl_inst_func_t;

  typedef struct
  {
    uint8_t (*processPing         )(uint8_t *p_data, uint16_t *p_length);
    uint8_t (*processReboot       )(void);
    uint8_t (*processFactoryReset )(uint8_t mode);
    uint8_t (*processRead         )(uint16_t addr, uint8_t *p_data, uint16_t length);
    uint8_t (*processWrite        )(uint16_t addr, uint8_t *p_data, uint16_t length);
  } dxl_process_func_t;

  typedef struct Dxl
  {
    float  packet_ver;
    uint8_t  dxl_mode;

    uint8_t  rx_pakcet_type;
    uint8_t  rx_state;
    uint8_t  id;
    uint8_t  present_id;
    uint8_t  pre_id;

    uint8_t  process_state;
    uint32_t process_pre_time;

    uint32_t rx_timeout;
    uint32_t tx_timeout;
    uint32_t prev_time;
    uint32_t tx_time;
    uint32_t rx_time;
    uint8_t  header_cnt;

#if (USE_SLAVE_FUNC)
    dxl_inst_func_t    inst_func;
    dxl_process_func_t process_func;
#endif
    void (*read_hook_func)(uint8_t data);

    dxl_packet_t    rx;
    dxl_packet_t    tx;
  } dxl_t;

  typedef struct
  {
    bool     use;
    uint8_t  ch;
    uint8_t  id;
    uint32_t baud;
    uint32_t rx_timeout;
    dxl_t    node;
  } dxl_node_t;

  bool setDxlPort(PortHandler *port);
  bool dxlInit(dxl_t *p_packet, float protocol_ver);
  void dxlSetTxDoneISR(dxl_t *p_packet, void (*p_txDoneISR)(void));
  void dxlSetRxByteISR(dxl_t *p_packet, void (*p_func)(uint8_t data));

  void dxlSetDirPin(dxl_t *p_packet, int8_t dir_pin);
  void dxlSetSerial(dxl_t *p_packet, HardwareSerial *p_serial);

#if (USE_SLAVE_FUNC)
  void dxlAddInstFunc(dxl_t *p_packet, uint8_t inst, dxlInstFunc_t func);
  void dxlAddProcessPingFunc(dxl_t *p_packet, uint8_t (*func)(uint8_t *p_data, uint16_t *p_length));
  void dxlAddProcessRebootFunc(dxl_t *p_packet, uint8_t (*func)(void));
  void dxlAddProcessFactoryResetFunc(dxl_t *p_packet, uint8_t (*func)(uint8_t mode));
  void dxlAddProcessReadFunc(dxl_t *p_packet, uint8_t (*func)(uint16_t addr, uint8_t *p_data, uint16_t length));
  void dxlAddProcessWriteFunc(dxl_t *p_packet, uint8_t (*func)(uint16_t addr, uint8_t *p_data, uint16_t length));
#endif

  bool    dxlSetId(dxl_t *p_packet, uint8_t id);
  uint8_t dxlGetId(dxl_t *p_packet);

  bool    dxlSetProtocolVersion(dxl_t *p_packet, float protocol_version);
  float dxlGetProtocolVersion(dxl_t *p_packet);

  uint32_t dxlRxAvailable();
  uint8_t  dxlRxRead();
  void     dxlTxWrite(uint8_t *p_data, uint32_t length);

#if (USE_SLAVE_FUNC)
  uint8_t     dxlProcessPacket(dxl_t *p_packet);
  lib_err_code_t dxlProcessInst(dxl_t *p_packet);
#endif

  lib_err_code_t dxlRxPacket(dxl_t *p_packet);
  lib_err_code_t dxlRxPacketDataIn(dxl_t *p_packet, uint8_t data_in);

  lib_err_code_t dxlTxPacket(dxl_t *p_packet);
  lib_err_code_t dxlTxPacketInst(dxl_t *p_packet, uint8_t id, uint8_t inst_cmd, uint8_t *p_data, uint16_t length);
  lib_err_code_t dxlTxPacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length);

  lib_err_code_t dxlMakePacketStatus(dxl_t *p_packet, uint8_t id, uint8_t error, uint8_t *p_data, uint16_t length );

}

#endif /* DYNAMIXEL_PROTOCOL_H_ */