/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

#ifndef DYNAMIXEL_MASTER_HPP_
#define DYNAMIXEL_MASTER_HPP_


#include "dxl_c/protocol.h"
#include "port_handler.h"
#include "config.h"


#define UNREGISTERED_MODEL  (uint16_t)0xFFFF
#define COMMON_MODEL_NUMBER_ADDR         0
#define COMMON_MODEL_NUMBER_ADDR_LENGTH  2


namespace DYNAMIXEL {

typedef struct InfoFromPing{
  uint8_t id;
  uint16_t model_number;
  uint8_t firmware_version;
} InfoFromPing_t;

typedef struct InfoSyncBulkBuffer{
  uint8_t* p_buf;
  uint16_t buf_capacity;
  uint16_t gen_length;
  bool is_completed;
} __attribute__((packed)) InfoSyncBulkBuffer_t;

/* Sync Instructions */
typedef struct XELInfoSyncRead{
  uint8_t *p_recv_buf;
  uint8_t id;
  uint8_t error;
} __attribute__((packed)) XELInfoSyncRead_t;

typedef struct InfoSyncReadInst{
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncRead_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed)) InfoSyncReadInst_t;

typedef struct XELInfoSyncWrite{
  uint8_t* p_data;
  uint8_t id;
} __attribute__((packed)) XELInfoSyncWrite_t;

typedef struct InfoSyncWriteInst{
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncWrite_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed)) InfoSyncWriteInst_t;

/* Bulk Instructions */
typedef struct XELInfoBulkRead{
  uint16_t addr;
  uint16_t addr_length;
  uint8_t *p_recv_buf;
  uint8_t id;
  uint8_t error;
} __attribute__((packed)) XELInfoBulkRead_t;

typedef struct InfoBulkReadInst{
  XELInfoBulkRead_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed)) InfoBulkReadInst_t;

typedef struct XELInfoBulkWrite{
  uint16_t addr;
  uint16_t addr_length;
  uint8_t* p_data;
  uint8_t id;
} __attribute__((packed)) XELInfoBulkWrite_t;

typedef struct InfoBulkWriteInst{
  XELInfoBulkWrite_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed)) InfoBulkWriteInst_t;



class Master
{
  public:
    /**
     * @brief The constructor.
     * @code
     * const int DXL_DIR_PIN = 2;
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::SerialPortHandler dxl_port(Serial1, DXL_DIR_PIN);
     * DYNAMIXEL::Master dxl_master(dxl_port, PROTOCOL_VER);
     * @endcode
     * @param port The DXLPortHandler instance you want to use on the board to communicate with DYNAMIXELs.
     *             It can be used not only for Serial but also for other communication port handlers like SerialPortHandler class.
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)
     */
    Master(DXLPortHandler &port, float protocol_ver = 2.0, uint16_t malloc_buf_size = 256);

    /**
     * @brief The constructor.
     *        This constructor must be added to the PortHanlder instance via the @setPort () function after creation.
     * @code
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::Master dxl_master(PROTOCOL_VER);
     * @endcode
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)        
     */    
    Master(float protocol_ver = 2.0, uint16_t malloc_buf_size = 256);

    bool setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity);
    uint8_t* getPacketBuffer() const;
    uint16_t getPacketBufferCapacity() const;

    bool setPortProtocolVersion(float version);
    bool setPortProtocolVersionUsingIndex(uint8_t version_idx);
    float getPortProtocolVersion() const;

    bool setPort(DXLPortHandler &port);
    bool setPort(DXLPortHandler *p_port);
    DXLPortHandler* getPort() const;

    /* Instructions */
    uint8_t ping(uint8_t id, uint8_t *p_recv_id_array, uint8_t recv_array_capacity,
      uint32_t timeout_ms = 10);
    uint8_t ping(uint8_t id, InfoFromPing_t *recv_ping_info_array, uint8_t recv_array_cnt,
      uint32_t timeout_ms = 10);
    int32_t read(uint8_t id, uint16_t addr, uint16_t addr_length,
      uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout_ms = 10);
    bool write(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms = 10);      
    bool writeNoResp(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length);
    bool regWrite(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms = 10);
    bool action(uint8_t id, uint32_t timeout_ms = 10);      
    bool factoryReset(uint8_t id, uint8_t option, uint32_t timeout_ms = 10);
    bool reboot(uint8_t id, uint32_t timeout_ms = 10);
    bool clear(uint8_t id, uint8_t option, uint32_t ex_option, uint32_t timeout_ms = 10);
    uint8_t syncRead(InfoSyncReadInst_t* p_info, uint32_t timeout_ms = 10);
    bool syncWrite(InfoSyncWriteInst_t* p_info);
    uint8_t bulkRead(InfoBulkReadInst_t* p_info, uint32_t timeout_ms = 10);
    bool bulkWrite(InfoBulkWriteInst_t* p_info);

    uint8_t getLastStatusPacketError() const;
    
    void setLastLibErrCode(DXLLibErrorCode_t err_code);
    DXLLibErrorCode_t getLastLibErrCode() const;

    // raw APIs
    bool txInstPacket(uint8_t id, uint8_t inst_idx, uint8_t *p_param, uint16_t param_len);
    const InfoToParseDXLPacket_t* rxStatusPacket(uint8_t* p_param_buf, uint16_t param_buf_cap, uint32_t timeout_ms = 10);

  private:
    DXLPortHandler *p_port_;

    uint8_t protocol_ver_idx_;

    bool is_buf_malloced_;
    uint8_t *p_packet_buf_;
    uint16_t packet_buf_capacity_;
    InfoToMakeDXLPacket_t info_tx_packet_;
    InfoToParseDXLPacket_t info_rx_packet_;

    DXLLibErrorCode_t last_lib_err_;
};



template <uint16_t START_ADDR,
          typename T,
          uint8_t XEL_CNT_MAX = 16>
class SyncWrite
{
  public:
    SyncWrite(Master &dxl_master)
    : dxl_master_(dxl_master)
    {
      setPacketBuffer(dxl_master_.getPacketBuffer(), dxl_master_.getPacketBufferCapacity());
      memset(&sw_info_, 0, sizeof(sw_info_));
      memset(xel_infos_, 0, sizeof(xel_infos_));
      sw_info_.addr = START_ADDR;
      sw_info_.addr_length = sizeof(T);
      sw_info_.p_xels = xel_infos_;
    }

    bool setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
    {
      bool ret = false;
      if(p_buf != nullptr && buf_capacity > 0){
        sw_info_.packet.p_buf = p_buf;
        sw_info_.packet.buf_capacity = buf_capacity;
        ret = true;
      }
      return ret;
    }

    uint8_t* getPacketBuffer() const
    {
      return sw_info_.packet.p_buf;
    }

    bool addParam(uint8_t id, T &data)
    {
      bool ret = false;
      uint8_t i;

      for(i=0; i<sw_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          break;
        }
      }
      if(i == sw_info_.xel_count && i < XEL_CNT_MAX){
        xel_infos_[sw_info_.xel_count].id = id;
        xel_infos_[sw_info_.xel_count++].p_data = (uint8_t*)&data;
        sw_info_.is_info_changed = true;
        ret = true;
      }

      return ret;
    }

    bool changeParam(uint8_t id, T &data)
    {
      bool ret = false;
      uint8_t i;

      for(i=0; i<sw_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          xel_infos_[i].p_data = (uint8_t*)&data;
          sw_info_.is_info_changed = true;
          ret = true;
          break;
        }
      }
      
      return ret;
    }

    void updateParamData(){
      sw_info_.is_info_changed = true;
    }

    bool sendPacket(){
      return dxl_master_.syncWrite(&sw_info_);
    }

    uint16_t getStartAddr() const
    {
      return sw_info_.addr;
    }

    uint16_t getAddrLength() const
    {
      return sw_info_.addr_length;
    }

    uint8_t getIDByIndex(uint8_t index) const
    {
      if(index >= XEL_CNT_MAX){
        return 0xFF;
      }
      return xel_infos_[index].id;
    }

    T* getDataPtr(uint8_t id)
    {
      T* p_ret = nullptr;
      uint8_t i;

      for(i=0; i<sw_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          p_ret = (T*)xel_infos_[i].p_data;
          break;
        }
      }

      return p_ret;
    }

  private:
    Master &dxl_master_;
    InfoSyncWriteInst_t sw_info_;
    XELInfoSyncWrite_t xel_infos_[XEL_CNT_MAX];
};


template <uint16_t START_ADDR,
          typename T,
          uint8_t XEL_CNT_MAX = 16>
class SyncRead
{
  public:
    SyncRead(Master &dxl_master)
    : dxl_master_(dxl_master)
    {
      setPacketBuffer(dxl_master_.getPacketBuffer(), dxl_master_.getPacketBufferCapacity());
      memset(&sr_info_, 0, sizeof(sr_info_));
      memset(xel_infos_, 0, sizeof(xel_infos_));
      sr_info_.addr = START_ADDR;
      sr_info_.addr_length = sizeof(T);
      sr_info_.p_xels = xel_infos_;
    }

    bool setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
    {
      bool ret = false;
      if(p_buf != nullptr && buf_capacity > 0){
        sr_info_.packet.p_buf = p_buf;
        sr_info_.packet.buf_capacity = buf_capacity;
        ret = true;
      }
      return ret;
    }

    uint8_t* getPacketBuffer() const
    {
      return sr_info_.packet.p_buf;
    }

    bool addParam(uint8_t id, T &recv_data)
    {
      bool ret = false;
      uint8_t i;

      for(i=0; i<sr_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          break;
        }
      }
      if(i == sr_info_.xel_count && i < XEL_CNT_MAX){
        xel_infos_[sr_info_.xel_count].id = id;
        xel_infos_[sr_info_.xel_count++].p_recv_buf = (uint8_t*)&recv_data;
        sr_info_.is_info_changed = true;
        ret = true;
      }

      return ret;
    }

    bool changeParam(uint8_t id, T &recv_data)
    {
      bool ret = false;
      uint8_t i;

      for(i=0; i<sr_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          xel_infos_[i].p_recv_buf = (uint8_t*)&recv_data;
          sr_info_.is_info_changed = true;
          ret = true;
          break;
        }
      }
      
      return ret;
    }

    uint8_t sendPacket()
    {
      return dxl_master_.syncRead(&sr_info_);
    }

    uint16_t getStartAddr() const
    {
      return sr_info_.addr;
    }

    uint16_t getAddrLength() const
    {
      return sr_info_.addr_length;
    }

    uint8_t getIDByIndex(uint8_t index) const
    {
      if(index >= XEL_CNT_MAX){
        return 0xFF;
      }
      return xel_infos_[index].id;
    }

    uint8_t getError(uint8_t id) const
    {
      uint8_t i, ret = 0;

      for(i=0; i<sr_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          ret = xel_infos_[i].error;
          break;
        }
      }

      return ret;
    }

    T* getDataPtr(uint8_t id)
    {
      T* p_ret = nullptr;
      uint8_t i;

      for(i=0; i<sr_info_.xel_count; i++){
        if(xel_infos_[i].id == id){
          p_ret = (T*)xel_infos_[i].p_recv_buf;
          break;
        }
      }

      return p_ret;
    }

  private:
    Master &dxl_master_;
    InfoSyncReadInst_t sr_info_;
    XELInfoSyncRead_t xel_infos_[XEL_CNT_MAX];
};


//TODO
class BulkWriteMaster
{

};

class BulkReadMaster
{

};


} //namespace DYNAMIXEL





//legacy
typedef DYNAMIXEL::InfoFromPing_t XelInfoFromPing_t;


#endif /* DYNAMIXEL_MASTER_HPP_ */