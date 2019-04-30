
#include "actuator.h"
#include "Arduino.h"



#if defined(__arm__) && !defined(PROGMEM)
#define PROGMEM
#define PSTR(STR) STR
#else
#include <avr/pgmspace.h>
#endif


using namespace DYNAMIXEL;

typedef struct ModelAvailableFunc{
  bool torque_on_off;
  bool led_ctrl;
  bool velocity_ctrl;
  bool position_ctrl;
  bool ext_pos_ctrl;
  bool pwm_ctrl;
  bool current_ctrl;
  bool current_based_pos_ctrl;
  bool direction_ctrl;
  bool profile_ctrl;
  bool time_based_profile_ctrl;
  bool acceleration_ctrl;
  bool joint_speed_ctrl;
  bool position_pid_gain;
  bool velocity_pi_gain;
  bool feed_forward_gain;
  bool cw_ccw_compliance;
  bool multi_protocol;
  bool get_hw_err_status;
} ModelAvailableFunc_t;



typedef struct ModelConrolTableInfo{
  uint8_t index;
  uint16_t addr;
  uint8_t addr_length;
} ModelConrolTableInfo_t;

const ModelConrolTableInfo_t adrx_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,           0, 2},
  {ControlTableItem::FIRMWARE_VERSION,       2, 1},
  {ControlTableItem::ID,                     3, 1},
  {ControlTableItem::BAUD_RATE,              4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,      5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,         6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,        8, 2},
  {ControlTableItem::TEMPERATURE_LIMIT,     11, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,     12, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,     13, 1},
  {ControlTableItem::MAX_TORQUE,            14, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,   16, 1},
  {ControlTableItem::ALARM_LED,             17, 1},
  {ControlTableItem::SHUTDOWN,              18, 1},

  {ControlTableItem::TORQUE_ENABLE,         24, 1},
  {ControlTableItem::LED,                   25, 1},
  {ControlTableItem::CW_COMPLIANCE_MARGIN,  26, 1},
  {ControlTableItem::CCW_COMPLIANCE_MARGIN, 27, 1},
  {ControlTableItem::CW_COMPLIANCE_SLOPE,   28, 1},
  {ControlTableItem::CCW_COMPLIANCE_SLOPE,  29, 1},
  {ControlTableItem::GOAL_POSITION,         30, 2},
  {ControlTableItem::MOVING_SPEED,          32, 2},
  {ControlTableItem::TORQUE_LIMIT,          34, 2},
  {ControlTableItem::PRESENT_POSITION,      36, 2},  
  {ControlTableItem::PRESENT_SPEED,         38, 2},
  {ControlTableItem::PRESENT_LOAD,          40, 2},
  {ControlTableItem::PRESENT_VOLTAGE,       42, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,   43, 1},
  {ControlTableItem::REGISTERED,            44, 1},
  {ControlTableItem::MOVING,                46, 1},    
  {ControlTableItem::LOCK,                  47, 1},
  {ControlTableItem::PUNCH,                 48, 2},

  {ControlTableItem::LAST_DUMMY_ITEM,        0, 0}
};

const ModelConrolTableInfo_t ex_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,           0, 2},
  {ControlTableItem::FIRMWARE_VERSION,       2, 1},
  {ControlTableItem::ID,                     3, 1},
  {ControlTableItem::BAUD_RATE,              4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,      5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,         6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,        8, 2},
  {ControlTableItem::DRIVE_MODE,            10, 1},
  {ControlTableItem::TEMPERATURE_LIMIT,     11, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,     12, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,     13, 1},
  {ControlTableItem::MAX_TORQUE,            14, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,   16, 1},
  {ControlTableItem::ALARM_LED,             17, 1},
  {ControlTableItem::SHUTDOWN,              18, 1},

  {ControlTableItem::TORQUE_ENABLE,         24, 1},
  {ControlTableItem::LED,                   25, 1},
  {ControlTableItem::CW_COMPLIANCE_MARGIN,  26, 1},
  {ControlTableItem::CCW_COMPLIANCE_MARGIN, 27, 1},
  {ControlTableItem::CW_COMPLIANCE_SLOPE,   28, 1},
  {ControlTableItem::CCW_COMPLIANCE_SLOPE,  29, 1},
  {ControlTableItem::GOAL_POSITION,         30, 2},
  {ControlTableItem::MOVING_SPEED,          32, 2},
  {ControlTableItem::TORQUE_LIMIT,          34, 2},
  {ControlTableItem::PRESENT_POSITION,      36, 2},  
  {ControlTableItem::PRESENT_SPEED,         38, 2},
  {ControlTableItem::PRESENT_LOAD,          40, 2},
  {ControlTableItem::PRESENT_VOLTAGE,       42, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,   43, 1},
  {ControlTableItem::REGISTERED,            44, 1},
  {ControlTableItem::MOVING,                46, 1},    
  {ControlTableItem::LOCK,                  47, 1},
  {ControlTableItem::PUNCH,                 48, 2},
  {ControlTableItem::SENSED_CURRENT,        56, 2},

  {ControlTableItem::LAST_DUMMY_ITEM,        0, 0}
};

const ModelConrolTableInfo_t mx12_28_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,           0, 2},
  {ControlTableItem::FIRMWARE_VERSION,       2, 1},
  {ControlTableItem::ID,                     3, 1},
  {ControlTableItem::BAUD_RATE,              4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,      5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,         6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,        8, 2},
  {ControlTableItem::TEMPERATURE_LIMIT,     11, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,     12, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,     13, 1},
  {ControlTableItem::MAX_TORQUE,            14, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,   16, 1},
  {ControlTableItem::ALARM_LED,             17, 1},
  {ControlTableItem::SHUTDOWN,              18, 1},
  {ControlTableItem::MULTI_TURN_OFFSET,     20, 2},
  {ControlTableItem::RESOLUTION_DIVIDER,    22, 1},

  {ControlTableItem::TORQUE_ENABLE,         24, 1},
  {ControlTableItem::LED,                   25, 1},
  {ControlTableItem::D_GAIN,                26, 1},
  {ControlTableItem::I_GAIN,                27, 1},
  {ControlTableItem::P_GAIN,                28, 1},
  {ControlTableItem::GOAL_POSITION,         30, 2},
  {ControlTableItem::MOVING_SPEED,          32, 2},
  {ControlTableItem::TORQUE_LIMIT,          34, 2},
  {ControlTableItem::PRESENT_POSITION,      36, 2},  
  {ControlTableItem::PRESENT_SPEED,         38, 2},
  {ControlTableItem::PRESENT_LOAD,          40, 2},
  {ControlTableItem::PRESENT_VOLTAGE,       42, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,   43, 1},
  {ControlTableItem::REGISTERED,            44, 1},
  {ControlTableItem::MOVING,                46, 1},    
  {ControlTableItem::LOCK,                  47, 1},
  {ControlTableItem::PUNCH,                 48, 2},
  {ControlTableItem::REALTIME_TICK,         50, 2},
  {ControlTableItem::GOAL_ACCELERATION,     73, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,        0, 0}
};

const ModelConrolTableInfo_t mx64_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::FIRMWARE_VERSION,         2, 1},
  {ControlTableItem::ID,                       3, 1},
  {ControlTableItem::BAUD_RATE,                4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,           6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,          8, 2},
  {ControlTableItem::TEMPERATURE_LIMIT,       11, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       12, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       13, 1},
  {ControlTableItem::MAX_TORQUE,              14, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,     16, 1},
  {ControlTableItem::ALARM_LED,               17, 1},
  {ControlTableItem::SHUTDOWN,                18, 1},
  {ControlTableItem::MULTI_TURN_OFFSET,       20, 2},
  {ControlTableItem::RESOLUTION_DIVIDER,      22, 1},

  {ControlTableItem::TORQUE_ENABLE,           24, 1},
  {ControlTableItem::LED,                     25, 1},
  {ControlTableItem::D_GAIN,                  26, 1},
  {ControlTableItem::I_GAIN,                  27, 1},
  {ControlTableItem::P_GAIN,                  28, 1},
  {ControlTableItem::GOAL_POSITION,           30, 2},
  {ControlTableItem::MOVING_SPEED,            32, 2},
  {ControlTableItem::TORQUE_LIMIT,            34, 2},
  {ControlTableItem::PRESENT_POSITION,        36, 2},  
  {ControlTableItem::PRESENT_SPEED,           38, 2},
  {ControlTableItem::PRESENT_LOAD,            40, 2},
  {ControlTableItem::PRESENT_VOLTAGE,         42, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,     43, 1},
  {ControlTableItem::REGISTERED,              44, 1},
  {ControlTableItem::MOVING,                  46, 1},    
  {ControlTableItem::LOCK,                    47, 1},
  {ControlTableItem::PUNCH,                   48, 2},
  {ControlTableItem::REALTIME_TICK,           50, 2},
  {ControlTableItem::CURRENT,                 68, 2},
  {ControlTableItem::TORQUE_CTRL_MODE_ENABLE, 70, 1},
  {ControlTableItem::GOAL_TORQUE,             71, 2},
  {ControlTableItem::GOAL_ACCELERATION,       73, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t mx106_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::FIRMWARE_VERSION,         2, 1},
  {ControlTableItem::ID,                       3, 1},
  {ControlTableItem::BAUD_RATE,                4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,           6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,          8, 2},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::TEMPERATURE_LIMIT,       11, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       12, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       13, 1},
  {ControlTableItem::MAX_TORQUE,              14, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,     16, 1},
  {ControlTableItem::ALARM_LED,               17, 1},
  {ControlTableItem::SHUTDOWN,                18, 1},
  {ControlTableItem::MULTI_TURN_OFFSET,       20, 2},
  {ControlTableItem::RESOLUTION_DIVIDER,      22, 1},

  {ControlTableItem::TORQUE_ENABLE,           24, 1},
  {ControlTableItem::LED,                     25, 1},
  {ControlTableItem::D_GAIN,                  26, 1},
  {ControlTableItem::I_GAIN,                  27, 1},
  {ControlTableItem::P_GAIN,                  28, 1},
  {ControlTableItem::GOAL_POSITION,           30, 2},
  {ControlTableItem::MOVING_SPEED,            32, 2},
  {ControlTableItem::TORQUE_LIMIT,            34, 2},
  {ControlTableItem::PRESENT_POSITION,        36, 2},  
  {ControlTableItem::PRESENT_SPEED,           38, 2},
  {ControlTableItem::PRESENT_LOAD,            40, 2},
  {ControlTableItem::PRESENT_VOLTAGE,         42, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,     43, 1},
  {ControlTableItem::REGISTERED,              44, 1},
  {ControlTableItem::MOVING,                  46, 1},    
  {ControlTableItem::LOCK,                    47, 1},
  {ControlTableItem::PUNCH,                   48, 2},
  {ControlTableItem::REALTIME_TICK,           50, 2},
  {ControlTableItem::CURRENT,                 68, 2},
  {ControlTableItem::TORQUE_CTRL_MODE_ENABLE, 70, 1},
  {ControlTableItem::GOAL_TORQUE,             71, 2},
  {ControlTableItem::GOAL_ACCELERATION,       73, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t xl320_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,           0, 2},
  {ControlTableItem::FIRMWARE_VERSION,       2, 1},
  {ControlTableItem::ID,                     3, 1},
  {ControlTableItem::BAUD_RATE,              4, 1},
  {ControlTableItem::RETURN_DELAY_TIME,      5, 1},
  {ControlTableItem::CW_ANGLE_LIMIT,         6, 2},
  {ControlTableItem::CCW_ANGLE_LIMIT,        8, 2},
  {ControlTableItem::CONTROL_MODE,          11, 1},
  {ControlTableItem::TEMPERATURE_LIMIT,     12, 1},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,     13, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,     14, 1},
  {ControlTableItem::MAX_TORQUE,            15, 2},
  {ControlTableItem::STATUS_RETURN_LEVEL,   17, 1},
  {ControlTableItem::SHUTDOWN,              18, 1},

  {ControlTableItem::TORQUE_ENABLE,         24, 1},
  {ControlTableItem::LED,                   25, 1},
  {ControlTableItem::D_GAIN,                27, 1},
  {ControlTableItem::I_GAIN,                28, 1},
  {ControlTableItem::P_GAIN,                29, 1},
  {ControlTableItem::GOAL_POSITION,         30, 2},
  {ControlTableItem::MOVING_SPEED,          32, 2},
  {ControlTableItem::TORQUE_LIMIT,          35, 2},
  {ControlTableItem::PRESENT_POSITION,      37, 2},  
  {ControlTableItem::PRESENT_SPEED,         39, 2},
  {ControlTableItem::PRESENT_LOAD,          41, 2},
  {ControlTableItem::PRESENT_VOLTAGE,       45, 1},
  {ControlTableItem::PRESENT_TEMPERATURE,   46, 1},
  {ControlTableItem::REGISTERED,            47, 1},
  {ControlTableItem::MOVING,                49, 1},    
  {ControlTableItem::HARDWARE_ERROR_STATUS, 50, 1},
  {ControlTableItem::PUNCH,                 51, 2},

  {ControlTableItem::LAST_DUMMY_ITEM,        0, 0}
};

const ModelConrolTableInfo_t xl430_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::MODEL_INFORMATION,        2, 4},
  {ControlTableItem::FIRMWARE_VERSION,         6, 1},
  {ControlTableItem::ID,                       7, 1},
  {ControlTableItem::BAUD_RATE,                8, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        9, 1},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::OPERATING_MODE,          11, 1},
  {ControlTableItem::SECONDARY_ID,            12, 1},
  {ControlTableItem::PROTOCOL_VERSION,        13, 1},
  {ControlTableItem::HOMING_OFFSET,           20, 4},
  {ControlTableItem::MOVING_THRESHOLD,        24, 4},
  {ControlTableItem::TEMPERATURE_LIMIT,       31, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       32, 2},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       34, 2},
  {ControlTableItem::PWM_LIMIT,               36, 2},
  {ControlTableItem::VELOCITY_LIMIT,          44, 4},
  {ControlTableItem::MAX_POSITION_LIMIT,      48, 4},
  {ControlTableItem::MIN_POSITION_LIMIT,      52, 4},
  {ControlTableItem::SHUTDOWN,                63, 1},

  {ControlTableItem::TORQUE_ENABLE,           64, 1},
  {ControlTableItem::LED,                     65, 1},
  {ControlTableItem::STATUS_RETURN_LEVEL,     68, 1},
  {ControlTableItem::REGISTERED_INSTRUCTION,  69, 1},
  {ControlTableItem::HARDWARE_ERROR_STATUS,   70, 1},
  {ControlTableItem::VELOCITY_I_GAIN,         76, 2},
  {ControlTableItem::VELOCITY_P_GAIN,         78, 2},
  {ControlTableItem::POSITION_D_GAIN,         80, 2},
  {ControlTableItem::POSITION_I_GAIN,         82, 2},
  {ControlTableItem::POSITION_P_GAIN,         84, 2},
  {ControlTableItem::FEEDFORWARD_2ND_GAIN,    88, 2},
  {ControlTableItem::FEEDFORWARD_1ST_GAIN,    90, 2},  
  {ControlTableItem::BUS_WATCHDOG,            98, 2},
  {ControlTableItem::GOAL_PWM,               100, 2},
  {ControlTableItem::GOAL_VELOCITY,          104, 4},
  {ControlTableItem::PROFILE_ACCELERATION,   108, 4},
  {ControlTableItem::PROFILE_VELOCITY,       112, 4},
  {ControlTableItem::GOAL_POSITION,          116, 4},
  {ControlTableItem::REALTIME_TICK,          120, 2},
  {ControlTableItem::MOVING,                 122, 1},    
  {ControlTableItem::MOVING_STATUS,          123, 1},    
  {ControlTableItem::PRESENT_PWM,            124, 2},
  {ControlTableItem::PRESENT_LOAD,           126, 2},
  {ControlTableItem::PRESENT_VELOCITY,       128, 4},
  {ControlTableItem::PRESENT_POSITION,       132, 4},
  {ControlTableItem::VELOCITY_TRAJECTORY,    136, 4},
  {ControlTableItem::POSITION_TRAJECTORY,    140, 4},  
  {ControlTableItem::PRESENT_INPUT_VOLTAGE,  144, 2},
  {ControlTableItem::PRESENT_TEMPERATURE,    146, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t mx28_2_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::MODEL_INFORMATION,        2, 4},
  {ControlTableItem::FIRMWARE_VERSION,         6, 1},
  {ControlTableItem::ID,                       7, 1},
  {ControlTableItem::BAUD_RATE,                8, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        9, 1},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::OPERATING_MODE,          11, 1},
  {ControlTableItem::SECONDARY_ID,            12, 1},
  {ControlTableItem::PROTOCOL_VERSION,        13, 1},
  {ControlTableItem::HOMING_OFFSET,           20, 4},
  {ControlTableItem::MOVING_THRESHOLD,        24, 4},
  {ControlTableItem::TEMPERATURE_LIMIT,       31, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       32, 2},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       34, 2},
  {ControlTableItem::PWM_LIMIT,               36, 2},
  {ControlTableItem::ACCELERATION_LIMIT,      40, 4},  
  {ControlTableItem::VELOCITY_LIMIT,          44, 4},
  {ControlTableItem::MAX_POSITION_LIMIT,      48, 4},
  {ControlTableItem::MIN_POSITION_LIMIT,      52, 4},
  {ControlTableItem::SHUTDOWN,                63, 1},

  {ControlTableItem::TORQUE_ENABLE,           64, 1},
  {ControlTableItem::LED,                     65, 1},
  {ControlTableItem::STATUS_RETURN_LEVEL,     68, 1},
  {ControlTableItem::REGISTERED_INSTRUCTION,  69, 1},
  {ControlTableItem::HARDWARE_ERROR_STATUS,   70, 1},
  {ControlTableItem::VELOCITY_I_GAIN,         76, 2},
  {ControlTableItem::VELOCITY_P_GAIN,         78, 2},
  {ControlTableItem::POSITION_D_GAIN,         80, 2},
  {ControlTableItem::POSITION_I_GAIN,         82, 2},
  {ControlTableItem::POSITION_P_GAIN,         84, 2},
  {ControlTableItem::FEEDFORWARD_2ND_GAIN,    88, 2},
  {ControlTableItem::FEEDFORWARD_1ST_GAIN,    90, 2},  
  {ControlTableItem::BUS_WATCHDOG,            98, 2},
  {ControlTableItem::GOAL_PWM,               100, 2},
  {ControlTableItem::GOAL_VELOCITY,          104, 4},
  {ControlTableItem::PROFILE_ACCELERATION,   108, 4},
  {ControlTableItem::PROFILE_VELOCITY,       112, 4},
  {ControlTableItem::GOAL_POSITION,          116, 4},
  {ControlTableItem::REALTIME_TICK,          120, 2},
  {ControlTableItem::MOVING,                 122, 1},    
  {ControlTableItem::MOVING_STATUS,          123, 1},    
  {ControlTableItem::PRESENT_PWM,            124, 2},
  {ControlTableItem::PRESENT_LOAD,           126, 2},
  {ControlTableItem::PRESENT_VELOCITY,       128, 4},
  {ControlTableItem::PRESENT_POSITION,       132, 4},
  {ControlTableItem::VELOCITY_TRAJECTORY,    136, 4},
  {ControlTableItem::POSITION_TRAJECTORY,    140, 4},  
  {ControlTableItem::PRESENT_INPUT_VOLTAGE,  144, 2},
  {ControlTableItem::PRESENT_TEMPERATURE,    146, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t mx64_106_2_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::MODEL_INFORMATION,        2, 4},
  {ControlTableItem::FIRMWARE_VERSION,         6, 1},
  {ControlTableItem::ID,                       7, 1},
  {ControlTableItem::BAUD_RATE,                8, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        9, 1},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::OPERATING_MODE,          11, 1},
  {ControlTableItem::SECONDARY_ID,            12, 1},
  {ControlTableItem::PROTOCOL_VERSION,        13, 1},
  {ControlTableItem::HOMING_OFFSET,           20, 4},
  {ControlTableItem::MOVING_THRESHOLD,        24, 4},
  {ControlTableItem::TEMPERATURE_LIMIT,       31, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       32, 2},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       34, 2},
  {ControlTableItem::PWM_LIMIT,               36, 2},
  {ControlTableItem::CURRENT_LIMIT,           38, 2},
  {ControlTableItem::ACCELERATION_LIMIT,      40, 4},  
  {ControlTableItem::VELOCITY_LIMIT,          44, 4},
  {ControlTableItem::MAX_POSITION_LIMIT,      48, 4},
  {ControlTableItem::MIN_POSITION_LIMIT,      52, 4},
  {ControlTableItem::SHUTDOWN,                63, 1},

  {ControlTableItem::TORQUE_ENABLE,           64, 1},
  {ControlTableItem::LED,                     65, 1},
  {ControlTableItem::STATUS_RETURN_LEVEL,     68, 1},
  {ControlTableItem::REGISTERED_INSTRUCTION,  69, 1},
  {ControlTableItem::HARDWARE_ERROR_STATUS,   70, 1},
  {ControlTableItem::VELOCITY_I_GAIN,         76, 2},
  {ControlTableItem::VELOCITY_P_GAIN,         78, 2},
  {ControlTableItem::POSITION_D_GAIN,         80, 2},
  {ControlTableItem::POSITION_I_GAIN,         82, 2},
  {ControlTableItem::POSITION_P_GAIN,         84, 2},
  {ControlTableItem::FEEDFORWARD_2ND_GAIN,    88, 2},
  {ControlTableItem::FEEDFORWARD_1ST_GAIN,    90, 2},  
  {ControlTableItem::BUS_WATCHDOG,            98, 2},
  {ControlTableItem::GOAL_PWM,               100, 2},
  {ControlTableItem::GOAL_CURRENT,           102, 2},
  {ControlTableItem::GOAL_VELOCITY,          104, 4},
  {ControlTableItem::PROFILE_ACCELERATION,   108, 4},
  {ControlTableItem::PROFILE_VELOCITY,       112, 4},
  {ControlTableItem::GOAL_POSITION,          116, 4},
  {ControlTableItem::REALTIME_TICK,          120, 2},
  {ControlTableItem::MOVING,                 122, 1},    
  {ControlTableItem::MOVING_STATUS,          123, 1},    
  {ControlTableItem::PRESENT_PWM,            124, 2},
  {ControlTableItem::PRESENT_CURRENT,        126, 2},
  {ControlTableItem::PRESENT_VELOCITY,       128, 4},
  {ControlTableItem::PRESENT_POSITION,       132, 4},
  {ControlTableItem::VELOCITY_TRAJECTORY,    136, 4},
  {ControlTableItem::POSITION_TRAJECTORY,    140, 4},  
  {ControlTableItem::PRESENT_INPUT_VOLTAGE,  144, 2},
  {ControlTableItem::PRESENT_TEMPERATURE,    146, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t xmh430_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::MODEL_INFORMATION,        2, 4},
  {ControlTableItem::FIRMWARE_VERSION,         6, 1},
  {ControlTableItem::ID,                       7, 1},
  {ControlTableItem::BAUD_RATE,                8, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        9, 1},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::OPERATING_MODE,          11, 1},
  {ControlTableItem::SECONDARY_ID,            12, 1},
  {ControlTableItem::PROTOCOL_VERSION,        13, 1},
  {ControlTableItem::HOMING_OFFSET,           20, 4},
  {ControlTableItem::MOVING_THRESHOLD,        24, 4},
  {ControlTableItem::TEMPERATURE_LIMIT,       31, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       32, 2},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       34, 2},
  {ControlTableItem::PWM_LIMIT,               36, 2},
  {ControlTableItem::CURRENT_LIMIT,           38, 2},
  {ControlTableItem::VELOCITY_LIMIT,          44, 4},
  {ControlTableItem::MAX_POSITION_LIMIT,      48, 4},
  {ControlTableItem::MIN_POSITION_LIMIT,      52, 4},
  {ControlTableItem::SHUTDOWN,                63, 1},

  {ControlTableItem::TORQUE_ENABLE,           64, 1},
  {ControlTableItem::LED,                     65, 1},
  {ControlTableItem::STATUS_RETURN_LEVEL,     68, 1},
  {ControlTableItem::REGISTERED_INSTRUCTION,  69, 1},
  {ControlTableItem::HARDWARE_ERROR_STATUS,   70, 1},
  {ControlTableItem::VELOCITY_I_GAIN,         76, 2},
  {ControlTableItem::VELOCITY_P_GAIN,         78, 2},
  {ControlTableItem::POSITION_D_GAIN,         80, 2},
  {ControlTableItem::POSITION_I_GAIN,         82, 2},
  {ControlTableItem::POSITION_P_GAIN,         84, 2},
  {ControlTableItem::FEEDFORWARD_2ND_GAIN,    88, 2},
  {ControlTableItem::FEEDFORWARD_1ST_GAIN,    90, 2},  
  {ControlTableItem::BUS_WATCHDOG,            98, 2},
  {ControlTableItem::GOAL_PWM,               100, 2},
  {ControlTableItem::GOAL_CURRENT,           102, 2},
  {ControlTableItem::GOAL_VELOCITY,          104, 4},
  {ControlTableItem::PROFILE_ACCELERATION,   108, 4},
  {ControlTableItem::PROFILE_VELOCITY,       112, 4},
  {ControlTableItem::GOAL_POSITION,          116, 4},
  {ControlTableItem::REALTIME_TICK,          120, 2},
  {ControlTableItem::MOVING,                 122, 1},    
  {ControlTableItem::MOVING_STATUS,          123, 1},    
  {ControlTableItem::PRESENT_PWM,            124, 2},
  {ControlTableItem::PRESENT_CURRENT,        126, 2},
  {ControlTableItem::PRESENT_VELOCITY,       128, 4},
  {ControlTableItem::PRESENT_POSITION,       132, 4},
  {ControlTableItem::VELOCITY_TRAJECTORY,    136, 4},
  {ControlTableItem::POSITION_TRAJECTORY,    140, 4},  
  {ControlTableItem::PRESENT_INPUT_VOLTAGE,  144, 2},
  {ControlTableItem::PRESENT_TEMPERATURE,    146, 1},

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};

const ModelConrolTableInfo_t xmh540_control_table[] PROGMEM = {
  {ControlTableItem::MODEL_NUMBER,             0, 2},
  {ControlTableItem::MODEL_INFORMATION,        2, 4},
  {ControlTableItem::FIRMWARE_VERSION,         6, 1},
  {ControlTableItem::ID,                       7, 1},
  {ControlTableItem::BAUD_RATE,                8, 1},
  {ControlTableItem::RETURN_DELAY_TIME,        9, 1},
  {ControlTableItem::DRIVE_MODE,              10, 1},
  {ControlTableItem::OPERATING_MODE,          11, 1},
  {ControlTableItem::SECONDARY_ID,            12, 1},
  {ControlTableItem::PROTOCOL_VERSION,        13, 1},
  {ControlTableItem::HOMING_OFFSET,           20, 4},
  {ControlTableItem::MOVING_THRESHOLD,        24, 4},
  {ControlTableItem::TEMPERATURE_LIMIT,       31, 1},
  {ControlTableItem::MAX_VOLTAGE_LIMIT,       32, 2},
  {ControlTableItem::MIN_VOLTAGE_LIMIT,       34, 2},
  {ControlTableItem::PWM_LIMIT,               36, 2},
  {ControlTableItem::CURRENT_LIMIT,           38, 2},
  {ControlTableItem::VELOCITY_LIMIT,          44, 4},
  {ControlTableItem::MAX_POSITION_LIMIT,      48, 4},
  {ControlTableItem::MIN_POSITION_LIMIT,      52, 4},
  {ControlTableItem::EXTERNAL_PORT_MODE_1,    56, 1},
  {ControlTableItem::EXTERNAL_PORT_MODE_2,    57, 1},
  {ControlTableItem::EXTERNAL_PORT_MODE_3,    58, 1},  
  {ControlTableItem::SHUTDOWN,                63, 1},

  {ControlTableItem::TORQUE_ENABLE,           64, 1},
  {ControlTableItem::LED,                     65, 1},
  {ControlTableItem::STATUS_RETURN_LEVEL,     68, 1},
  {ControlTableItem::REGISTERED_INSTRUCTION,  69, 1},
  {ControlTableItem::HARDWARE_ERROR_STATUS,   70, 1},
  {ControlTableItem::VELOCITY_I_GAIN,         76, 2},
  {ControlTableItem::VELOCITY_P_GAIN,         78, 2},
  {ControlTableItem::POSITION_D_GAIN,         80, 2},
  {ControlTableItem::POSITION_I_GAIN,         82, 2},
  {ControlTableItem::POSITION_P_GAIN,         84, 2},
  {ControlTableItem::FEEDFORWARD_2ND_GAIN,    88, 2},
  {ControlTableItem::FEEDFORWARD_1ST_GAIN,    90, 2},  
  {ControlTableItem::BUS_WATCHDOG,            98, 2},
  {ControlTableItem::GOAL_PWM,               100, 2},
  {ControlTableItem::GOAL_CURRENT,           102, 2},
  {ControlTableItem::GOAL_VELOCITY,          104, 4},
  {ControlTableItem::PROFILE_ACCELERATION,   108, 4},
  {ControlTableItem::PROFILE_VELOCITY,       112, 4},
  {ControlTableItem::GOAL_POSITION,          116, 4},
  {ControlTableItem::REALTIME_TICK,          120, 2},
  {ControlTableItem::MOVING,                 122, 1},    
  {ControlTableItem::MOVING_STATUS,          123, 1},    
  {ControlTableItem::PRESENT_PWM,            124, 2},
  {ControlTableItem::PRESENT_CURRENT,        126, 2},
  {ControlTableItem::PRESENT_VELOCITY,       128, 4},
  {ControlTableItem::PRESENT_POSITION,       132, 4},
  {ControlTableItem::VELOCITY_TRAJECTORY,    136, 4},
  {ControlTableItem::POSITION_TRAJECTORY,    140, 4},  
  {ControlTableItem::PRESENT_INPUT_VOLTAGE,  144, 2},
  {ControlTableItem::PRESENT_TEMPERATURE,    146, 1},
  {ControlTableItem::EXTERNAL_PORT_DATA_1,   152, 2},
  {ControlTableItem::EXTERNAL_PORT_DATA_2,   154, 2},
  {ControlTableItem::EXTERNAL_PORT_DATA_3,   156, 2},  

  {ControlTableItem::LAST_DUMMY_ITEM,          0, 0}
};




static bool getModelAvailableFunc(uint16_t model_num, ModelAvailableFunc_t *func_table);

ControlTableItemInfo_t DYNAMIXEL::getControlTableItemInfo(uint16_t model_num, ControlTableItem control_item)
{
  uint8_t item_idx, i = 0;
  ModelConrolTableInfo_t *p_ctable = nullptr;
  ControlTableItemInfo_t item_info;
  memset(&item_info, 0, sizeof(item_info));

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
      p_ctable = (ModelConrolTableInfo_t*)adrx_control_table;
      break;

    case EX106:
      p_ctable = (ModelConrolTableInfo_t*)ex_control_table;
      break;

    case MX12W:
    case MX28:
      p_ctable = (ModelConrolTableInfo_t*)mx12_28_control_table;
      break;

    case MX64:
      p_ctable = (ModelConrolTableInfo_t*)mx64_control_table;
      break;      

    case MX106:
      p_ctable = (ModelConrolTableInfo_t*)mx106_control_table;
      break;            

    case XL320:
      p_ctable = (ModelConrolTableInfo_t*)xl320_control_table;
      break;      

    case XL430_W250:
      p_ctable = (ModelConrolTableInfo_t*)xl430_control_table;
      break;

    case MX28_2:
      p_ctable = (ModelConrolTableInfo_t*)mx28_2_control_table;
      break;

    case MX64_2:
    case MX106_2:
      p_ctable = (ModelConrolTableInfo_t*)mx64_106_2_control_table;
      break;

    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
      p_ctable = (ModelConrolTableInfo_t*)xmh430_control_table;
      break;

    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
      p_ctable = (ModelConrolTableInfo_t*)xmh540_control_table;
      break;            

    default:
      break;
  }

  if(p_ctable == nullptr){
    return item_info;
  }

  do{
    item_idx = pgm_read_byte_far(&p_ctable[i].index);
    if(item_idx == control_item) {
      item_info.addr = pgm_read_word_far(&p_ctable[i].addr);
      item_info.addr_length = pgm_read_byte_far(&p_ctable[i].addr_length);
      break;
    }
    i++;
  }while(item_idx != LAST_DUMMY_ITEM);

  return item_info;
}

bool availableFunction(uint16_t model_num, Functions dxl_func)
{
  bool ret = false;
  ModelAvailableFunc_t func_table;

  if(getModelAvailableFunc(model_num, &func_table) == false)
    return ret;

  switch(dxl_func)
  {
    case TORQUE_ON_OFF:
      ret = func_table.torque_on_off;
      break;
    case LED_CTRL:
      ret = func_table.led_ctrl;
      break;
    case VELOCITY_CTRL:
      ret = func_table.velocity_ctrl;
      break;
    case POSITION_CTRL:
      ret = func_table.position_ctrl;
      break;  
    case EXT_POSITION_CTRL:
      ret = func_table.ext_pos_ctrl;
      break;      
    case PWM_CTRL:
      ret = func_table.pwm_ctrl;
      break;
    case CURRENT_CTRL:
      ret = func_table.current_ctrl;
      break;
    case CURRENT_BASED_POS_CTRL:
      ret = func_table.current_based_pos_ctrl;
      break;
    case DIRECTION_CTRL:
      ret = func_table.direction_ctrl;
      break;  
    case PROFILE_CTRL:
      ret = func_table.profile_ctrl;
      break;            
    case TIME_BASED_PROFILE_CTRL:
      ret = func_table.time_based_profile_ctrl;
      break;
    case ACCELERATION_CTRL:
      ret = func_table.acceleration_ctrl;
      break;
    case JOINT_SPEED_CTRL:
      ret = func_table.joint_speed_ctrl;
      break;
    case POSITION_PID_GAIN:
      ret = func_table.position_pid_gain;
      break;  
    case VELOCITY_PI_GAIN:
      ret = func_table.velocity_pi_gain;
      break;            
    case FEED_FORWARD_GAIN:
      ret = func_table.feed_forward_gain;
      break;  
    case CW_CCW_COMPLIANCE:
      ret = func_table.cw_ccw_compliance;
      break;        
    case MULTI_PROTOCOL:
      ret = func_table.multi_protocol;
      break;       
    case GET_HARDWARE_ERROR_STATUS:
      ret = func_table.get_hw_err_status;

    default:
      break;
  }

  return ret;
}


static bool getModelAvailableFunc(uint16_t model_num, ModelAvailableFunc_t *func_table)
{
  bool ret = true;

  //Common
  func_table->torque_on_off = true;
  func_table->led_ctrl = true;
  func_table->velocity_ctrl = true;
  func_table->position_ctrl = true;

  func_table->ext_pos_ctrl = false;
  func_table->pwm_ctrl = false;
  func_table->current_ctrl = false;
  func_table->current_based_pos_ctrl = false;
  func_table->direction_ctrl = false;
  func_table->profile_ctrl = false;
  func_table->time_based_profile_ctrl = false;
  func_table->acceleration_ctrl = false;
  func_table->joint_speed_ctrl = false;
  func_table->position_pid_gain = false;
  func_table->velocity_pi_gain = false;
  func_table->feed_forward_gain = false;
  func_table->cw_ccw_compliance = false;
  func_table->multi_protocol = false;
  func_table->get_hw_err_status = false;

  switch (model_num)
  {
    case AX12A:
    case AX12W:
    case AX18A:
      func_table->joint_speed_ctrl = true;
      func_table->cw_ccw_compliance = true;
      break;

    case XL320:
      func_table->joint_speed_ctrl = true;
      func_table->cw_ccw_compliance = true;
      func_table->get_hw_err_status = true;
      break;      

    case MX12W:
    case MX28:
      func_table->ext_pos_ctrl = true;
      func_table->acceleration_ctrl = true;
      func_table->joint_speed_ctrl = true;
      func_table->cw_ccw_compliance = true;
      break;      

    case MX64:
    case MX106:
      func_table->ext_pos_ctrl = true;
      func_table->current_ctrl = true;
      func_table->acceleration_ctrl = true;
      func_table->joint_speed_ctrl = true;
      func_table->cw_ccw_compliance = true;
      break;

    case MX28_2:
    case XL430_W250:
      func_table->multi_protocol = true;
      func_table->ext_pos_ctrl = true;
      func_table->pwm_ctrl = true;    
      func_table->direction_ctrl = true;
      func_table->profile_ctrl = true;
      func_table->time_based_profile_ctrl = true;
      func_table->position_pid_gain = true;
      func_table->velocity_pi_gain = true;
      func_table->feed_forward_gain = true;
      func_table->get_hw_err_status = true;
      break;  

    case MX64_2:
    case MX106_2:
    case XM430_W210:
    case XM430_W350:
    case XH430_V210:
    case XH430_V350:
    case XH430_W210:
    case XH430_W350:
      func_table->multi_protocol = true;
      func_table->ext_pos_ctrl = true;
      func_table->pwm_ctrl = true;
      func_table->current_ctrl = true;
      func_table->current_based_pos_ctrl = true;
      func_table->direction_ctrl = true;
      func_table->profile_ctrl = true;
      func_table->time_based_profile_ctrl = true;
      func_table->position_pid_gain = true;
      func_table->velocity_pi_gain = true;
      func_table->feed_forward_gain = true;
      func_table->get_hw_err_status = true;
      break;

    case XM540_W150:
    case XM540_W270:
    case XH540_W150:
    case XH540_W270:
    case XH540_V150:
    case XH540_V270:
      func_table->multi_protocol = true;
      func_table->ext_pos_ctrl = true;
      func_table->pwm_ctrl = true;
      func_table->current_ctrl = true;
      func_table->current_based_pos_ctrl = true;
      func_table->direction_ctrl = true;
      func_table->profile_ctrl = true;
      func_table->time_based_profile_ctrl = true;
      func_table->position_pid_gain = true;
      func_table->velocity_pi_gain = true;
      func_table->feed_forward_gain = true;
      func_table->get_hw_err_status = true;
      break;

    default:
      ret = false;
      break;
  }

  return ret;
}