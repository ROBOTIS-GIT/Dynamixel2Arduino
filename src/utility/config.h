#ifndef DYNAMIXEL_CONFIG_H_
#define DYNAMIXEL_CONFIG_H_


#define ENABLE_ACTUATOR_AX              1
#define ENABLE_ACTUATOR_DX              0
#define ENABLE_ACTUATOR_RX              0
#define ENABLE_ACTUATOR_EX              0
#define ENABLE_ACTUATOR_MX12W           1
#define ENABLE_ACTUATOR_MX28            1
#define ENABLE_ACTUATOR_MX64            1
#define ENABLE_ACTUATOR_MX106           1

#define ENABLE_ACTUATOR_MX28_PROTOCOL2  1
#define ENABLE_ACTUATOR_MX64_PROTOCOL2  1
#define ENABLE_ACTUATOR_MX106_PROTOCOL2 1

#define ENABLE_ACTUATOR_XL320           1
#define ENABLE_ACTUATOR_XL430           1   //Includes 2XL430
#define ENABLE_ACTUATOR_XC430           1
#define ENABLE_ACTUATOR_XM430           1
#define ENABLE_ACTUATOR_XH430           1
#define ENABLE_ACTUATOR_XM540           1
#define ENABLE_ACTUATOR_XH540           1

#define ENABLE_ACTUATOR_PRO_R           1
#define ENABLE_ACTUATOR_PRO_RA          1
#define ENABLE_ACTUATOR_PRO_PLUS        1


#if defined (ARDUINO_AVR_UNO) || defined (ARDUINO_AVR_YUN) \
  || defined (ARDUINO_AVR_INDUSTRIAL101)
#define DEFAULT_DXL_BUF_LENGTH       192
#elif defined (ARDUINO_AVR_LEONARDO)
#define DEFAULT_DXL_BUF_LENGTH       256
#elif defined (OpenCR)
#define DEFAULT_DXL_BUF_LENGTH       2048
#else
#define DEFAULT_DXL_BUF_LENGTH       1024
#endif


#if (DEFAULT_DXL_BUF_LENGTH > 0xFFFF)
#error "\r\nError : DEFAULT_DXL_BUF_LENGTH is OVERFLOW! This must be a 16 bit range."
#endif




#endif /* DYNAMIXEL_CONFIG_H_ */