#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)
typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0301,
  CHASSIS_CTRL_CMD_ID = 0x0302,
  
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{

  float vx;
  float vy;
  float vw;
  
}  chassis_odom_info_t;


typedef struct
{

  float vx;
  float vy;
  float vw;

}  chassis_ctrl_info_t;

typedef struct
{

  uint16_t R;
  uint16_t G;
  uint16_t B;

} RGB_info_t;

typedef struct
{

 uint16_t LED1;
 uint16_t LED2;
 uint16_t LED3;
 
} message_info_t;
typedef struct 
{
  uint32_t motor1_ecd;
 uint32_t motor1_temp;
 uint32_t motor1_speed_rpm;
}motor_message_info_t;

typedef struct
{

  uint16_t pwm2;
  uint16_t pwm1;

}  servo_ctrl_info_t;

typedef struct 
{
  uint16_t current1;
  uint16_t current2;
  uint16_t current3;
  uint16_t current4;
} motor_info_t;


typedef struct
{

  int16_t ch[5];
  char s[2];

}  rc_info_t;


#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
