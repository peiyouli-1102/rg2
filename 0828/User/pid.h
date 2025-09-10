/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       pid.c/h
  * @brief      to control motor speed and angle by p i d .
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-18-2025     ??              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "struct_typedef.h"
#include "CAN_receive.h"

#ifndef PID_H
#define PID_H

// PID结构体定义
typedef struct
{
    int32_t target_val;   // 目标值
    int32_t output_val;   // 输出值
    int16_t err;          // 当前误差
    int16_t err_last;     // 上次误差
    int32_t integral;     // 积分项
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
} PID;

// 常量声明为 extern const
extern const int16_t Upside_rpm;
extern const float Kp;
extern const float Ki;
extern const float Kd;

extern const int32_t P_threshold;
extern const int32_t I_threshold;
extern const int32_t D_threshold;

extern const float Speeding_rate;
extern const int16_t Zero;
extern const int16_t Dt;
extern const int16_t Safety_threshold_shoot;
extern const int16_t Safety_threshold_chass;

// 变量声明为 extern
extern int32_t Integral_right;
extern int32_t Integral_left;
extern int32_t Integral[6];

extern int32_t goal;
extern int32_t goal_ch[4];
extern int32_t last_goal;
extern int32_t last_goal_ch[4];

extern int16_t ls_right;
extern int16_t ls_left;
extern int16_t ls[4];
extern int16_t crt_right;
extern int16_t crt_left;
extern int16_t crt[4];

// 位置-速度串级PID相关变量
extern int64_t ls_pos[4];        // 上次高精度位置
extern int64_t postn[4];         // 高精度位置
extern int32_t pos[4];           // 当前位置
extern int32_t target[4];        // 目标位置
extern PID pid_position[4];      // 位置PID控制器
extern PID pid_velocity[4];      // 速度PID控制器

extern PID pid_location;
extern PID pid_speed;

#endif // PID_H

int16_t shoot_pid(int16_t spgiven, int16_t spreal, int16_t rpm, int16_t goal);
void pid_shoot();
void pid_chassis();
uint16_t ABS(int16_t a);
void get_angle(int8_t i);

// 位置-速度串级PID相关函数
void pid_angle_init(void);
void pid_angle(void);
int16_t pid_calculate_position(PID *pid, int32_t actual_val);
int16_t pid_calculate_velocity(PID *pid, int16_t actual_rpm);
void set_pid_target(PID *pid, int16_t target_val);
void pid_angle_update_position(int8_t motor_id);
int8_t check_pid(void);
void clean_pid(void);
void PID_param_init();
float location_pid_realize(PID *pid, float actual_val);

