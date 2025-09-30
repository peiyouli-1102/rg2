/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       pid.c/h
  * @brief      Optimized cascaded PID controller for motor control
  * @note       Refactored for better structure, memory efficiency and readability
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-18-2025     ??name            1. done
  *  V1.1.0     Sep-13-2025     ??name            2. chassis
  *  V2.0.0     Sep-13-2025     AI Assistant      3. Refactored and optimized
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
//extern const int16_t Upside_rpm;
extern const float Kp;
extern const float Ki;
extern const float Kd;

extern const int32_t I_threshold;

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

// 位置-速度串级PID相关变量 (仅保留外部使用的)
extern int32_t pos[4];           // 当前位置 (用于调试输出)
extern PID pid_position[4];      // 位置PID控制器
extern PID pid_velocity[4];      // 速度PID控制器
extern PID pid_location[3];      // 底盘PID控制器

extern int32_t pid_total_part[3]; // 底盘位置积分
extern int32_t pid_veloc_part[3]; // 底盘速度输出
extern int32_t pid_target;        // 底盘目标值
extern int32_t pid_total;         // 总位置
extern int8_t tar[4];             // 电机方向
extern int8_t flag;               // 控制模式标志
extern int8_t chassis_stat;       // 底盘状态

extern uint8_t get_part ;

#endif // PID_H

void pid_shoot();
void pid_chassis();
uint16_t ABS(int16_t a);
void get_angle(int8_t i);

// 位置-速度串级PID相关函数
void pid_angle_init(void);
void pid_angle(void);
void chassis_analyse(void);
int8_t check_pid(void);
void clean_pid(void);

// 新增的控制接口函数
void set_control_mode(int8_t mode);        // 设置控制模式: 0=位置控制, 1=底盘分析
void set_chassis_mode(int8_t mode);        // 设置底盘模式: 0=转向, 1=右移, 2=前进
void set_position_target(int8_t motor_id, int32_t target);  // 设置位置目标
void set_chassis_target(int32_t target);   // 设置底盘目标

