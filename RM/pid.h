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

#endif // PID_H


int16_t shoot_pid(int16_t spgiven, int16_t spreal, int16_t rpm, int16_t goal);
void pid_shoot(void);
void pid_chassis(void);
