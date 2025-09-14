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
#include "pid.h"

// =============================================================================
// 常量定义
// =============================================================================
#define ENCODER_RESOLUTION 8192U
#define POSITION_SCALE_FACTOR 100U
#define MOTOR_COUNT 4U
#define CHASSIS_MODE_COUNT 3U

// PID参数常量
#define POSITION_DEADZONE 10
#define VELOCITY_DEADZONE 100
#define INTEGRAL_LIMIT_POSITION 10000
#define INTEGRAL_LIMIT_VELOCITY 1000
#define INTEGRAL_LIMIT_CHASSIS 114514

// 安全阈值
#define SAFETY_THRESHOLD_SHOOT 1000
#define SAFETY_THRESHOLD_CHASSIS 3000

// =============================================================================
// 射击PID相关变量 (保留原有功能)
// =============================================================================
static const float Kp_shoot = 0.7f;
static const float Ki_shoot = 0.01f;
static const float Kd_shoot = 0.1f;

static int32_t integral_right = 0;
static int32_t integral_left = 0;
static int16_t last_speed_right = 0;
static int16_t last_speed_left = 0;
static int16_t current_right = 0;
static int16_t current_left = 0;

// =============================================================================
// 底盘PID相关变量 (保留原有功能)
// =============================================================================
static const float Kp_chassis = 5.0f;
static const float Ki_chassis = 0.01f;
static const float Kd_chassis = 0.0f;

static int32_t integral_chassis[MOTOR_COUNT] = {0};
static int32_t goal_chassis[MOTOR_COUNT] = {0};
static int32_t last_goal_chassis[MOTOR_COUNT] = {0};
static int16_t last_speed_chassis[MOTOR_COUNT] = {0};
static int16_t current_chassis[MOTOR_COUNT] = {0};

// =============================================================================
// 串级PID相关变量 (优化后)
// =============================================================================
// 位置相关变量 - 使用int32_t替代int64_t，减少内存占用
static int32_t position_raw[MOTOR_COUNT] = {0};      // 原始位置计数
static int32_t position_scaled[MOTOR_COUNT] = {0};   // 缩放后位置
static int32_t last_position_raw[MOTOR_COUNT] = {0}; // 上次原始位置

static int8_t control_mode = 0;  // 0:位置控制, 1:底盘分析模式
static int8_t chassis_mode = 0;  // 底盘运动模式

// 底盘分析相关
static int32_t chassis_position[CHASSIS_MODE_COUNT] = {0};
static int32_t chassis_velocity[CHASSIS_MODE_COUNT] = {0};

// =============================================================================
// 外部接口变量 (仅保留实际使用的)
// =============================================================================
// 射击相关常量
const float Kp = 0.7f;
const float Ki = 0.01f;
const float Kd = 0.1f;
const int16_t Dt = 1;
const int32_t I_threshold = 114514;
const int16_t Safety_threshold_shoot = 1000;
const int16_t Safety_threshold_chass = 3000;

// 底盘相关常量
const float kp = 5.0f;
const float ki = 0.01f;
const float kd = 0.0f;

// 射击相关变量
int32_t Integral_right = 0;
int32_t Integral_left = 0;
int16_t ls_right = 0;
int16_t ls_left = 0;
int16_t crt_right = 0;
int16_t crt_left = 0;

// 底盘相关变量
int32_t Integral[6] = {0};
int32_t goal_ch[4] = {0};
int32_t last_goal_ch[4] = {0};
int16_t ls[4] = {0};
int16_t crt[4] = {0};

// 串级PID相关变量 (仅保留外部使用的)
int32_t pos[4] = {0};                    // 用于调试输出
PID pid_position[4];                     // 位置PID控制器
PID pid_velocity[4];                     // 速度PID控制器
PID pid_location[3];                     // 底盘PID控制器

int32_t pid_total_part[3] = {0};         // 底盘位置积分
int32_t pid_veloc_part[3] = {0};         // 底盘速度输出
int32_t pid_target = 0;                  // 底盘目标值
int32_t pid_total = 0;                   // 总位置
int8_t tar[4] = {0};                     // 电机方向
int8_t flag = 0;                         // 控制模式标志
int8_t chassis_stat = 0;                 // 底盘状态

// 常量定义
const int32_t I_threshold_position = 10000;
const int32_t I_threshold_velocity = 3000;

// =============================================================================
// 工具函数
// =============================================================================
/**
 * @brief 计算绝对值
 * @param value 输入值
 * @return 绝对值
 */
static inline uint16_t abs_int16(int16_t value)
{
    return (value >= 0) ? (uint16_t)value : (uint16_t)(-value);
}

/**
 * @brief 限制值在指定范围内
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
static inline int32_t clamp_int32(int32_t value, int32_t min_val, int32_t max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
 * @brief 限制值在指定范围内
 * @param value 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
static inline int16_t clamp_int16(int16_t value, int16_t min_val, int16_t max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

// =============================================================================
// 位置计算函数 (优化后)
// =============================================================================
/**
 * @brief 更新电机位置信息
 * @param motor_id 电机ID (0-3)
 */
static void update_motor_position(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) return;
    
    // 计算编码器差值
    int16_t encoder_diff = motor_chassis[motor_id].ecd - motor_chassis[motor_id].last_ecd;
    
    // 处理编码器溢出，选择最小角度变化
    int16_t res1, res2;
    if (encoder_diff > 0) {
        res1 = encoder_diff - ENCODER_RESOLUTION;  // 反转
        res2 = encoder_diff;                       // 正转
    } else {
        res1 = encoder_diff + ENCODER_RESOLUTION;  // 正转
        res2 = encoder_diff;                       // 反转
    }
    
    // 选择角度变化较小的值
    encoder_diff = (abs_int16(res1) < abs_int16(res2)) ? res1 : res2;
    
    // 更新位置
    position_raw[motor_id] += encoder_diff;
    last_position_raw[motor_id] = position_raw[motor_id];
    
    // 转换为缩放位置 (避免浮点运算)
    position_scaled[motor_id] = (position_raw[motor_id] * POSITION_SCALE_FACTOR) / ENCODER_RESOLUTION;
    
    // 更新外部接口变量
    pos[motor_id] = position_scaled[motor_id];
}

// =============================================================================
// PID计算函数 (优化后)
// =============================================================================
/**
 * @brief 通用PID计算函数
 * @param pid PID控制器指针
 * @param actual_val 实际值
 * @param integral_limit 积分限制
 * @param deadzone 死区
 * @return PID输出
 */
static int16_t calculate_pid(PID *pid, int32_t actual_val, int32_t integral_limit, int16_t deadzone)
{
    // 计算误差
    int16_t error = (int16_t)(pid->target_val - actual_val);
    
    // 死区检查
    if (abs_int16(error) < deadzone) {
        return 0;
    }
    
    // 积分项计算
    pid->integral += error;
    pid->integral = clamp_int32(pid->integral, -integral_limit, integral_limit);
    
    // PID计算 (使用整数运算提高效率)
    int32_t p_term = (int32_t)(pid->Kp * error);
    int32_t i_term = (int32_t)(pid->Ki * pid->integral);
    int32_t d_term = (int32_t)(pid->Kd * (error - pid->err_last));
    
    int32_t output = p_term + i_term + d_term;
    pid->output_val = output;
    pid->err_last = error;
    
    return (int16_t)output;
}

/**
 * @brief 位置环PID计算
 * @param pid PID控制器指针
 * @param actual_val 实际位置
 * @return 目标速度(RPM)
 */
static int16_t calculate_position_pid(PID *pid, int32_t actual_val)
{
    int16_t velocity = calculate_pid(pid, actual_val, INTEGRAL_LIMIT_POSITION, POSITION_DEADZONE);
    return clamp_int16(velocity, -3000, 3000);
}

/**
 * @brief 速度环PID计算
 * @param pid PID控制器指针
 * @param actual_rpm 实际转速
 * @return 控制电流
 */
static int16_t calculate_velocity_pid(PID *pid, int16_t actual_rpm)
{
    int16_t current = calculate_pid(pid, actual_rpm, INTEGRAL_LIMIT_VELOCITY, 0);
    return clamp_int16(current, -SAFETY_THRESHOLD_CHASSIS, SAFETY_THRESHOLD_CHASSIS);
}

// =============================================================================
// 底盘分析函数 (优化后)
// =============================================================================
/**
 * @brief 底盘运动分析
 */
static void analyze_chassis_motion(void)
{
    // 计算各轮位置变化
    int16_t delta_pos[MOTOR_COUNT];
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        delta_pos[i] = position_scaled[i] - last_position_raw[i];
    }
    
    // 计算底盘运动分量 (麦克纳姆轮运动学)
    int16_t turn = delta_pos[0] + delta_pos[1] + delta_pos[2] + delta_pos[3];      // 转向
    int16_t right = -delta_pos[0] + delta_pos[1] + delta_pos[2] - delta_pos[3];   // 右移
    int16_t forward = -delta_pos[0] - delta_pos[1] + delta_pos[2] + delta_pos[3]; // 前进
    
    // 更新位置积分
    chassis_position[0] += turn / 4;
    chassis_position[1] += right / 4;
    chassis_position[2] += forward / 4;
    
    // 更新外部接口变量
    pid_total_part[0] = chassis_position[0];
    pid_total_part[1] = chassis_position[1];
    pid_total_part[2] = chassis_position[2];
    
    // 根据底盘模式设置目标
    switch (chassis_mode) {
        case 0: pid_location[0].target_val = pid_target; break;  // 转向
        case 1: pid_location[1].target_val = pid_target; break;  // 右移
        case 2: pid_location[2].target_val = pid_target; break;  // 前进
    }
    
    // 计算各方向速度
    for (uint8_t i = 0; i < CHASSIS_MODE_COUNT; i++) {
        chassis_velocity[i] = calculate_position_pid(&pid_location[i], chassis_position[i]);
        pid_veloc_part[i] = chassis_velocity[i];
    }
    
    // 计算各轮目标速度 (麦克纳姆轮逆运动学)
    int16_t target_velocity[MOTOR_COUNT];
    target_velocity[0] = -chassis_velocity[0] - chassis_velocity[1] + chassis_velocity[2];
    target_velocity[1] = +chassis_velocity[0] - chassis_velocity[1] + chassis_velocity[2];
    target_velocity[2] = +chassis_velocity[0] + chassis_velocity[1] + chassis_velocity[2];
    target_velocity[3] = -chassis_velocity[0] + chassis_velocity[1] + chassis_velocity[2];
    
    // 设置速度环目标
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pid_velocity[i].target_val = target_velocity[i];
    }
}

// =============================================================================
// 公共接口函数
// =============================================================================

void pid_shoot(void)
{
    int16_t speed_right = motor_chassis[4].speed_rpm;
    int16_t speed_left = motor_chassis[5].speed_rpm;
    
    int16_t error_right = goal - speed_right;
    int16_t error_left = -goal - speed_left;
    
    // 积分项
    integral_right += error_right / 2;
    integral_left += error_left / 2;
    integral_right = clamp_int32(integral_right, -INTEGRAL_LIMIT_CHASSIS, INTEGRAL_LIMIT_CHASSIS);
    integral_left = clamp_int32(integral_left, -INTEGRAL_LIMIT_CHASSIS, INTEGRAL_LIMIT_CHASSIS);
    
    // 微分项
    int16_t derivative_right = error_right - (last_goal - last_speed_right);
    int16_t derivative_left = error_left - (-last_goal - last_speed_left);
    
    last_goal = goal;
    last_speed_right = speed_right;
    last_speed_left = speed_left;
    
    // PID输出
    float output_right = Kp_shoot * error_right + Ki_shoot * integral_right - Kd_shoot * derivative_right;
    float output_left = Kp_shoot * error_left + Ki_shoot * integral_left - Kd_shoot * derivative_left;
    
    current_right = clamp_int16((int16_t)output_right, -SAFETY_THRESHOLD_SHOOT, SAFETY_THRESHOLD_SHOOT);
    current_left = clamp_int16((int16_t)output_left, -SAFETY_THRESHOLD_SHOOT, SAFETY_THRESHOLD_SHOOT);
    
    // 更新外部接口变量
    Integral_right = integral_right;
    Integral_left = integral_left;
    ls_right = last_speed_right;
    ls_left = last_speed_left;
    crt_right = current_right;
    crt_left = current_left;
    
    CAN_cmd_shoot(current_right, current_left);
}

void pid_chassis(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int16_t speed = motor_chassis[i].speed_rpm;
        int16_t error = goal_chassis[i] - speed;
        
        integral_chassis[i] += error / 2;
        integral_chassis[i] = clamp_int32(integral_chassis[i], -INTEGRAL_LIMIT_CHASSIS, INTEGRAL_LIMIT_CHASSIS);
        
        int16_t derivative = (last_goal_chassis[i] - last_speed_chassis[i]) - (goal_chassis[i] - speed);
        last_goal_chassis[i] = goal_chassis[i];
        last_speed_chassis[i] = speed;
        
        float output = Kp_chassis * error + Ki_chassis * integral_chassis[i] + Kd_chassis * derivative;
        current_chassis[i] = clamp_int16((int16_t)output, -SAFETY_THRESHOLD_CHASSIS, SAFETY_THRESHOLD_CHASSIS);
        
        // 更新外部接口变量
        Integral[i] = integral_chassis[i];
        goal_ch[i] = goal_chassis[i];
        last_goal_ch[i] = last_goal_chassis[i];
        ls[i] = last_speed_chassis[i];
        crt[i] = current_chassis[i];
    }
    CAN_cmd_chassis(current_chassis[0], current_chassis[1], current_chassis[2], current_chassis[3]);
}

// =============================================================================
// 外部接口函数 (仅保留实际使用的)
// =============================================================================
uint16_t ABS(int16_t a)
{
    return abs_int16(a);
}

void get_angle(int8_t i)
{
    update_motor_position(i);
}

/**
 * @brief 串级PID初始化
 */
void pid_angle_init(void)
{
    // 初始化位置PID
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pid_position[i].target_val = 0;
        pid_position[i].output_val = 0;
        pid_position[i].err = 0;
        pid_position[i].err_last = 0;
        pid_position[i].integral = 0;
        pid_position[i].Kp = 2.0f;
        pid_position[i].Ki = 0.1f;
        pid_position[i].Kd = 2.0f;
        
        // 初始化速度PID
        pid_velocity[i].target_val = 0;
        pid_velocity[i].output_val = 0;
        pid_velocity[i].err = 0;
        pid_velocity[i].err_last = 0;
        pid_velocity[i].integral = 0;
        pid_velocity[i].Kp = 2.0f;
        pid_velocity[i].Ki = 0.1f;
        pid_velocity[i].Kd = 0.5f;
        
        // 初始化位置变量
        position_raw[i] = 0;
        position_scaled[i] = 0;
        last_position_raw[i] = 0;
        
        // 初始化外部接口变量
        pos[i] = 0;
        tar[i] = 0;
    }

    // 初始化底盘PID
    for (uint8_t i = 0; i < CHASSIS_MODE_COUNT; i++) {
        pid_location[i].target_val = 0;
        pid_location[i].output_val = 0;
        pid_location[i].err = 0;
        pid_location[i].err_last = 0;
        pid_location[i].integral = 0;
        pid_location[i].Kp = 2.0f;
        pid_location[i].Ki = 0.1f;
        pid_location[i].Kd = 0.5f;
        
        chassis_position[i] = 0;
        chassis_velocity[i] = 0;
        pid_total_part[i] = 0;
        pid_veloc_part[i] = 0;
    }
    

    // 初始化控制状态
    pid_target = 0;
    pid_total = 0;
    control_mode = 0;
    chassis_mode = 0;
    flag = 0;
    chassis_stat = 0;
}

/**
 * @brief 串级PID控制主函数
 */
void pid_angle(void)
{
    // 更新所有电机位置
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        update_motor_position(i);
    }
    
    // 根据控制模式选择控制策略
    if (control_mode == 1) {
        analyze_chassis_motion();
    } else {
        // 位置控制模式
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            int16_t target_velocity = calculate_position_pid(&pid_position[i], position_scaled[i]);
            pid_velocity[i].target_val = target_velocity;
        }
    }
    
    // 计算速度环输出
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int16_t current_output = calculate_velocity_pid(&pid_velocity[i], motor_chassis[i].speed_rpm);
        current_chassis[i] = current_output;
        crt[i] = current_output;  // 更新外部接口变量
    }
    
    // 更新外部接口变量
    pid_total = 0;
    flag = control_mode;
    chassis_stat = chassis_mode;
}



// =============================================================================
// 兼容性函数实现 (仅保留实际使用的)
// =============================================================================
// 注意：以下函数没有外部调用，已移除：
// - pid_calculate_position() - 仅内部使用
// - pid_calculate_velocity() - 仅内部使用  
// - set_pid_target() - 仅内部使用
// - pid_angle_update_position() - 仅内部使用

/**
 * @brief 检查PID是否到达目标
 * @return 1:到达目标, 0:未到达目标
 */
int8_t check_pid(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        int32_t position_error = pid_position[i].target_val - position_scaled[i];
        if (abs_int16((int16_t)position_error) > 50) {
            return 0;
        }
        if (abs_int16(motor_chassis[i].speed_rpm) > VELOCITY_DEADZONE) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief 清理PID状态
 */
void clean_pid(void)
{
    pid_angle_init();
}

/**
 * @brief 底盘运动分析 (兼容性函数)
 */
void chassis_analyse(void)
{
    analyze_chassis_motion();
}

// =============================================================================
// 新增控制接口函数
// =============================================================================
/**
 * @brief 设置控制模式
 * @param mode 0:位置控制模式, 1:底盘分析模式
 */
void set_control_mode(int8_t mode)
{
    control_mode = mode;
    flag = mode;  // 更新外部接口变量
}

/**
 * @brief 设置底盘运动模式
 * @param mode 0:转向, 1:右移, 2:前进
 */
void set_chassis_mode(int8_t mode)
{
    chassis_mode = mode;
    chassis_stat = mode;  // 更新外部接口变量
}

/**
 * @brief 设置位置目标
 * @param motor_id 电机ID (0-3)
 * @param target 目标位置
 */
void set_position_target(int8_t motor_id, int32_t target)
{
    if (motor_id >= 0 && motor_id < MOTOR_COUNT) {
        pid_position[motor_id].target_val = target;
    }
}

/**
 * @brief 设置底盘目标
 * @param target 目标值
 */
void set_chassis_target(int32_t target)
{
    pid_target = target;
}
