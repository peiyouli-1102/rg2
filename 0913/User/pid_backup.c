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

//shoot part
//const int16_t Upside_rpm=4000;
const float Kp=0.7;
const float Ki=0.01;
const float Kd=0.1;

const int16_t Dt=1;

int32_t goal = 0 ;
int32_t last_goal = 0 ;
int32_t Integral_right = 0 ;
int32_t Integral_left = 0 ;
int16_t ls_right = 0 ;			//last speed
int16_t ls_left = 0 ;
int16_t crt_right = 0 ; 		//current
int16_t crt_left = 0 ;

const int32_t I_threshold=114514;

const int16_t Safety_threshold_shoot=1000;
const int16_t Safety_threshold_chass=3000;

const float kp = 5 ;
const float ki = 0.01 ;
const float kd = 0.0 ;

int32_t Integral[6]={0};
int32_t goal_ch[4]={0};
int32_t last_goal_ch[4]={0};
int16_t ls[4]={0};

int16_t crt[4] ={0};

// 位置-速度串级PID相关变量

int64_t ls_pos[4] = {0};
int64_t postn[4] = {0};
int32_t pos[4] = {0};

const int32_t I_threshold_position=10000;
const int32_t I_threshold_velocity=3000;
// 编码器分辨率定义
#define TOTAL_RESOLUTION 8192  // 编码器总分辨率

int32_t target[4] = {0};        // 目标位置
PID pid_position[4];            // 位置PID控制器
PID pid_velocity[4];            // 速度PID控制器

PID pid_location[3];
PID pid_speed;

int32_t pid_total_part[3] ;
int32_t pid_veloc_part[3] ;

int32_t pid_target = 0 , pid_total = 0;
int8_t tar[4] = {0};
int16_t pid_turn = 0;
int8_t flag = 0;

int8_t chassis_stat ;

void pid_shoot(){

	  int16_t speed_right = motor_chassis[4].speed_rpm;
	  int16_t speed_left  = motor_chassis[5].speed_rpm;//get rpm info

	  int16_t differ_right=goal-speed_right;
	  int16_t differ_left =-goal-speed_left;

	  Integral_right+=differ_right/2;
	  Integral_left +=differ_left /2;
	  if(Integral_right>I_threshold)Integral_right=I_threshold;
	  if(Integral_left >I_threshold)Integral_left =I_threshold;
	  if(-Integral_right>I_threshold)Integral_right=-I_threshold;
	  if(-Integral_left >I_threshold)Integral_left =-I_threshold;

	  int16_t divirative_right=(goal-speed_right)-((last_goal)-ls_right);
	  int16_t divirative_left =(-goal-speed_left)-(-(last_goal)-ls_left);

	  last_goal=goal;

	  float k_right=(float)divirative_right/(float)Dt;
	  float k_left =(float)divirative_left /(float)Dt;

	  ls_right=speed_right;
	  ls_left =speed_left ;

	  float output_right = Kp * differ_right + Ki * Integral_right - Kd * k_right;
	  float output_left  = Kp * differ_left  + Ki * Integral_left  - Kd * k_left ;

	  crt_right = (int16_t)output_right;
	  crt_left  = (int16_t)output_left;

	  if(crt_right>Safety_threshold_shoot)crt_right=Safety_threshold_shoot;
	  if(crt_left >Safety_threshold_shoot)crt_left =Safety_threshold_shoot;
	  if(crt_right<-Safety_threshold_shoot)crt_right=-Safety_threshold_shoot;
	  if(crt_left <-Safety_threshold_shoot)crt_left =-Safety_threshold_shoot;

	  CAN_cmd_shoot(crt_right, crt_left);
}

void pid_chassis(){
    for(int8_t i = 0; i < 4 ; i ++){
        int16_t speed = motor_chassis[i].speed_rpm;//get rpm info
        int16_t differ=goal_ch[i]-speed;

        Integral[i]+=differ/2;

        if(Integral[i]>I_threshold)Integral[i]=I_threshold;
        if(-Integral[i]>I_threshold)Integral[i]=-I_threshold;

        int16_t divirative=((last_goal_ch[i])-ls[i])-(goal_ch[i]-speed);

          last_goal_ch[i]=goal_ch[i];
          float k=(float)divirative/(float)Dt;
        ls[i]=speed;

        float output = kp * differ + ki * Integral[i] + kd * k;
        crt[i] = (int16_t)output;

        if(crt[i] > Safety_threshold_chass) crt[i] = Safety_threshold_chass;
        if(crt[i] <-Safety_threshold_chass) crt[i] =-Safety_threshold_chass;

    }
    CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

//以下是串级

uint16_t ABS(int16_t a){
	if(a>0)return a ;
	if(a<0)return (uint16_t)(-a) ;
	return 0 ;
}

void get_angle(int8_t i){
	int16_t  res1, res2;
	int16_t  error ;

	error =  motor_chassis[i].ecd - motor_chassis[i].last_ecd ;

	if(error>0){
		res1 = error - 8192;//反转，自减
		res2 = error ;
	}else{
		res1 = error + 8192;//正转，自加一个周期的角度值（360）
		res2 = error ;
	}

	if(ABS(res1)<ABS(res2)){ //不管正反转，肯定是转的角度小的那个是真的
		error = res1;
	}else{
		error = res2;
	}
	postn[i] += (int64_t)error;
	ls_pos[i] = postn[i];

    pos[i] = (int32_t)(postn[i] * 100 / 8192); // 避免浮点运算
}

// 位置-速度串级PID初始化
void pid_angle_init(void)
{
    for(int8_t i = 0; i < 4; i++)
    {
        // 位置PID参数初始化
        pid_position[i].target_val = 0;
        pid_position[i].output_val = 0;
        pid_position[i].err = 0;
        pid_position[i].err_last = 0;
        pid_position[i].integral = 0;
        pid_position[i].Kp = 2.0f;    // 位置环比例系数
        pid_position[i].Ki = 0.1f;   // 位置环积分系数
        pid_position[i].Kd = 2.0f;    // 位置环微分系数
        
        // 速度PID参数初始化
        pid_velocity[i].target_val = 0;
        pid_velocity[i].output_val = 0;
        pid_velocity[i].err = 0;
        pid_velocity[i].err_last = 0;
        pid_velocity[i].integral = 0;
        pid_velocity[i].Kp = 2.0f;   // 速度环比例系数
        pid_velocity[i].Ki = 0.1f;    // 速度环积分系数
        pid_velocity[i].Kd = 0.5f;    // 速度环微分系数
        
        // 位置相关变量初始化
        pos[i] = 0;
        postn[i] = 0;
        ls_pos[i] = 0;
        target[i] = 0;
        tar[i] = 0;
    }
    pid_target = 0 ;
    pid_total = 0 ;
    pid_turn = 0 ;
    flag = 0 ;

    /* 位置相关初始化参数 */
    for(int8_t i = 0 ; i < 3 ; i++){
        pid_location[i].target_val = 0;
        pid_location[i].output_val = 0;
        pid_location[i].err = 0;
        pid_location[i].err_last = 0;
        pid_location[i].integral = 0;
     
        pid_location[i].Kp = 2.0f;
        pid_location[i].Ki = 0.1f;
        pid_location[i].Kd = 0.5f;
    }

 
    /* 速度相关初始化参数 */
    pid_speed.target_val=0;
    pid_speed.output_val=0;
    pid_speed.err=0;
    pid_speed.err_last=0;
    pid_speed.integral=0;
 
    pid_speed.Kp = 2.0f;
    pid_speed.Ki = 0.1f;
    pid_speed.Kd = 0.5f;

    pid_total_part[0] = pid_total_part[1] = pid_total_part[2] = 0 ;
    // turn forward right
}

// 位置-速度串级PID控制
void pid_angle(void)
{
	pid_total = 0 ;
	pid_turn = 0 ;
    for(int8_t i = 0; i < 4; i++)
    {
        // 1. 更新位置信息
        pid_angle_update_position(i);
    }
    if(flag == 1){
        chassis_analyse();
    }
    for(int8_t i = 0; i < 4; i++)
    {
        if(flag == 1){
            
        }else {
            // 2. 位置环PID计算，输出目标速度(RPM)
            int16_t target_velocity = pid_calculate_position(&pid_position[i], pos[i]);

            // 3. 设置速度环目标值
            set_pid_target(&pid_velocity[i], target_velocity);
        }
        
        // 4. 速度环PID计算，输出控制电流
        int16_t current_output = pid_calculate_velocity(&pid_velocity[i], motor_chassis[i].speed_rpm);
        
        // 5. 限制输出电流
        if(current_output > Safety_threshold_chass)
            current_output = Safety_threshold_chass;
        else if(current_output < -Safety_threshold_chass)
            current_output = -Safety_threshold_chass;
        
        // 6. 更新控制输出
        crt[i] = current_output;
    }
}



// 位置环PID计算函数 - 输出RPM值
int16_t pid_calculate_position(PID *pid, int32_t actual_val)
{
    // 计算误差
    pid->err = (int16_t)(pid->target_val - actual_val);
    
    if(ABS(pid->err)< 10){
    	return 0 ;
    }

    // 积分项计算
    pid->integral += (int32_t)pid->err;
    
    // 积分限幅
    if(pid->integral > I_threshold_position)
        pid->integral = I_threshold_position;
    else if(pid->integral < - I_threshold_position)
        pid->integral = - I_threshold_position;
    
    // PID输出计算 (使用float进行精确计算)
    float p_term = pid->Kp * (float)pid->err;
    float i_term = pid->Ki * (float)pid->integral;
    float d_term = pid->Kd * (float)(pid->err - pid->err_last);
    float output = p_term + i_term + d_term;
    
    pid->output_val = (int32_t)output;
    
    // 更新上次误差
    pid->err_last = pid->err;
    
    // 限制RPM输出范围在-4000到4000之间
    int16_t rpm_output = (int16_t)pid->output_val;
    if(rpm_output > I_threshold_velocity)
        rpm_output = I_threshold_velocity;
    else if(rpm_output < - I_threshold_velocity)
        rpm_output = - I_threshold_velocity;
    
    return rpm_output;
}

// 速度环PID计算函数 - 输出控制电流
int16_t pid_calculate_velocity(PID *pid, int16_t actual_rpm)
{
    // 计算误差
    pid->err = (int16_t)(pid->target_val - actual_rpm);
    
    // 积分项计算
    pid->integral += (int32_t)pid->err;
    
    // 积分限幅
    if(pid->integral > 1000)
        pid->integral = 1000;
    else if(pid->integral < -1000)
        pid->integral = -1000;
    
    // PID输出计算 (使用float进行精确计算)
    float p_term = pid->Kp * (float)pid->err;
    float i_term = pid->Ki * (float)pid->integral;
    float d_term = pid->Kd * (float)(pid->err - pid->err_last);
    float output = p_term + i_term + d_term;
    
    pid->output_val = (int32_t)output;
    
    // 更新上次误差
    pid->err_last = pid->err;
    
    // 转换为整数输出
    int16_t current_output = (int16_t)pid->output_val;
    
    return current_output;
}

// 设置PID目标值
void set_pid_target(PID *pid, int16_t target_val)
{
    pid->target_val = (int32_t)target_val;
}

// 更新电机位置信息
void pid_angle_update_position(int8_t motor_id)
{
    if(motor_id >= 0 && motor_id < 4)
    {
        get_angle(motor_id);

        pid_total += pos[motor_id] * tar[motor_id];
    }
}

int8_t check_pid(void){
	for(int i = 0 ; i < 4 ; i++){
		if(pid_position[i].target_val - pos[i] > 50 ){
			return 0 ;
		}
		if(pid_position[i].target_val - pos[i] < -50 ){
			return 0 ;
		}
		if(motor_chassis[i].speed_rpm > 100){
			return 0 ;
		}
		if(motor_chassis[i].speed_rpm < -100){
			return 0 ;
		}
	}
	return 1 ;
}

void clean_pid(void){
    for(int8_t i = 0; i < 4; i++)
    {
        // 位置PID参数初始化
        pid_position[i].target_val = 0;
        pid_position[i].output_val = 0;
        pid_position[i].err = 0;
        pid_position[i].err_last = 0;
        pid_position[i].integral = 0;

        // 速度PID参数初始化
        pid_velocity[i].target_val = 0;
        pid_velocity[i].output_val = 0;
        pid_velocity[i].err = 0;
        pid_velocity[i].err_last = 0;
        pid_velocity[i].integral = 0;

        // 位置相关变量初始化
        pos[i] = 0;
        postn[i] = 0;
        ls_pos[i] = 0;
        target[i] = 0;
        tar[i] = 0;
    }
    pid_target = 0 ;
    pid_total = 0 ;
    pid_turn = 0 ;
    flag = 0 ;
    for(int8_t i = 0 ; i < 3 ; i++){
        pid_location[i].target_val = 0;
        pid_location[i].output_val = 0;
        pid_location[i].err = 0;
        pid_location[i].err_last = 0;
        pid_location[i].integral = 0;
     
        pid_location[i].Kp = 2.0f;
        pid_location[i].Ki = 0.1f;
        pid_location[i].Kd = 0.5f;
    }
}

void chassis_analyse(void){
    // pid_total /= 4 ;
    // for(int8_t i = 0; i < 4; i++){
    // 	pid_turn += pos[i] - tar[i]* pid_total ;
    // }
    // pid_turn /= 4 ;
    // pid_turn = (pid_turn < 100)? pid_turn : 100 ;
    // pid_turn = (pid_turn > -100)? pid_turn : -100 ;
    int16_t x_1, x_2, x_3, x_4, total, vol;
    x_1 = pos[0]-ls_pos[0];
    x_2 = pos[1]-ls_pos[1];
    x_3 = pos[2]-ls_pos[2];
    x_4 = pos[3]-ls_pos[3];
    total = x_1 + x_2 + x_3 + x_4 ;         //turn -z
    pid_total_part[0] += total / 4 ;
    total = - x_1 + x_2 + x_3 - x_4 ;       //right -x
    pid_total_part[1] += total / 4 ;
    total = - x_1 - x_2 + x_3 + x_4 ;       //forward -y
    pid_total_part[2] += total / 4 ;


    switch(chassis_stat){
        case 0 :
            pid_location[0].target_val = pid_target ;
            break ;
        case 1 :
            pid_location[1].target_val = pid_target ;
            break ;
        case 2 :
            pid_location[2].target_val = pid_target ;
            break ;
    }

    for(int8_t i = 0; i < 3; i++){
        pid_veloc_part[i] = pid_calculate_position(&pid_location[i], pid_total_part[i]);
    }
    vol = - pid_veloc_part[0] - pid_veloc_part[1] + pid_veloc_part[2] ;
    set_pid_target(&pid_velocity[0], vol);
    vol = + pid_veloc_part[0] - pid_veloc_part[1] + pid_veloc_part[2] ;
    set_pid_target(&pid_velocity[1], vol);
    vol = + pid_veloc_part[0] + pid_veloc_part[1] + pid_veloc_part[2] ;
    set_pid_target(&pid_velocity[2], vol);
    vol = - pid_veloc_part[0] + pid_veloc_part[1] + pid_veloc_part[2] ;
    set_pid_target(&pid_velocity[3], vol);
}
