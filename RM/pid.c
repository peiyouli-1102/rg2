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

const int16_t Upside_rpm=4000;
const float Kp=0.7;
//0.06  3442
//0.07  2508 2884
//0.08  2384 2158 2158
//0.09  1950:3360  1982:3364
//0.1   1732:2716  1750:2816
//0.12  1392:2114
//0.15  1176:1858
//0.2   946:1352
//0.3	812:1080
//0.5   746:896
//0.7
//0.8   288:442
//1     754:870	268:370 270:388
//2  	750:840 246:294
//5     262:288

const float Ki=0.01;
//0.04  5000 2300
//0.02  4500 2700
//0.015 4300 2800
//0.013 4118 2902
//0.012 4049 2912
//0.011 3975 2925
//0.01  3890 2946
//0.0095 3854 2950

const float Kd=0.1;
//0.1
//0.01 3888  2944
//0.04 3854  2950

const float kp = 5 ;
const float ki = 0.01 ;
const float kd = 0.0 ;

const int32_t P_threshold=114514;
const int32_t I_threshold=114514;
const int32_t D_threshold=114514;

//const float Speeding_rate=10;
//const int16_t Zero=100;
const int16_t Zero=0;
const int16_t Dt=2;
//const int16_t Safety_threshold=400;
const int16_t Safety_threshold_shoot=1000;
const int16_t Safety_threshold_chass=3000;

int32_t Integral_right=0;
int32_t Integral_left=0;
int32_t Integral[6]={0};

int32_t goal=0;
int32_t goal_ch[4]={0};
int32_t last_goal=0;
int32_t last_goal_ch[4]={0};

int16_t ls_right=0;//last speed
int16_t ls_left=0;
int16_t ls[4]={0};
int16_t crt_right=0;//current
int16_t crt_left=0;
int16_t crt[4] ={0};

int16_t ls_pos[4] = {0};
int16_t pos[4] = {0};

// the first version

int16_t shoot_pid(int16_t spgiven, int16_t spreal, int16_t rpm, int16_t goal){
	int16_t sptarg;
	int16_t error, error2;
	error = rpm - goal ;
	error2 = error/3 ;
	if((error > -50)&(error < 50)){
		return 0 ;
	}else if((error > -100)&(error < 100)){
		sptarg = - error2 ;
	}else if((error > -1000)&(error < 1000)){
		sptarg = - 2*error2 ;
	}else{
		sptarg = - 3*error2 ;
	}
	sptarg = (sptarg > 1000) ? 1000 : sptarg ;
	sptarg = (sptarg < -1000) ? -1000 : sptarg ;
	return sptarg;
}


void pid_shoot(){
//	  goal=goal>=Upside_rpm?Upside_rpm:goal+Speeding_rate*Dt;//Speed up!
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
	pos[i] += error;
	ls_pos[i] = pos[i];
}
void pid_chassis(){
	  for(int8_t i = 0; i < 4 ; i ++){
//		  goal[i]=(goal[i]>=Upside_rpm)?Upside_rpm:goal[i]+Speeding_rate*Dt;//Speed up!
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

//void pid_angle(void){
//	for(int8_t i = 0; i < 4 ; i ++){
//		get_angle(i);
//		int16_t error = target[i] - pos[i] ;
//		int16_t diff
//	}
//}
//
///*定义位置PID与速度PID结构体型的全局变量*/
//PID pid_location;
//PID pid_speed;
//
///**
//  * @brief  PID参数初始化
//  * @note   无
//  * @retval 无
//  */
//void PID_param_init()
//{
//    /* 位置相关初始化参数 */
//    pid_location.target_val = TOTAL_RESOLUTION*10;
//    pid_location.output_val = 0.0;
//    pid_location.err = 0.0;
//    pid_location.err_last = 0.0;
//    pid_location.integral = 0.0;
//
//    pid_location.Kp = 0.05;
//    pid_location.Ki = 0;
//    pid_location.Kd = 0;
//
//    /* 速度相关初始化参数 */
//    pid_speed.target_val=10.0;
//    pid_speed.output_val=0.0;
//    pid_speed.err=0.0;
//    pid_speed.err_last=0.0;
//    pid_speed.integral=0.0;
//
//    pid_speed.Kp = 80.0;
//    pid_speed.Ki = 2.0;
//    pid_speed.Kd = 100.0;
//}
///**
//  * @brief  位置PID算法实现
//  * @param  actual_val:实际值
//  * @note   无
//  * @retval 通过PID计算后的输出
//  */
//#define LOC_DEAD_ZONE 60 /*位置环死区*/
//#define LOC_INTEGRAL_START_ERR 200 /*积分分离时对应的误差范围*/
//#define LOC_INTEGRAL_MAX_VAL 800   /*积分范围限定，防止积分饱和*/
//float location_pid_realize(PID *pid, float actual_val)
//{
//    /*计算目标值与实际值的误差*/
//    pid->err = pid->target_val - actual_val;
//
//    /* 设定闭环死区 */
//    if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }
//
//    /*积分项，积分分离，偏差较大时去掉积分作用*/
//    if(pid->err > -LOC_INTEGRAL_START_ERR && pid->err < LOC_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;
//        /*积分范围限定，防止积分饱和*/
//        if(pid->integral > LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = LOC_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -LOC_INTEGRAL_MAX_VAL;
//        }
//    }
//
//    /*PID算法实现*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd * (pid->err - pid->err_last);
//
//    /*误差传递*/
//    pid->err_last = pid->err;
//
//    /*返回当前实际值*/
//    return pid->output_val;
//}
//
///**
//  * @brief  速度PID算法实现
//  * @param  actual_val:实际值
//  * @note   无
//  * @retval 通过PID计算后的输出
//  */
//#define SPE_DEAD_ZONE 5.0f /*速度环死区*/
//#define SPE_INTEGRAL_START_ERR 100 /*积分分离时对应的误差范围*/
//#define SPE_INTEGRAL_MAX_VAL 260   /*积分范围限定，防止积分饱和*/
//float speed_pid_realize(PID *pid, float actual_val)
//{
//    /*计算目标值与实际值的误差*/
//    pid->err = pid->target_val - actual_val;
//
//    /* 设定闭环死区 */
//    if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
//    {
//        pid->err = 0;
//        pid->integral = 0;
//        pid->err_last = 0;
//    }
//
//    /*积分项，积分分离，偏差较大时去掉积分作用*/
//    if(pid->err > -SPE_INTEGRAL_START_ERR && pid->err < SPE_INTEGRAL_START_ERR)
//    {
//        pid->integral += pid->err;
//        /*积分范围限定，防止积分饱和*/
//        if(pid->integral > SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = SPE_INTEGRAL_MAX_VAL;
//        }
//        else if(pid->integral < -SPE_INTEGRAL_MAX_VAL)
//        {
//            pid->integral = -SPE_INTEGRAL_MAX_VAL;
//        }
//    }
//
//    /*PID算法实现*/
//    pid->output_val = pid->Kp * pid->err +
//                      pid->Ki * pid->integral +
//                      pid->Kd *(pid->err - pid->err_last);
//
//    /*误差传递*/
//    pid->err_last = pid->err;
//
//    /*返回当前实际值*/
//    return pid->output_val;
//}
////周期定时器的回调函数
//void AutoReloadCallback()
//{
//    static uint32_t location_timer = 0;    // 位置环周期
//
//    static __IO int encoderNow = 0;    /*当前时刻总计数值*/
//    static __IO int encoderLast = 0;   /*上一时刻总计数值*/
//    int encoderDelta = 0; /*当前时刻与上一时刻编码器的变化量*/
//    float actual_speed = 0;  /*实际测得速度*/
//    int actual_speed_int = 0;
//
//    int res_pwm = 0;/*PID计算得到的PWM值*/
//    static int i=0;
//
//    /*【1】读取编码器的值*/
//    encoderNow = read_encoder() + EncoderOverflowCnt*ENCODER_TIM_PERIOD;/*获取当前的累计值*/
//    encoderDelta = encoderNow - encoderLast; /*得到变化值*/
//    encoderLast = encoderNow;/*更新上次的累计值*/
//
//    /*【2】位置PID运算，得到PWM控制值*/
//    if ((location_timer++ % 2) == 0)
//    {
//        float control_val = 0;   /*当前控制值*/
//
//        /*位置PID计算*/
//        control_val = location_pid_realize(&pid_location, encoderNow);
//
//        /*目标速度值限制*/
//        speed_val_protect(&control_val);
//
//        /*设定速度PID的目标值*/
//        set_pid_target(&pid_speed, control_val);
//    }
//
//    /* 转速(1秒钟转多少圈)=单位时间内的计数值/总分辨率*时间系数, 再乘60变为1分钟转多少圈 */
//    actual_speed = (float)encoderDelta / TOTAL_RESOLUTION * 10 * 60;
//
//    /*【3】速度PID运算，得到PWM控制值*/
//    actual_speed_int = actual_speed;
//    res_pwm = pwm_val_protect((int)speed_pid_realize(&pid_speed, actual_speed));
//
//    /*【4】PWM控制电机*/
//    set_motor_rotate(res_pwm);
//
//    /*【5】数据上传到上位机显示*/
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &encoderNow, 1);   /*给通道1发送实际的电机【位置】值*/
//}
