#include "dsptch.h"

#define SRVO11 550		//557 555 558 570 555 550 557
#define SRVO12 290
#define SRVO13 480		//500

#define SRVO21 350		//360 368 358 360 370 360
#define SRVO22 570
#define SRVO23 380

#define SRVO31 265			//265
#define SRVO32 730			//740 730 720 730
#define SRVO33 1170			//1175 1350 1320 1160 1162 1168 1170
#define SRVO301 120		//490-265  120 130 145 1310-1160 = 130 120
#define SRVO302 100		//225 * 2/3

#define SRVO41 590			//580
#define SRVO42 790

// 定义全局变量
TaskSchdle Time;

extern uint32_t result;
extern uint32_t result2;

const uint8_t MAXTASK = 8;
int8_t get_stat = 0 ;
int8_t dart_num = 0 ;

uint8_t check(void){
	// 检查是否需要切换到下一个任务
	// Time.t 是当前应该执行的任务索引 (0-7)
	// Time.cur 是当前正在执行的任务索引
	if (Time.cur != Time.t){
		Time.cur = Time.t;
		return 1; // 需要执行新任务
	}
	return 0; // 继续执行当前任务
}

void dispatch(void){
	// 更新dispatch调用时间
	Time.last_dispatch_time = HAL_GetTick();
	// 确保任务索引在有效范围内
	if(Time.cur >= 0 && Time.cur < MAXTASK){
		struct Task* curtask = &Time.tasks[Time.cur];
		// 执行当前时间片的任务
		if(curtask->task_func == NULL){
			// 空任务槽，只更新状态
			curtask->stat = 0 ;
		} else {
			// 记录任务开始时间
			Time.task_start_time = HAL_GetTick();
			// 执行任务 - 每个任务在1/8ms内完成
			curtask->task_func();			
			// 检查任务执行时间
			uint32_t exec_time = HAL_GetTick() - Time.task_start_time;
			if(exec_time > Time.max_task_time) {
				curtask->stat = 2; // 使用状态2表示任务超时
			} else {
				curtask->stat = 1; // 正常完成
			}
		}
	} else {
		Time.cur = 0;
	}
}

// 初始化任务调度器
void Init_schdl(void) {
	Time.cur = -1;  // 当前执行的任务索引
	Time.ms = 0;    // 毫秒计数器
	Time.t = 0;     // 当前时间片索引 (0-7)
	Time.timer = -1; // 定时器值
	Time.task_start_time = 0;
	Time.max_task_time = 1; // 设置最大任务执行时间为1ms (每个时间片1/8ms)
	Time.last_dispatch_time = 0;
	Time.watchdog_timeout = 10; // 设置看门狗超时时间为10ms

    // 初始化8个时间片任务
	Time.tasks[0] = (struct Task){NULL, "none", 0, 0};      // 时间片0: 停止位
	Time.tasks[1] = (struct Task){_1mssg, "message", 1, 0}; // 时间片1: 消息处理
	Time.tasks[2] = (struct Task){NULL, "none", 0, 0};      // 时间片2: 空闲			为解决消息时间溢出问题
	Time.tasks[3] = (struct Task){_3pid, "pid", 1, 0};      // 时间片3: PID控制
	Time.tasks[4] = (struct Task){_4stp, "stop", 1, 0};    	// 时间片4: 底盘控制
	Time.tasks[5] = (struct Task){_5rtn, "return", 1, 0};   	// 时间片5: 舵机控制
	Time.tasks[6] = (struct Task){NULL, "none", 1, 0};      // 时间片6: 发射控制
	Time.tasks[7] = (struct Task){NULL, "none", 0, 0};      // 时间片7: 空闲

}

// 0 号位： 常闭

void _0STOP(void){
	for(uint8_t i = 0; i < MAXTASK; i++){
		Time.tasks[i] = (struct Task){NULL, "none" , 0} ;
	}
}

void _0Stop(uint8_t i){
	Time.tasks[i] = (struct Task){NULL, "none" , 0} ;
}

// 1 号位： 常开

void _1mssg(void){
	// 时间片1: 消息处理任务
	// 在1/8ms内完成，实现并发效果	
	// 快速检查所有任务状态
	for(uint8_t i = 0; i < MAXTASK; i++){
		if(Time.tasks[i].stat == 2){
			printf("%d\r\n",i);
//			printf("task %d is done unexpectively\r\n",i);
		}

	}
	if((Time.ms % 200) < 3){
		// printf("??%d\t%d  %d  %d  %d\t %d %d %d %d %d \t%d %d %d\r\n",
		// 		Time.ms,pos[0],pos[1],pos[2],pos[3],
		// 		pid_velocity[0].target_val,pid_velocity[1].target_val,
		// 		pid_velocity[2].target_val,pid_velocity[3].target_val,pid_target,
		// 		pid_location[0].err, pid_location[1].err, pid_location[2].err);
//		printf("%d \r\n",pid_total_part[0]);
//		printf("%d %d %d %d\r\n",Time.ms,pid_location[0].err, pid_location[1].err, pid_location[2].err);
//		printf("%d  %d  %d  %d  %d \r\n",Time.ms, motor_chassis[4].speed_rpm,motor_chassis[5].speed_rpm,crt_right,crt_left);
		printf("%d\r\n",Time.ms);
	}
}

// 3 号位： 常开

void _3pid(void){
	pid_angle();
}

// 4 号位： 常开

void _4fwd(void){			//forward
	set_control_mode(1); // 位置控制
	set_chassis_mode(2); // 前进
	set_chassis_target(result);
	set_position_target(0, -10 * result);
	set_position_target(1, 10 * result);
	set_position_target(2, 10 * result);
	set_position_target(3, -10 * result);
	_4();
}

void _4rgt(void){			//right
	set_control_mode(1); // 位置控制
	set_chassis_mode(1); // 右移
	set_chassis_target(result);
	set_position_target(0, -10 * result);
	set_position_target(1, -10 * result);
	set_position_target(2, 10 * result);
	set_position_target(3, 10 * result);
	_4();
}

void _4rnd(void){			//round,clock side
	set_control_mode(1); // 位置控制
	set_chassis_mode(0); // 转向
	set_chassis_target(result);
	set_position_target(0, 10 * result);
	set_position_target(1, 10 * result);
	set_position_target(2, 10 * result);
	set_position_target(3, 10 * result);
	_4();
}

void _4(void){
	if(Time.ms - Time.tasks[4].tim > 10){
		if(check_pid()){
			clean_pid();
			Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
			printf("don");
		}
		if(Time.ms - Time.tasks[4].tim > 3* ABS(result) + 800){
			clean_pid();
			Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
			printf("err");
		}
	}
    CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

void _4stp(void){
	set_control_mode(1); 		// 位置控制
	set_chassis_mode(0); 		// 转向
	set_chassis_target(0);
	set_position_target(0, 0);
	set_position_target(1, 0);
	set_position_target(2, 0);
	set_position_target(3, 0);
	// 由pid.c统一输出CAN_cmd_chassis
	CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

void _4upstr(void){
	if(Time.ms < Time.tasks[4].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[4].tim ;
	uint32_t uptime = 3000 , staytime = 100 , movetime = 1000 ;
	if(t < uptime ){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(t < uptime + staytime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	}else if(t < uptime + staytime + movetime){
		_4for(100);
	}else if(t < uptime + 2 * staytime + movetime){
		clean_pid();
	}else if(t < 2 *uptime + 2 * staytime + movetime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else if(t < 2 *uptime + 3 * staytime + movetime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	}else if(t < 2 *uptime + 3 * staytime + 2 * movetime){
		_4for(200);
	}else if(t < 2 *uptime + 4 * staytime + 2 * movetime){
		clean_pid();
	}else if(t < 3 *uptime + 4 * staytime + 2 * movetime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_SET);
	}else if(t < 3 *uptime + 5 * staytime + 2 * movetime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_RESET);
	}else if(t < 3 *uptime + 5 * staytime + 3 * movetime){
		_4for(300);
	}else if(t < 3 *uptime + 6 * staytime + 3 * movetime){
		clean_pid();
	}else{
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, 0};
		return ;
	}
}

void _4dwnstr(void){
	if(Time.ms < Time.tasks[4].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[4].tim ;
	uint32_t uptime = 3000 , staytime = 100 , movetime = 1000 ;

	if(t < staytime){
		clean_pid();
	}else if(t < staytime + movetime){
		_4for(-100);
	}else if(t < 2* staytime + movetime){
		clean_pid();
	}else if(t < 2* staytime + movetime + uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}else if(t < 3* staytime + movetime + uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	}else if(t < 3* staytime + 2* movetime + uptime){
		_4for(-180);
	}else if(t < 4* staytime + 2* movetime + uptime){
		clean_pid();
	}else if(t < 4* staytime + 2* movetime + 2* uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(t < 5* staytime + 2* movetime + 2* uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	}else if(t < 5* staytime + 3* movetime + 2* uptime){
		_4for(-90);
	}else if(t < 6* staytime + 3* movetime + 2* uptime){
		clean_pid();
	}else if(t < 6* staytime + 3* movetime + 3* uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else if(t < 7* staytime + 3* movetime + 3* uptime){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	}else if(t < 7* staytime + 4* movetime + 3* uptime){
		_4for(-150);
	}else{
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, 0};
		return ;
	}
}

void _4for(int16_t distance){
	set_control_mode(1); // 位置控制
	set_chassis_mode(2); // 前进
	set_chassis_target(distance);
	set_position_target(0, -10 * distance);
	set_position_target(1, 10 * distance);
	set_position_target(2, 10 * distance);
	set_position_target(3, -10 * distance);
//	if(check_pid()){
//		clean_pid();
//		return;
//	}
	CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}


// 5 号位： 常开

void _5get(void){
	uint32_t t;
	uint32_t pul1time = (get_stat == 1 )?8500 : 2800, puttime = 1000, movetime = 500, staytime = 1000, adds = 1000;
	int32_t k = (get_stat == 1)? -1 : 1 ;
	// 13000 : 13800 : 14000 : 14200 :14000
	//1200
	// 预计算相对时间，避免重复计算
	if (Time.ms < Time.tasks[5].tim){
		return ;
	}
	t = Time.ms - Time.tasks[5].tim;

	int16_t shk = 0;
	shk = ( ( t / 100 ) % 2) ? -30 : 30 ;
	uint16_t dart = SRVO31 ;
	switch(dart_num){
	case 0:
		dart = SRVO31 ;
		break;
	case 1:
		dart = SRVO32 ;
		break;
	case 2:
		dart = SRVO33 ;
		break;
	}

	if( t < staytime ){
		_5srvo(SRVO11, SRVO21, dart, SRVO41);
	}else if( t < staytime + movetime){
		_5move(SRVO11, SRVO21, dart, SRVO41, SRVO12, SRVO22, dart, SRVO41, t-staytime);
	}else if( t < 2* staytime + movetime){
		_5srvo(SRVO12 + shk, SRVO22, dart, SRVO41);
	}else if( t < 2* staytime + movetime + 2000){
		_5srvo(SRVO12 + shk, SRVO22, dart, SRVO41);
		_5othr( 1, 0 );
	}else if( t < 3* staytime + movetime + 2000){
		_5srvo(SRVO12, SRVO22, dart, SRVO41);
		_5othr( 0, 0 );
	}else if( t < 3* staytime + movetime + pul1time){
		_5srvo(SRVO12, SRVO22, dart, SRVO41);
		_5othr( 3, 1 );
	}else if( t < 4* staytime + movetime + pul1time){
		_5srvo(SRVO12, SRVO22, dart, SRVO41);
		_5othr( 0, 1 );
	}else if( t < 4* staytime + movetime + 3* pul1time + k* adds){
		_5srvo(SRVO12, SRVO22, dart, SRVO41);
		_5othr( 2, 1 );
	}else if( t < 5* staytime + movetime + 3* pul1time + k* adds){
		_5srvo(SRVO12, SRVO22, dart, SRVO41);
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 2* movetime + 3* pul1time + k* adds){
		_5move(SRVO12, SRVO22, dart, SRVO41, SRVO13, SRVO23, dart, SRVO41, t-(5* staytime + movetime + 3* pul1time + k* adds));
		_5othr( 0, 1 );
	}else if( t < 6* staytime + 2* movetime + 3* pul1time + k* adds){
		_5srvo(SRVO13, SRVO23, dart, SRVO41);
		_5othr( 0, 1 );
		if(get_stat == 1){
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
			result = 0 ;
		}else{
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
			result = -70 ;		//50 60 70
		}
	}else if( t < 7* staytime + 2* movetime + 3* pul1time + k* adds){
		_5srvo(SRVO13, SRVO23 + shk, dart, SRVO41);
		_5othr( 0, 1 );
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if( t < 7* staytime + 2* movetime + 3* pul1time + puttime + k* adds){
		_5srvo(SRVO13, SRVO23 + shk, dart, SRVO41);
		_5othr( 1, 1 );
	}else if( t < 7* staytime + 3* movetime + 3* pul1time + puttime + k* adds){
		_5move(SRVO13, SRVO23, dart, SRVO41, SRVO11, SRVO21, dart, SRVO41, t-(7* staytime + 2* movetime + 3* pul1time + puttime + k* adds));
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 3* movetime + 3* pul1time + puttime + k* adds){
		_5srvo(SRVO11, SRVO21, dart, SRVO41);
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 4* movetime + 3* pul1time + puttime + k* adds){
		_5move(SRVO11, SRVO21, dart, SRVO41, SRVO11, SRVO21, dart + SRVO301, SRVO41, t-(8* staytime + 3* movetime + 3* pul1time + puttime + k* adds));
		_5othr( 0, 1 );
	}else if( t < 10* staytime + 4* movetime + 3* pul1time + puttime + k* adds){
		_5srvo(SRVO11, SRVO21, dart + SRVO301, SRVO41);
		_5othr( 0, 2 );
	}else if( t < 10* staytime + 4* movetime + 3* pul1time + 3* puttime + k* adds){
		_5srvo(SRVO11, SRVO21, dart + SRVO301, SRVO41);
		_5othr( 1, 2 );
	}else if( t < 10* staytime + 5* movetime + 3* pul1time + 3* puttime + k* adds){
		_5move(SRVO11, SRVO21, dart + SRVO301, SRVO41, 500, 250, dart + SRVO301, SRVO41, t-(9* staytime + 5* movetime + 3* pul1time + 3* puttime + k* adds));
		_5othr( 0, 2 );
	}else if( t < 11* staytime + 5* movetime + 3* pul1time + 3* puttime + k* adds){
		_5srvo(500, 250, dart + SRVO301, SRVO41);
		_5othr( 0, 2 );
	}else if( t < 11* staytime + 5* movetime + 3* pul1time + 9* puttime + (k+1)* adds){
		_5srvo(500, 250, dart + SRVO301, SRVO41);
//		if(Time.flag == 0){
//			printf("don");
//			Time.flag = 1 ;
//		}
		_5othr( 2, 2 );
	}else if( t < 12* staytime + 5* movetime + 3* pul1time + 9* puttime + (k+1)* adds){
		_5srvo(SRVO11, SRVO21, dart + SRVO301, SRVO41);
		_5othr( 0, 0 );
	}else{
		Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
		dart_num ++ ;
		dart_num = dart_num % 3  ;
		printf("don");
	}
}

void _5put1(void){		
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 1000 , t_move = 500;
	uint32_t t1 = Time.tasks[5].tim ;

	if(t > 0 && t < t_stay){
		_5srvo(SRVO11,SRVO21,SRVO33,SRVO41);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(SRVO11,SRVO21,SRVO33,SRVO41, SRVO11,SRVO21,SRVO33,SRVO42, t1 + t_stay);
	}else if ( t < 2* t_stay + t_move){
		_5srvo(SRVO11,SRVO21,SRVO33,SRVO42);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty1, "stay1", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}

void _5put2(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 1000 , t_move = 500;
	uint32_t t1 = Time.tasks[5].tim ;

	if(t > 0 && t < t_stay){
		_5srvo(SRVO11,SRVO21,SRVO33,SRVO42);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(SRVO11,SRVO21,SRVO33,SRVO42,  SRVO11,SRVO21,SRVO32,SRVO42, t1 + t_stay);
	}else if ( t < 2* t_stay + t_move){
		_5srvo(SRVO11,SRVO21,SRVO32,SRVO42);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty2, "stay2", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}
void _5put3(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 1000 , t_move = 500;
	uint32_t t1 = Time.tasks[5].tim ;

	if(t > 0 && t < t_stay){
		_5srvo(SRVO11,SRVO21,SRVO32,SRVO42);
	}else if ( t < t_stay + t_move){
		_5move(SRVO11,SRVO21,SRVO32,SRVO42,  SRVO11,SRVO21,SRVO31,SRVO42, t1 + t_stay);
	}else if ( t < 2* t_stay + t_move){
		_5srvo(SRVO11,SRVO21,SRVO31,SRVO42);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty3, "stay3", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}

void _5rtn(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SRVO11);		//560	255
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SRVO21);		//390	570
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SRVO33);		//490
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, SRVO41);		//800
}

void _5rtn2(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SRVO11);		//490
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SRVO21);		//545
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SRVO33);		//500
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, SRVO41);		//800
}

//拾取 1 2 3
//发射 1 2 3

void _5sty1(void){
	_5srvo(SRVO11, SRVO21, SRVO33, SRVO42);
}

void _5sty2(void){
	_5srvo(SRVO11, SRVO21, SRVO32, SRVO42);
}

void _5sty3(void){
	_5srvo(SRVO11, SRVO21, SRVO31, SRVO42);
}

void _5test(void){
	uint16_t angle = 0 ;
	uint32_t t = Time.ms % 500;
	angle = (t* SRVO11 + ( 500 - t )* SRVO12)/500 ;
	_5srvo(angle, SRVO21, SRVO31, SRVO41);
}

void _5srvo(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle4);
}

void _5move(uint16_t angle11, uint16_t angle21, uint16_t angle31, uint16_t angle41, uint16_t angle12, uint16_t angle22, uint16_t angle32, uint16_t angle42, uint16_t t ){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle11+(angle12-angle11)*t/500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle21+(angle22-angle21)*t/500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle31+(angle32-angle31)*t/500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle41+(angle42-angle41)*t/500);
}

void _5othr(uint8_t state1, uint8_t state2){
	switch(state1){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);			//放线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);		//收线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			break;
		case 3:
			uint32_t t1 = Time.ms - Time.tasks[5].tim ;
			if( (t1/200)%6 == 0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);			//放线
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);				//放线
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

	}
	switch(state2){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			break;
	}
}


// 6 号位： 常闭

void _6shoot(void){

	if (Time.ms < Time.tasks[6].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[6].tim ; 
	if (t > 7200){					//10000
		Time.tasks[6] = (struct Task){NULL, "none", 0, Time.ms};
		return ;
	}
//	goal = 3830 ;
	goal = 2830 ;
//	goal = 2500 ;
//	goal = 3500 ;
	if( (result2 > 1000)&(result2 < 4000) ){goal = result2 ;}
	// 1) 射击电机输出窗口（低内存/低计算）
	if (t < 6000) {
		uint8_t ok = (check_motor(4) ? 1 : 0) + (check_motor(5) ? 2 : 0);
		if (ok == 3) {
			pid_shoot();
		} else {
			int16_t s1 = 600, s2 = -600;
			if (ok == 1) { s1 = 100; s2 = -600; }
			else if (ok == 2) { s1 = 600; s2 = -100; }
			CAN_cmd_shoot(s1, s2);
		}
	} else {
		CAN_cmd_shoot(0, 0);
	}

	// 2) LED 与 TIM4 CH1（只计算一次区间）
	uint8_t in_a = (t > 1000) && (t < 4045);//1000 4050 4230 7200
	uint8_t in_b = (!in_a) && (t >= 4200) && (t < 7195 );
	if (in_a) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 450);
	} else if (in_b) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 450);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}

	// 3) TIM3 CH1/CH2（一次布尔判断）
	uint8_t in_tim3 = (t > 2000) && (t < 6000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, in_tim3 ? 600u : 500u);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, in_tim3 ? 600u : 500u);
}

// 7 号位： 空闲

void _7step1(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;

	if(t < 300){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
	}else if(t < 600){
		Time.tasks[4] = (struct Task){_4upstr, "upstair", 1, t1+300};
	}
	if(t > 14000){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
		printf("step1don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
	}
}

void _7step2(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;

	if(t < 300){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
	}else if(t < 600){
		Time.tasks[4] = (struct Task){_4dwnstr, "downstair", 1, t1+300};
	}
	if(t > 15000){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
		printf("step2don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
	}
}

void _7shoot(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;
	uint32_t t_shoot = 7200 , t_put = 2500 , t_stop = 200;		//500

	// 关键节点时间点
	uint32_t node1 = 0;
	uint32_t node2 = t_stop;
	uint32_t node3 = t_stop + t_put + t_shoot;
	uint32_t node4 = 2 * t_stop + t_put + t_shoot;
	uint32_t node5 = 2 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node6 = 3 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node7 = 3 * t_stop + 3 * t_put + 3 * t_shoot;
	uint32_t node8 = 4 * t_stop + 3 * t_put + 3 * t_shoot;

	if(t < node2){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, t1};
		clean_pid();
		result = 0 ;
		if(t >= node1 && t < node1 + 200){
			Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
		}
	}else if(t < node3){
		if(t >= node2 && t < node2 + 200){
			Time.tasks[5] = (struct Task){_5put1, "put", 1, t1 + node2};
		}
	}else if(t < node4){
		if(t >= node3 && t < node3 + 200){
			Time.tasks[5] = (struct Task){_5sty1, "stay1", 1, Time.ms};
		}
	}else if (t < node5){
		if(t >= node4 && t < node4 + 200){
			Time.tasks[5] = (struct Task){_5put2, "put", 1, t1 + node4};
		}
	}else if (t < node6){
		if(t >= node5 && t < node5 + 200){
			Time.tasks[5] = (struct Task){_5sty2, "stay2", 1, Time.ms};
		}
	}else if (t < node7){
		if(t >= node6 && t < node6 + 200){
			Time.tasks[5] = (struct Task){_5put3, "put", 1, t1 + node6};
		}
	}else if (t < node8){
		if(t >= node7 && t < node7 + 200){
			Time.tasks[5] = (struct Task){_5sty3, "stay3", 1, Time.ms};
		}
	}else{
		dart_num = 0 ;
		Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};	
		printf("shootdon\r\n");
	}
}

// 看门狗检查函数
void watchdog_check(void){
	uint32_t current_time = HAL_GetTick();
	
	// 检查是否超过看门狗超时时间
	if((current_time - Time.last_dispatch_time) > Time.watchdog_timeout) {
		// 调度器可能卡死，执行恢复操作
		Time.cur = 0;  // 重置当前任务
		Time.last_dispatch_time = current_time;  // 更新看门狗时间
		
	}
}

uint8_t check_motor(uint8_t sign){
	int16_t speed ;
	speed = motor_chassis[sign].speed_rpm ;
	if(speed > 15){
		return 1 ;
	}
	if(speed < -15){
		return 1 ;
	}
	return 0 ;
}
