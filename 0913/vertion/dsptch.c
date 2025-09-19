#include "dsptch.h"

// 定义全局变量
TaskSchdle Time;

extern uint32_t result;

const uint8_t MAXTASK = 8;
int8_t get_stat = 0 ;

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
	Time.tasks[5] = (struct Task){_5sty1, "return", 1, 0};   	// 时间片5: 舵机控制
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
		printf("??%d\t%d  %d  %d  %d\t %d %d %d %d %d \t%d %d %d\r\n",
				Time.ms,pos[0],pos[1],pos[2],pos[3],
				pid_velocity[0].target_val,pid_velocity[1].target_val,
				pid_velocity[2].target_val,pid_velocity[3].target_val,pid_target,
				pid_location[0].err, pid_location[1].err, pid_location[2].err);
		printf("%d \r\n",pid_total_part[0]);
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
	if(Time.ms - Time.tasks[4].tim > 10){
		_4();
	}
}

void _4rgt(void){			//right
	set_control_mode(1); // 位置控制
	set_chassis_mode(1); // 右移
	set_chassis_target(result);
	set_position_target(0, -10 * result);
	set_position_target(1, -10 * result);
	set_position_target(2, 10 * result);
	set_position_target(3, 10 * result);
	if(Time.ms - Time.tasks[4].tim > 10){
		_4();
	}
}

void _4rnd(void){			//round,clock side
	set_control_mode(1); // 位置控制
	set_chassis_mode(0); // 转向
	set_chassis_target(result);
	set_position_target(0, 10 * result);
	set_position_target(1, 10 * result);
	set_position_target(2, 10 * result);
	set_position_target(3, 10 * result);
	if(Time.ms - Time.tasks[4].tim > 10){
		_4();
	}
}

void _4(void){
	if(check_pid()){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		printf("don");
	}
	if(Time.ms - Time.tasks[4].tim > 10 * ABS(result) + 1000){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		printf("err");
	}
    CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

void _4stp(void){
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
		_4for(100);
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
		_4for(-70);
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
	if(check_pid()){
		clean_pid();
	}
	CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

// 5 号位： 常开
void _5get(void){
	uint32_t t;
	uint32_t pul1time = (get_stat == 1 )?13900:3000, puttime = 1500, movetime = 500, staytime = 1000;
	// 13000
	// 预计算相对时间，避免重复计算
	if (Time.ms < Time.tasks[5].tim){
		return ;
	}
	t = Time.ms - Time.tasks[5].tim;

	//83
	int16_t a = 280 ,b = 0;
	b = ((t/100)%2)? -15 : 15 ;

    // 使用更高效的条件判断和预计算
	if( t < staytime ){
		_5srvo(490, 545, 645, 570);
	}else if( t < staytime + movetime){
		_5move(490, 545, 645, 570, 675, 280, 645, 570, t-staytime);
	}else if( t < 2* staytime + movetime){
		_5srvo(b + 675, a, 645, 570);
	}else if( t < 2* staytime + movetime + pul1time-9000){
		_5srvo(b + 675, a, 645, 570);
		_5othr( 1, 0 );
	}else if( t < 3* staytime + movetime + pul1time-9000){
		_5srvo(675, a, 645, 570);
		_5othr( 0, 0 );
	}else if( t < 3* staytime + movetime + pul1time){
		_5srvo(675, a, 645, 570);
		_5othr( 1, 0 );
	}else if( t < 4* staytime + movetime + pul1time){
		_5srvo(675, a, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 4* staytime + movetime + 2* pul1time){
		_5srvo(675, a, 500, 570);
		_5othr( 2, 1 );
	}else if( t < 5* staytime + movetime + 2* pul1time){
		_5srvo(675, a, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 2* movetime + 2* pul1time){
		_5move(675, 280, 500, 570, 460, 280, 500, 570, t-(5* staytime + movetime + 2* pul1time));
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 3* movetime + 2* pul1time){
		_5move(460, 280, 500, 570, 460, 555, 500, 570, t-(5* staytime + 2* movetime + 2* pul1time));
		_5othr( 0, 1 );
	}else if( t < 6* staytime + 3* movetime + 2* pul1time){
		_5srvo(460, 555, 500, 570);
		_5othr( 0, 1 );
		if(get_stat == 1){
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
			result = 0 ;
		}else{
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
			result = -20 ;
		}
	}else if( t < 7* staytime + 3* movetime + 2* pul1time){
		_5srvo(460, b + 555, 500, 570);
		_5othr( 1, 1 );
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
	}else if( t < 7* staytime + 3* movetime + 2* pul1time + puttime - 600 ){
		_5srvo(460, b + 555, 500, 570);
		_5othr( 1, 1 );
	}else if( t < 8* staytime + 3* movetime + 2* pul1time + puttime - 600 ){
		_5srvo(460, 555, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 4* movetime + 2* pul1time + puttime - 600 ){
		_5move(460, 555, 500, 570, 460, 555, 645, 570, t-(8* staytime + 2* movetime + 2* pul1time + puttime - 600));
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 4* movetime + 2* pul1time + puttime ){
		_5srvo(460, 555, 645, 570);
		_5othr( 1, 1 );
	}else if( t < 9* staytime + 4* movetime + 2* pul1time + puttime ){
		_5srvo(460, 555, 645, 570);
		_5othr( 0, 0 );
	}else if( t < 9* staytime + 4* movetime + 2* pul1time + 2* puttime){
		_5srvo(460, 555, 645, 570);
		_5othr( 2, 0 );
	}else if( t < 10* staytime + 4* movetime + 2* pul1time + 2* puttime){
		_5srvo(460, 555, 645, 570);
		_5othr( 0, 0 );
	}else{
		Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
		printf("don");
	}
}

void _5get__1(void){
	uint32_t t;
	uint32_t pul1time = (get_stat == 1 )?6950:3000, puttime = 800, movetime = 500, staytime = 1000;
	// 13000
	// 预计算相对时间，避免重复计算
	if (Time.ms < Time.tasks[5].tim){
		return ;
	}
	t = Time.ms - Time.tasks[5].tim;

	//83
	int16_t a = 280 ,b = 0;
	b = ((t/100)%2)? -15 : 15 ;

    // 使用更高效的条件判断和预计算
	if( t < staytime ){
		_5srvo(490, 545, 645, 570);
	}else if( t < staytime + movetime){
		_5move(490, 545, 645, 570, 675, 280, 645, 570, t-staytime);
	}else if( t < 2* staytime + movetime){
		_5srvo(b + 675, a, 645, 570);
	}else if( t < 2* staytime + movetime + pul1time-4500){
		_5srvo(b + 675, a, 645, 570);
		_5othr( 1, 0 );
	}else if( t < 3* staytime + movetime + pul1time-4500){
		_5srvo(675, a, 645, 570);
		_5othr( 0, 0 );
	}else if( t < 3* staytime + movetime + pul1time){
		_5srvo(675, a, 645, 570);
		_5othr( 1, 0 );
	}else if( t < 4* staytime + movetime + pul1time){
		_5srvo(675, a, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 4* staytime + movetime + 3* pul1time){
		_5srvo(675, a, 645, 570);
		_5othr( 2, 1 );
	}else if( t < 5* staytime + movetime + 3* pul1time){
		_5srvo(675, a, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 2* movetime + 3* pul1time){
		_5move(675, 280, 500, 570, 460, 555, 500, 570, t-(5* staytime + movetime + 3* pul1time));
		_5othr( 0, 1 );
	}else if( t < 6* staytime + 2* movetime + 3* pul1time){
		_5srvo(460, 555, 500, 570);
		_5othr( 0, 1 );
		if(get_stat == 1){
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
//			result = 20 ;
		}else{
			Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
			result = -20 ;
		}
	}else if( t < 7* staytime + 2* movetime + 3* pul1time){
		_5srvo(460, b + 555, 500, 570);
		_5othr( 1, 1 );
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		result = 0 ;
	}else if( t < 7* staytime + 2* movetime + 3* pul1time + puttime - 300 ){
		_5srvo(460, b + 555, 500, 570);
		_5othr( 1, 1 );
	}else if( t < 8* staytime + 2* movetime + 3* pul1time + puttime - 300 ){
		_5srvo(460, 555, 500, 570);
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 3* movetime + 3* pul1time + puttime - 300 ){
		_5move(460, 555, 500, 570, 460, 555, 645, 570, t-(8* staytime + 2* movetime + 3* pul1time + puttime - 300));
		_5othr( 0, 1 );
	}else if( t < 8* staytime + 3* movetime + 3* pul1time + puttime ){
		_5srvo(460, 555, 645, 570);
		_5othr( 1, 1 );
	}else if( t < 9* staytime + 3* movetime + 3* pul1time + puttime ){
		_5srvo(460, 555, 645, 570);
		_5othr( 0, 0 );
	}else if( t < 9* staytime + 3* movetime + 3* pul1time + 3* puttime){
		_5srvo(460, 555, 645, 570);
		_5othr( 2, 0 );
	}else if( t < 10* staytime + 3* movetime + 3* pul1time + 3* puttime){
		_5srvo(460, 555, 645, 570);
		_5othr( 0, 0 );
	}else{
		Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
		printf("don");
	}
}

void _5put1(void){		//500
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;

	if(t > 0 && t < t_stay){
		_5srvo(490,545,1570,570);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(490,545,1570,570,  490,545,1570,780, t-t_stay);
	}else if ( t < t_stay + 2 * t_move){
		_5move(490,545,1570,780,  490,545,1470,780, t-t_stay-t_move);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty1, "stay2", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}
void _5put2(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;
	if(t > 0 && t < t_stay){
		_5srvo(490,545,1470,780);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(490,545,1400,780,  490,545,960,780, t-t_stay);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty2, "stay2", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}
void _5put3(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;
	if(t > 0 && t < t_stay){
		_5srvo(490,545,940,780);
	}else if ( t < t_stay + t_move){
		_5move(490,545,940,780,  490,545,490,780, t-t_stay-t_move);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			Time.tasks[5] = (struct Task){_5sty3, "stay3", 1, Time.ms};
			Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
		}
	}
}

void _5rtn(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 535);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 645);		//500
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);		//800
}

void _5rtn2(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1570);		//500
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);		//800
}

void _5sty1(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1570);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 780);
}

void _5sty2(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 940);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 780);
}

void _5sty3(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 780);
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
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		case 1:
			if((((Time.ms-Time.tasks[5].tim)/500)%2)== 1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);			//放线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			}else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);			//放线
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			}
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);		//收线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			break;
	}
	switch(state2){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			break;
	}
}

// 6 号位： 常闭

void _6shoot(void){

	if (Time.ms < Time.tasks[6].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[6].tim ; 
	if (t > 10000){
		Time.tasks[6] = (struct Task){NULL, "none", 0, Time.ms};
		return ;
	}
//	goal = 3830 ;
	goal = 2830 ;
	// 1) 射击电机输出窗口（低内存/低计算）
	if (t < 6000) {
		uint8_t ok = (check_motor(4) ? 1 : 0) | (check_motor(5) ? 2 : 0);
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
	uint8_t in_a = (t > 1000) && (t < 4100);
	uint8_t in_b = (!in_a) && (t >= 4200) && (t < 7200);
	if (in_a) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
	} else if (in_b) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}

	// 3) TIM3 CH1/CH2（一次布尔判断）
	uint8_t in_tim3 = (t > 1000) && (t < 6000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, in_tim3 ? 600u : 500u);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, in_tim3 ? 600u : 500u);
}

void _7step1(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;
	uint32_t t_stay = 500 , t_move_long = 7000 , t_move_short = 1200 ;
	get_part = 1;
	if(t < t_stay){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < t_stay + t_move_long){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, t1 + t_stay};
		result = 770 ;
	}else if(t < 2* t_stay + t_move_long){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 2* t_stay + t_move_short + t_move_long){
		Time.tasks[4] = (struct Task){_4rnd, "round", 1, t1 + 2* t_stay + t_move_long};
		result = -410 ;
		// 413
	}else if(t < 3* t_stay + t_move_short + t_move_long){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 3* t_stay + 2* t_move_long + t_move_short){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, t1 + 3* t_stay + t_move_short + t_move_long };
		result = 3200 ;
	}else if(t < 4* t_stay + 2* t_move_long + t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else{
		get_part = 2 ;
		printf("step1don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
	}
}

void _7step2(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t_stay = 500 , t_move_long = 5000 , t_move_short = 1000 , t_up = 13000;
	if(t < t_stay){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < t_stay + t_move_short){
		Time.tasks[4] = (struct Task){_4rnd, "turn", 1, Time.ms};
		result = -450 ;
	}else if(t < 2* t_stay + t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 2* t_stay + t_move_short + t_move_long){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = 1000 ;
	}else if(t < 3* t_stay + t_move_short + t_move_long){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 3* t_stay + t_move_long + t_move_short + t_up){
		Time.tasks[4] = (struct Task){_4upstr, "upstair", 1, Time.ms};
		result = 0 ;	
	}else if(t < 4* t_stay + t_move_long + t_move_short + t_up){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else{
		printf("step2don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
	}
}

void _7step3(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t_stay = 500 , t_move_long = 5000 , t_move_short = 1000 , t_dwn = 14000;
	if(t < t_stay){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < t_stay + t_move_long){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = -450 ;
	}else if(t < 2* t_stay + t_move_long){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 2* t_stay + t_dwn + t_move_long){
		Time.tasks[4] = (struct Task){_4dwnstr, "downstair", 1, Time.ms};
	}else if(t < 3* t_stay + t_dwn + t_move_long){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 3* t_stay + t_move_long + t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4rnd, "round", 1, Time.ms};
		result = 900 ;
	}else if(t < 4* t_stay + t_move_long + t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 4* t_stay + 2* t_move_long + t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = 2000 ;	
	}else if(t < 5* t_stay + 2* t_move_long + t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 5* t_stay + 2* t_move_long + 2* t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4upstr, "upstair", 1, Time.ms};
	}else if(t < 6* t_stay + 2* t_move_long + 2* t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 6* t_stay + 3* t_move_long + 2* t_dwn + 2* t_move_short){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = 1000 ;
	}else{
		printf("step3don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
	}
}

void _7step4(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t_stay = 500 , t_move_long = 5000 , t_move_short = 1000 , t_dwn = 14000;
	if(t < t_stay){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < t_stay + t_move_long){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = -1000 ;
	}else if(t < 2* t_stay + t_move_short){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 2* t_stay + t_move_long + t_dwn){
		Time.tasks[4] = (struct Task){_4dwnstr, "downstair", 1, Time.ms};
		result = 0 ;	
	}else if(t < 3* t_stay + t_move_long + t_dwn){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else if(t < 4* t_stay + 2* t_move_long + t_dwn){
		Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
		result = -2000 ;
	}else if(t < 5* t_stay + 2* t_move_long + t_dwn){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		clean_pid();
		result = 0 ;
	}else{
		printf("step4don\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};	
	}
}

void _7shoot(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;
	uint32_t t_shoot = 10000 , t_put = 4000 , t_stop = 500;

	// 关键节点时间点
	uint32_t node1 = 0;
	uint32_t node2 = t_stop;
	uint32_t node3 = t_stop + t_put + t_shoot;
	uint32_t node4 = 2 * t_stop + t_put + t_shoot;
	uint32_t node5 = 2 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node6 = 3 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node7 = 3 * t_stop + 3 * t_put + 3 * t_shoot;

	if(t < t_stop){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, t1};
		clean_pid();
		result = 0 ;
		if(t >= node1 && t < node1 + 500){
			Time.tasks[5] = (struct Task){_5rtn2, "return", 1, Time.ms};
		}
	}else if(t < node3){
		if(t >= node2 && t < node2 + 500){
			Time.tasks[5] = (struct Task){_5put1, "put", 1, t1 + node2};
		}
	}else if(t < node4){
		if(t >= node3 && t < node3 + 500){
			Time.tasks[5] = (struct Task){_5sty1, "stay", 1, Time.ms};
		}
	}else if (t < node5){
		if(t >= node4 && t < node4 + 500){
			Time.tasks[5] = (struct Task){_5put2, "put", 1, t1 + node4};
		}
	}else if (t < node6){
		if(t >= node5 && t < node5 + 500){
			Time.tasks[5] = (struct Task){_5sty2, "stay", 1, Time.ms};
		}
	}else if (t < node7){
		if(t >= node6 && t < node6 + 500){
			Time.tasks[5] = (struct Task){_5put3, "put", 1, t1 + node7};
		}
	}else{
		printf("shootdon\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};	
	}
}

void _7shoot__1(void){
	if (Time.ms < Time.tasks[7].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[7].tim ;
	uint32_t t1 = Time.tasks[7].tim ;
	uint32_t t_shoot = 10000 , t_put = 4000 , t_stop = 500;

	// 关键节点时间点
	uint32_t node1 = 0;
	uint32_t node2 = t_stop;
	uint32_t node3 = t_stop + t_put + t_shoot;
	uint32_t node4 = 2 * t_stop + t_put + t_shoot;
	uint32_t node5 = 2 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node6 = 3 * t_stop + 2 * t_put + 2 * t_shoot;
	uint32_t node7 = 3 * t_stop + 3 * t_put + 3 * t_shoot;

	if(t < t_stop){
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, t1};
		clean_pid();
		result = 0 ;
		if(t >= node1 && t < node1 + 500){
			Time.tasks[5] = (struct Task){_5rtn2, "return", 1, Time.ms};
		}
	}else if(t < node3){
		if(t >= node2 && t < node2 + 500){
			Time.tasks[5] = (struct Task){_5put1, "put", 1, t1 + node2};
		}
	}else if(t < node4){
		if(t >= node3 && t < node3 + 500){
			Time.tasks[5] = (struct Task){_5sty1, "stay", 1, Time.ms};
		}
	}else if (t < node5){
		if(t >= node4 && t < node4 + 500){
			Time.tasks[5] = (struct Task){_5put2, "put", 1, t1 + node4};
		}
	}else if (t < node6){
		if(t >= node5 && t < node5 + 500){
			Time.tasks[5] = (struct Task){_5sty2, "stay", 1, Time.ms};
		}
	}else if (t < node7){
		if(t >= node6 && t < node6 + 500){
			Time.tasks[5] = (struct Task){_5put3, "put", 1, t1 + node7};
		}
	}else{
		printf("shootdon\r\n");
		Time.tasks[7] = (struct Task){NULL, "none", 0, Time.ms};
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

// 设置任务到指定时间片
void set_task(uint8_t slot, Taskpt func, const char* name, uint8_t stat) {
	if(slot < MAXTASK) {
		Time.tasks[slot].task_func = func;
		// 复制任务名称
		for(uint8_t i = 0; i < 9 && name[i] != '\0'; i++) {
			Time.tasks[slot].task_name[i] = name[i];
		}
		Time.tasks[slot].task_name[9] = '\0';
		Time.tasks[slot].stat = stat;
		Time.tasks[slot].tim = 0;
	}
}

// 清除指定时间片的任务
void clear_task(uint8_t slot) {
	if(slot < MAXTASK) {
		Time.tasks[slot] = (struct Task){NULL, "none", 0, 0};
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
// 示例：动态任务管理函数
void example_task_management(void) {
	// 示例：动态设置任务
	// set_task(0, _3fwd, "forward", 1);    // 在时间片0设置前进任务
	// set_task(5, _5shoot, "shoot", 1);    // 在时间片5设置射击任务
	// clear_task(2);                       // 清除时间片2的任务
	
	// 这样可以在运行时动态调整任务分配，实现真正的并发效果
}
