#include "dsptch.h"

// 定义全局变量
TaskSchdle Time;

extern uint32_t result;

const uint8_t MAXTASK = 8;

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
			// curtask->stat = (curtask->stat == 0) ? 0 : curtask->stat + 1 ;
			// if(curtask->stat >= 200){ 
			// 	curtask->stat = 0;
			// }
			curtask->stat = 0 ;
		} else {
			// 记录任务开始时间
			Time.task_start_time = HAL_GetTick();
			
			// 执行任务 - 每个任务在1/8ms内完成
			curtask->task_func();
			
			// 检查任务执行时间
			uint32_t exec_time = HAL_GetTick() - Time.task_start_time;
			if(exec_time > Time.max_task_time) {
				// 任务执行时间过长，记录错误状态
				curtask->stat = 2; // 使用状态2表示任务超时
				// 可以通过LED闪烁或其他方式指示任务超时
			} else {
				curtask->stat = 1; // 正常完成
			}
		}
	} else {
		// 如果cur超出范围，重置为安全值
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
	Time.tasks[4] = (struct Task){_4rgt, "stop", 1, 0};    	// 时间片4: 底盘控制
	Time.tasks[5] = (struct Task){_5rtn, "return", 1, 0};   	// 时间片5: 舵机控制
	Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, 0};      // 时间片6: 发射控制
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
		printf("??%d\t%d  %d  %d  %d\t %d %d %d %d\r\n",
				Time.ms,pos[0],pos[1],pos[2],pos[3],
				pid_velocity[0].target_val,pid_velocity[1].target_val,
				pid_velocity[2].target_val,pid_velocity[3].target_val);
	}
}

// 3 号位： 常开

void _3pid(void){
	pid_angle();
}

// 4 号位： 常开

void _4fwd(void){			//forward
	pid_target = result ;
	tar[0] = tar[3] = -1 ;
	tar[2] = tar[1] = 1 ;
	flag = 0 ;
	pid_position[0].target_val = -10 * result ;
	pid_position[1].target_val = 10 * result ;
	pid_position[2].target_val = 10 * result ;
	pid_position[3].target_val = -10 * result ;
	_4();
}

void _4rgt(void){			//right
	pid_target = result ;
	tar[0] = tar[1] = -1 ;
	tar[2] = tar[3] = 1 ;
	flag = 1 ;
//	pid_position[0].target_val = -10 * result ;
//	pid_position[1].target_val = -10 * result ;
//	pid_position[2].target_val = 10 * result ;
//	pid_position[3].target_val = 10 * result ;
	if(Time.ms - Time.tasks[4].tim > 10){
		_4();
	}
}

void _4rnd(void){			//round,clock side
	pid_position[0].target_val = 10 * result ;
	pid_position[1].target_val = 10 * result ;
	pid_position[2].target_val = 10 * result ;
	pid_position[3].target_val = 10 * result ;
	_4();
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
	pid_position[0].target_val = 0 ;
	pid_position[1].target_val = 0 ;
	pid_position[2].target_val = 0 ;
	pid_position[3].target_val = 0 ;
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
	pid_position[0].target_val = -10 * distance ;
	pid_position[1].target_val = 10 * distance ;
	pid_position[2].target_val = 10 * distance ;
	pid_position[3].target_val = -10 * distance ;
	if(check_pid()){
		clean_pid();
	}
	CAN_cmd_chassis(crt[0], crt[1], crt[2], crt[3]);
}

// 5 号位： 常开

void _5get(void){
	uint32_t t;
	uint32_t pultime = 10000, puttime = 5000, movetime = 500, staytime = 1000;
	// 预计算相对时间，避免重复计算
	if (Time.ms < Time.tasks[4].tim){
		return ;
	}
	t = Time.ms - Time.tasks[4].tim;

    // 使用更高效的条件判断和预计算
	if( t < staytime ){
		_5srvo(490, 545, 645, 570);
	}else if( t < staytime + movetime){
		_5move(490, 545, 645, 570, 675, 83, 645, 570, t-staytime);
	}else if( t < 2* staytime + movetime){
		_5srvo(675, 83, 645, 570);
	}else if( t < 2* staytime + movetime + pultime){
		_5srvo(675, 83, 645, 570);
		_5othr( 1, 0 );
	}else if( t < 3* staytime + movetime + pultime){
		_5srvo(675, 83, 645, 570);
		_5othr( 0, 1 );
	}else if( t < 3* staytime + movetime + 2* pultime){
		_5srvo(675, 83, 645, 570);
		_5othr( 2, 1 );
	}else if( t < 4* staytime + movetime + 2* pultime){
		_5srvo(675, 83, 645, 570);
		_5othr( 0, 1 );
	}else if( t < 4* staytime + 2* movetime + 2* pultime){
		_5move(675, 83, 645, 570, 490, 545, 645, 570, t-(4* staytime + movetime + 2* pultime));
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 2* movetime + 2* pultime){
		_5srvo(490, 545, 645, 570);
		_5othr( 0, 1 );
	}else if( t < 5* staytime + 2* movetime + 2* pultime + puttime){
		_5srvo(490, 545, 645, 570);
		_5othr( 1, 1 );
	}else if( t < 6* staytime + 2* movetime + 2* pultime + puttime){
		_5srvo(490, 545, 645, 570);
		_5othr( 0, 0 );
	}else if( t < 6* staytime + 2* movetime + 2* pultime + 2* puttime){
		_5srvo(490, 545, 645, 570);
		_5othr( 2, 0 );
	}else if( t < 7* staytime + 2* movetime + 2* pultime + 2* puttime){
		_5srvo(490, 545, 645, 570);
		_5othr( 0, 0 );
	}else{
		Time.tasks[5] = (struct Task){_5rtn, "return", 1, Time.ms};
	}
}

void _5put(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;

	if(t > 0 && t < t_stay){
		_5srvo(490,545,990,570);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(490,545,990,570,  490,545,500,570, t-t_stay);
	}else if ( t < t_stay + 2 * t_move){
		_5move(490,545,500,570,  490,545,500,780, t-t_stay-t_move);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			// Time.tasks[5] = (struct Task){_5sty, "stay", 1, Time.ms};
			// Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
			Time.tasks[5] = (struct Task){_5nex1, "next 2", 1, Time.ms};
		}
	}
}
void _5nex1(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;
	if(t > 0 && t < t_stay){
		_5srvo(490,545,500,780);
		_5othr(0,0);
	}else if ( t < t_stay + t_move){
		_5move(490,545,500,780,  490,545,980,570, t-t_stay);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			// Time.tasks[5] = (struct Task){_5sty, "stay", 1, Time.ms};
			// Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
			Time.tasks[5] = (struct Task){_5nex2, "next 2", 1, Time.ms};
		}
	}
}
void _5nex2(void){
	uint32_t t = Time.ms - Time.tasks[5].tim ;
	uint32_t t_stay = 2000 , t_move = 500;
	if(t > 0 && t < t_stay){
		_5srvo(490,545,980,780);
	}else if ( t < t_stay + t_move){
		_5move(490,545,980,780,  490,545,1460,570, t-t_stay-t_move);
	}else{
		if((Time.tasks[4].task_name[1] == 't') ){
			// Time.tasks[5] = (struct Task){_5sty, "stay", 1, Time.ms};
			// Time.tasks[6] = (struct Task){_6shoot, "shoot", 1, Time.ms};
			Time.tasks[5] = (struct Task){_5sty, "next 2", 1, Time.ms};
		}
	}
}

void _5rtn(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 645);		//500
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);		//800
}

void _5sty(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);		
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
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);		//放线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);			//收线
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
// 5 号位： 常闭

void _6shoot(void){
	// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
	// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
	// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
	if (Time.ms < Time.tasks[6].tim){
		return ;
	}
	uint32_t t = Time.ms - Time.tasks[6].tim ; 
	if (t > 10000){
		Time.tasks[6] = (struct Task){NULL, "none", 0, Time.ms};
		return ;
	}
	goal = 3830 ;
	// 1) 射击电机输出窗口（低内存/低计算）
	if (t < 5000) {
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
	uint8_t in_a = (t > 1000) && (t < 2850);
	uint8_t in_b = (!in_a) && (t >= 2850) && (t < 4700);
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
	}

	// 3) TIM3 CH1/CH2（一次布尔判断）
	uint8_t in_tim3 = (t > 1000) && (t < 5000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, in_tim3 ? 600u : 500u);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, in_tim3 ? 600u : 500u);
}


// 看门狗检查函数
void watchdog_check(void){
	uint32_t current_time = HAL_GetTick();
	
	// 检查是否超过看门狗超时时间
	if((current_time - Time.last_dispatch_time) > Time.watchdog_timeout) {
		// 调度器可能卡死，执行恢复操作
		Time.cur = 0;  // 重置当前任务
		Time.last_dispatch_time = current_time;  // 更新看门狗时间
		
		// 可以通过LED闪烁或其他方式指示看门狗触发
		// 例如：HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
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
