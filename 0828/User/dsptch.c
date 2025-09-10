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
			curtask->stat = (curtask->stat == 0) ? 0 : curtask->stat + 1 ;
			if(curtask->stat >= 1001){ 
				curtask->stat = 0;
			}
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
	Time.tasks[4] = (struct Task){_4stp, "stop", 1, 0};    // 时间片4: 底盘控制
	Time.tasks[5] = (struct Task){_5rtn, "return", 1, 0};   // 时间片5: 舵机控制
	Time.tasks[6] = (struct Task){NULL, "none", 0, 0};      // 时间片6: 发射控制
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
//	if((Time.ms % 200) < 3){
//		printf("??%d\t%d  %d  %d  %d\t %d %d %d %d\r\n",
//				Time.ms,pos[0],pos[1],pos[2],pos[3],
//				pid_velocity[0].target_val,pid_velocity[1].target_val,
//				pid_velocity[2].target_val,pid_velocity[3].target_val);
//	}
}

// 3 号位： 常开

void _3pid(void){
//	get_angle(0);
//	get_angle(1);
//	get_angle(2);
//	get_angle(3);

	pid_angle();
}

// 4 号位： 常开

void _4fwd(void){			//forward
	pid_position[0].target_val = -10 * result ;
	pid_position[1].target_val = 10 * result ;
	pid_position[2].target_val = 10 * result ;
	pid_position[3].target_val = -10 * result ;
	_4();
}

void _4rgt(void){			//right
	pid_position[0].target_val = -10 * result ;
	pid_position[1].target_val = -10 * result ;
	pid_position[2].target_val = 10 * result ;
	pid_position[3].target_val = 10 * result ;
	_4();
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
		result = 0 ;
		return ;
	}
	if(Time.ms - Time.tasks[4].tim > 100 * result){
		clean_pid();
		Time.tasks[4] = (struct Task){_4stp, "stop", 1, Time.ms};
		printf("err");
		result = 0 ;
		return ;
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

// 5 号位： 常开

void _5get(void){
	uint32_t t1, t;
	uint32_t ch1_val, ch2_val, ch3_val;
	
	t = Time.ms;
	t1 = Time.tasks[4].tim;
	
	// 预计算相对时间，避免重复计算
	uint32_t rel_time = (t > t1) ? (t - t1) : 0;

    // 使用更高效的条件判断和预计算
    if(rel_time < 500) {
    	// 线性插值计算
    	ch1_val = (675 * rel_time + 420 * (500 - rel_time)) / 500;
    	ch2_val = (83 * rel_time + 620 * (500 - rel_time)) / 500;
    	ch3_val = 650;
    } else if(rel_time < 23200) {
    	ch1_val = 675;
    	ch2_val = 83;
    	ch3_val = 650;
    } else if(rel_time < 23450) {
    	uint32_t phase_time = rel_time - 23200;
    	ch1_val = 675;
    	ch2_val = (593 * phase_time + 83 * (250 - phase_time)) / 250;
    	ch3_val = (450 * phase_time + 650 * (500 - phase_time)) / 500;
    } else if(rel_time < 23700) {
    	uint32_t phase_time = rel_time - 23450;
    	ch1_val = (295 * phase_time + 675 * (250 - phase_time)) / 250;
    	ch2_val = 593;
    	ch3_val = (450 * (rel_time - 23200) + 650 * (500 - (rel_time - 23200))) / 500;
    } else if(rel_time < 33100) {
    	ch1_val = 295;
    	ch2_val = 593;
    	ch3_val = 450;
    } else {
    	ch1_val = 420;
    	ch2_val = 620;
    	ch3_val = 650;
    }
    // 批量设置PWM值，减少函数调用次数
    _5srvo(ch1_val,ch2_val,ch3_val,500);

    //										电磁铁/收放线
    if(t > t1+5100 && t < t1+10900){		//放5800
		_5othr(1, 0);
    }else if(t > t1+10900 && t < t1+11900){
		_5othr(0, 0);
    }else if(t > t1+11900 && t < t1+23100){	//收11000
		_5othr(2, 1);
    }else if(t > t1+23100 && t < t1+27000){
		_5othr(0, 1);
    }else if(t > t1+27000 && t < t1+32000){	//放5000
		_5othr(1, 1);
    }else if(t > t1+32000 && t < t1+33000){
		_5othr(0, 0);
    }
}

void _5put(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 645);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

}

void _5rtn(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 490);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 545);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 645);		//500
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);		//800
}

void _5srvo(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angle2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, angle4);
}

void _5othr(uint8_t state1, uint8_t state2){
	switch(state1){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);		//放线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);			//收线
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
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
	;
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

// 示例：动态任务管理函数
void example_task_management(void) {
	// 示例：动态设置任务
	// set_task(0, _3fwd, "forward", 1);    // 在时间片0设置前进任务
	// set_task(5, _5shoot, "shoot", 1);    // 在时间片5设置射击任务
	// clear_task(2);                       // 清除时间片2的任务
	
	// 这样可以在运行时动态调整任务分配，实现真正的并发效果
}
