#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "ustreceive.h"


extern const uint8_t MAXTASK ;

// 定义任务函数指针类型
typedef void (*Taskpt)(void);

// 任务结构体
typedef struct Task {
    Taskpt task_func ;          // 任务函数指针
    uint8_t task_name[10] ;     // 任务名称
    uint8_t stat ;              // 任务状态
    uint32_t tim ;              // 任务时间
} Task;

// 任务调度器结构体
typedef struct TaskSchdle {
    Task tasks[8];       // 任务数组，与MAXTASK保持一致
    uint64_t timer;
    uint32_t ms;
    uint8_t t;
    uint8_t cur;
    uint32_t task_start_time;  // 任务开始时间
    uint32_t max_task_time;    // 最大任务执行时间(ms)
    uint32_t last_dispatch_time; // 上次dispatch调用时间
    uint32_t watchdog_timeout;   // 看门狗超时时间(ms)
} TaskSchdle;

// 声明全局变量
extern TaskSchdle Time;
extern int8_t get_stat;
extern int8_t dart_num;

// 可以添加相关的函数声明
uint8_t check(void);
void dispatch(void);
void Init_schdl(void);
void watchdog_check(void);  // 看门狗检查函数
void set_task(uint8_t slot, Taskpt func, const char* name, uint8_t stat); // 设置任务
void clear_task(uint8_t slot); // 清除任务
void _0STOP(void);
void _0Stop(uint8_t i);	//清除指定时间片的任务
void _1mssg(void);
void _3pid(void);
void _4fwd(void);
void _4rgt(void);
void _4rnd(void);
void _4stp(void);
void _4(void);
void _4upstr(void);
void _4dwnstr(void);
void _4for(int16_t distance);
void _5get(void);
void _5put1(void);
void _5put2(void);
void _5put3(void);
void _5rtn(void);
void _5sty1(void);
void _5sty2(void);
void _5sty3(void);
void _5srvo(uint16_t angle1, uint16_t angle2, uint16_t angle3, uint16_t angle4);
void _5move(uint16_t angle11, uint16_t angle21, uint16_t angle31, uint16_t angle41, uint16_t angle12, uint16_t angle22, uint16_t angle32, uint16_t angle42, uint16_t t);
void _5othr(uint8_t state1, uint8_t state2);
void _6shoot(void);
void _7step1(void);
void _7step2(void);
void _7step3(void);
void _7step4(void);
void _7shoot(void);
uint8_t check_motor(uint8_t sign);
void example_task_management(void);

#endif // TASK_SCHEDULER_H
