#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"


typedef struct
{
void(*task_func)(void);		// 函数指针，用于执行不同的任务
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif

