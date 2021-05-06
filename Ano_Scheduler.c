/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "include.h"
#include "Ano_RC.h"
#include "Ano_Parameter.h"
#include "Drv_time.h"
#include "Drv_led.h"
#include "Drv_pwm_in.h"
#include "Drv_icm20602.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlightDataCal.h"
#include "Ano_AttCtrl.h"
#include "Ano_Imu.h"
#include "Drv_laser.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_MagProcess.h"
#include "Ano_Power.h"
#include "Ano_OF.h"
#include "Drv_heating.h"
#include "Ano_FlyCtrl.h"
#include "Ano_UWB.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_OpenMV.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"
#include "Ano_OPMV_LineTracking_Ctrl.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_OF_DecoFusion.h"


u32 test_dT_1000hz[3],test_rT[6];			//全局数组

// 获取传感器数据和飞行状态
static void Loop_1000Hz(void)	//1ms执行一次
{
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (u32)(test_dT_1000hz[1] - test_dT_1000hz[0]) ;
	
	/*传感器数据读取*/
	Fc_Sensor_Get();
	
	/*惯性传感器数据准备*/
	Sensor_Data_Prepare(1);
	
	/*姿态解算更新*/
	IMU_Update_Task(1);
	
	/*获取WC_Z加速度*/
	WCZ_Acc_Get_Task();
	/*获取WC_XY的加速度*/
	WCXY_Acc_Get_Task();
	
	/*飞行状态任务*/
	Flight_State_Task(1,CH_N);
	
	/*开关状态任务*/
	Swtich_State_Task(1);
	
	/*光流融合数据准备任务*/
	ANO_OF_Data_Prepare_Task(0.01f);


	/*数传数据交换*/
	ANO_DT_Data_Exchange();
	
			test_rT[4]= GetSysTime_us ();
			test_rT[5] = (u32)(test_rT[4] - test_rT[3]) ;	
}

// PID控制
static void Loop_500Hz(void)	//2ms执行一次
{	
	/*姿态角速度环控制*/
	Att_1level_Ctrl(2*1e-3f);
	
	/*电机输出控制*/
	Motor_Ctrl_Task(2);	
	
	/*UWB数据获取*/
	Ano_UWB_Get_Data_Task(2);	
}

static void Loop_200Hz(void)	//5ms执行一次
{
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(5e-3f,CH_N);
	
}

// 气压计高度控制
static void Loop_100Hz(void)	//10ms执行一次
{
			test_rT[0]= GetSysTime_us ();
			
	/*遥控器数据处理*/
	RC_duty_task(10);
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(10);
	
	/*高度数据融合任务*/
	WCZ_Fus_Task(10);
	
	/*GPS 数据处理任务*/
	GPS_Data_Processing_Task(10);
	
	/*高度速度环控制*/
	Alt_1level_Ctrl(10e-3f);
	
	/*高度环控制*/
	Alt_2level_Ctrl(10e-3f);
	
	/*--*/	
	AnoOF_DataAnl_Task(10);

	/*灯光控制*/	
	LED_Task2(10);
	
			test_rT[1]= GetSysTime_us ();
			test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	
				
}


static void Loop_50Hz(void)	//20ms执行一次
{	
	/*罗盘数据处理任务*/
	Mag_Update_Task(20);
	
	/*程序指令控制*/
	FlyCtrl_Task(20);
	
	ANO_OFDF_Task(20);
//	/*UWB数据计算*/
//	Ano_UWB_Data_Calcu_Task(20);
	/*位置速度环控制*/
	Loc_1level_Ctrl(20,CH_N);
	/*OPMV检测是否掉线*/
	OpenMV_Offline_Check(20);
	/*OPMV色块追踪数据处理任务*/
	ANO_CBTracking_Task(20);
	/*OPMV寻线数据处理任务*/
	ANO_LTracking_Task(20);
	/*OPMV控制任务*/
	ANO_OPMV_Ctrl_Task(20);
}

static void Loop_20Hz(void)	//50ms执行一次
{	
	/*电压相关任务*/
	Power_UpdateTask(50);
	//恒温控制
	Thermostatic_Ctrl_Task(50);
}

static void Loop_2Hz(void)	//500ms执行一次
{
	/*延时存储任务*/
	Ano_Parame_Write_task(500);

}

/*
	：待扩展功能模块
static void task_1(void *p_arg)
{
		// 实现避障功能
		// 超声波模块
		
}

static void task_2(void *p_arg)
{
		// 实现寻迹功能
		// OpenMV摄像头模组模块
}

*/



/*
	自写了一个任务调度器算法


//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 		// 任务数组，结构体数组，7个任务
{
	{Loop_1000Hz,1000,  0, 0},						// 1000hz 	1ms
	{Loop_500Hz , 500,  0, 0},						// 500hz  	2ms
	{Loop_200Hz , 200,  0, 0},						// 200hz   	5ms
	{Loop_100Hz , 100,  0, 0},						// 100hz		10ms
	{Loop_50Hz  ,  50,  0, 0},						// 50hz			20ms
	{Loop_20Hz  ,  20,  0, 0},						// 20hz			50ms
	{Loop_2Hz   ,   2,  0, 0},						// 2hz			500ms
	//{Loop_1Hz , 1, 0, 0}
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))	// 获取数组大小算法：N = sizeof(array) / sizeof(array[0]) 

// 调度器初始化
void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for(index=0;index < TASK_NUM;index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz;		//  T = 1000 / rate_hz
		
		//最短周期为1，也就是1ms
		if(sched_tasks[index].interval_ticks < 1)			// 1ms ~ 500ms
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}

// 任务调度调度函数
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	
	//循环判断所有线程，是否应该执行
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = SysTick_GetTick();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		// t2 - t1 = delta t >= T(thread T)
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)	// 最近没有执行过该线程
			{
			
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();

		}	 
	}
}



/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
*/	

