/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：主循环
**********************************************************************************/
#include "include.h"
#include "Ano_FcData.h"
#include "FreeRTOS.h"
#include "task.h"


#define START_TASK_PRIO		1
#define START_STK_SIZE 		128  
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);


#define TASK1_TASK_PRIO		2
#define TASK1_STK_SIZE 		128  
TaskHandle_t Task1Task_Handler;
void task1_task(void *pvParameters);


#define TASK2_TASK_PRIO		3
#define TASK2_STK_SIZE 		128  
TaskHandle_t Task2Task_Handler;
void task2_task(void *pvParameters);


#define TASK3_TASK_PRIO		4	
#define TASK3_STK_SIZE 		128  
TaskHandle_t Task3Task_Handler;
void task3_task(void *pvParameters);

#define TASK4_TASK_PRIO		5	
#define TASK4_STK_SIZE 		128  
TaskHandle_t Task4Task_Handler;
void task4_task(void *pvParameters);

#define TASK5_TASK_PRIO		6	
#define TASK5_STK_SIZE 		128  
TaskHandle_t Task5Task_Handler;
void task5_task(void *pvParameters);


#define TASK6_TASK_PRIO		7	
#define TASK6_STK_SIZE 		128  
TaskHandle_t Task6Task_Handler;
void task6_task(void *pvParameters);

#define TASK7_TASK_PRIO		8	
#define TASK7_STK_SIZE 		128  
TaskHandle_t Task7Task_Handler;
void task7_task(void *pvParameters);



#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
		//当系统出错后，会进入这个死循环
  }
}
#endif
//=======================================================================================
//=======================================================================================
int main(void)
{
	flag.start_ok = All_Init();		//进行所有设备的初始化，并将初始化结果保存
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}



//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建TASK1任务
    xTaskCreate((TaskFunction_t )task1_task,             
                (const char*    )"task1_task",           
                (uint16_t       )TASK1_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )TASK1_TASK_PRIO,        
                (TaskHandle_t*  )&Task1Task_Handler);   
    //创建TASK2任务
    xTaskCreate((TaskFunction_t )task2_task,     
                (const char*    )"task2_task",   
                (uint16_t       )TASK2_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK2_TASK_PRIO,
                (TaskHandle_t*  )&Task2Task_Handler); 
				
	 //创建TASK3任务
    xTaskCreate((TaskFunction_t )task3_task,     
                (const char*    )"task3_task",   
                (uint16_t       )TASK3_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK3_TASK_PRIO,
                (TaskHandle_t*  )&Task3Task_Handler); 		
		
	 //创建TASK4任务
    xTaskCreate((TaskFunction_t )task4_task,     
                (const char*    )"task4_task",   
                (uint16_t       )TASK4_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK4_TASK_PRIO,
                (TaskHandle_t*  )&Task4Task_Handler); 

	 //创建TASK5任务
    xTaskCreate((TaskFunction_t )task5_task,     
                (const char*    )"task5_task",   
                (uint16_t       )TASK5_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK5_TASK_PRIO,
                (TaskHandle_t*  )&Task5Task_Handler); 
				
	 //创建TASK6任务
    xTaskCreate((TaskFunction_t )task6_task,     
                (const char*    )"task6_task",   
                (uint16_t       )TASK6_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK6_TASK_PRIO,
                (TaskHandle_t*  )&Task6Task_Handler); 
		
	 //创建TASK7任务
    xTaskCreate((TaskFunction_t )task7_task,     
                (const char*    )"task7_task",   
                (uint16_t       )TASK7_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )TASK7_TASK_PRIO,
                (TaskHandle_t*  )&Task7Task_Handler); 

				
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}


u32 test_dT_1000hz[3],test_rT[6];			//全局数组

// 获取传感器数据和飞行状态
static void task1_task(void *pvParameters)	//1ms执行一次
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
static void task2_task(void *pvParameters)	//2ms执行一次
{	
	/*姿态角速度环控制*/
	Att_1level_Ctrl(2*1e-3f);
	
	/*电机输出控制*/
	Motor_Ctrl_Task(2);	
	
	/*UWB数据获取*/
	Ano_UWB_Get_Data_Task(2);	
}

static void task3_task(void *pvParameters)	//5ms执行一次
{
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(5e-3f,CH_N);
	
}

// 气压计高度控制
static void task4_task(void *pvParameters)	//10ms执行一次
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


static void task5_task(void *pvParameters)	//20ms执行一次
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

static void task6_task(void *pvParameters)	//50ms执行一次
{	
	/*电压相关任务*/
	Power_UpdateTask(50);
	//恒温控制
	Thermostatic_Ctrl_Task(50);
}

static void task7_task(void *pvParameters)	//500ms执行一次
{
	/*延时存储任务*/
	Ano_Parame_Write_task(500);

}
