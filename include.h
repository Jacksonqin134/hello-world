#ifndef _INCLUDE_H_
#define _INCLUDE_H_

//#include "stm32f4xx.h"

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
#include "Ano_FcData.h"

#include "BSP_Init.h"
#include "Ano_DT.h"
#include "Ano_Parameter.h"
#include "Ano_USB.h"
#include "Drv_time.h"
#include "Drv_pwm_in.h"
#include "Drv_usart.h"
#include "Drv_Gps.h"
#include "Drv_nrf24l01.h"

//================系统===================
#define HW_TYPE	05
#define HW_VER	1
#define SOFT_VER 17
#define BL_VER	0
#define PT_VER	400


#define ANO_DT_USE_USART2 				//开启串口2数传功能
#define ANO_DT_USE_USB_HID				//开启飞控USBHID连接上位机功能
//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择

#define NVIC_PWMIN_P			1			//接收机采集中断配置
#define NVIC_PWMIN_S			0

#define NVIC_TIME_P       2					//定时器中断配置，暂未使用
#define NVIC_TIME_S       1

#define NVIC_UART6_P				4			//串口6中断配置
#define NVIC_UART6_S				1

#define NVIC_UART5_P				4			//串口5中断配置
#define NVIC_UART5_S				1

#define NVIC_UART4_P			2			//串口4中断配置
#define NVIC_UART4_S			1

#define NVIC_UART2_P			2			//串口2中断配置
#define NVIC_UART2_S			0

#define NVIC_UART1_P			3			//串口1中断配置 //gps
#define NVIC_UART1_S			0
/***********************************************/
//================传感器===================
//以下为板载各个传感器的使能引脚配置
// 陀螺仪传感器使能引脚配置
//#define ICM20602_CS_RCC		RCC_AHB1Periph_GPIOD
//#define ICM20602_CS_GPIO	GPIOD
//#define ICM20602_CS_PIN		GPIO_Pin_0

// 电子罗盘/磁力计
//#define AK8975_CS_RCC		RCC_AHB1Periph_GPIOC
//#define AK8975_CS_GPIO		GPIOC
//#define AK8975_CS_PIN		GPIO_Pin_10
// 气压计
//#define SPL06_CS_RCC		RCC_AHB1Periph_GPIOC
//#define SPL06_CS_GPIO		GPIOC
//#define SPL06_CS_PIN		GPIO_Pin_11


//=========================================
#endif

