/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"
#include "stm32f1xx_it.h"
#include "itoa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char RPMString[4];
char VelocityString[3];
int EncNumberOfPulses=4;   //liczba pulsow enkodera na obrot kola
float WheelCircumference=1.4356;  //obwod kola w m

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LCDMainHandle;
osThreadId BlinkHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void vLCDMain(void const * argument);
void vBlink(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LCDMain */
  osThreadDef(LCDMain, vLCDMain, osPriorityAboveNormal, 0, 1024);
  LCDMainHandle = osThreadCreate(osThread(LCDMain), NULL);

  /* definition and creation of Blink */
  osThreadDef(Blink, vBlink, osPriorityBelowNormal, 0, 512);
  BlinkHandle = osThreadCreate(osThread(Blink), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_vLCDMain */
/**
* @brief Function implementing the LCDMain thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vLCDMain */
void vLCDMain(void const * argument)
{
  /* USER CODE BEGIN vLCDMain */
  /* Infinite loop */
	while(1){

			char* pRPM=RPMString;
			int RPM=18000/Get_IC_Value();
			itoa(RPM, pRPM, 10);
			ILI9341_Draw_Rectangle( ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2-40, 250, 120, BLACK);
			ILI9341_Draw_Text(pRPM, ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2-40, WHITE, 8, BLACK);
			float Velocity;
			char* pVelocity=VelocityString;
			if(Get_VelocityTime_Value()==0){Velocity=0;}else{
			Velocity=((WheelCircumference/EncNumberOfPulses)/(Get_VelocityTime_Value()*0.0001))*3.6;}
			itoa(Velocity, pVelocity, 10);
			ILI9341_Draw_Text(pVelocity, ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2+20, WHITE, 8, BLACK);
	  	  	osDelay(200);




		}
  /* USER CODE END vLCDMain */
}

/* USER CODE BEGIN Header_vBlink */
/**
* @brief Function implementing the Blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vBlink */
void vBlink(void const * argument)
{
  /* USER CODE BEGIN vBlink */
	while(1){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			osDelay(200);
		}
  /* USER CODE END vBlink */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
