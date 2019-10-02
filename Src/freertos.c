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
#include "adc.h"
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
uint32_t RPM=0;
uint32_t RPMAvgSum=0;
int32_t lastPosition=0;
uint32_t RPMAvg_i=0;

uint32_t temp=0;
uint32_t tempAvg=0;
uint32_t tempAvg_i=0;

char RPMString[4];
char VelocityString[3];
char tempString[4];

int EncNumberOfPulses=4;   //liczba pulsow enkodera na obrot kola
float WheelCircumference=1.4356;  //obwod kola w m

uint32_t ADCBUF[3];

int32_t stepsToGo=0;
uint32_t lastStep=1;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LCDMainHandle;
osThreadId StepperHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StepperGoOneStep(uint32_t dir,uint32_t step);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void vLCDMain(void const * argument);
void vStepp(void const * argument);

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
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBUF, 3);
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
  osThreadDef(LCDMain, vLCDMain, osPriorityBelowNormal, 0, 1024);
  LCDMainHandle = osThreadCreate(osThread(LCDMain), NULL);

  /* definition and creation of Stepper */
  osThreadDef(Stepper, vStepp, osPriorityAboveNormal, 0, 512);
  StepperHandle = osThreadCreate(osThread(Stepper), NULL);

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

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  //tft backlight on
		//calculation and drawing of temperature
			uint32_t tempNow=ADCBUF[0];
			tempAvg+=tempNow;
			if(tempAvg_i==9){
				temp=tempAvg/10;
				tempAvg=0;
				tempAvg_i=0;
				int tempCelsius=100000/temp;
				char* pTemp=tempString;
				itoa(tempCelsius,pTemp,10);
				ILI9341_Draw_Rectangle( 30, 10, 80, 60, BLACK);
				ILI9341_Draw_Text(pTemp, 30, 10, WHITE, 4, BLACK);
			}
			tempAvg_i++;


			//RPM value drawing
			if(RPM<15000){
			char* pRPM=RPMString;
			itoa(RPM, pRPM, 10);
			ILI9341_Draw_Rectangle( ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2-40, 250, 120, BLACK);
			ILI9341_Draw_Text(pRPM, ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2-40, BLACK, 5, WHITE);
			}
			//Velocity drawing
			float Velocity;
			char* pVelocity=VelocityString;
			if(Get_VelocityTime_Value()==0){Velocity=0;}else{
			Velocity=((WheelCircumference/EncNumberOfPulses)/(Get_VelocityTime_Value()*0.0001))*3.6;}
			if(Velocity<100){
			itoa(Velocity, pVelocity, 10);
			ILI9341_Draw_Text(pVelocity, ILI9341_SCREEN_WIDTH/2-100, ILI9341_SCREEN_HEIGHT/2+20, WHITE, 8, BLACK);
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  //tft backlight on

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  	  	osDelay(100);




		}
  /* USER CODE END vLCDMain */
}

/* USER CODE BEGIN Header_vStepp */
/**
* @brief Function implementing the Stepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStepp */
void vStepp(void const * argument)
{
  /* USER CODE BEGIN vStepp */
  /* Infinite loop */
  for(;;)
  {

	  uint32_t RPMNow=360000/Get_IC_Value();
	  RPMAvgSum+=RPMNow;

	  if(RPMAvg_i==9){
	  	RPM=RPMAvgSum/10;
	  	RPMAvgSum=0;
	  	RPMAvg_i=0;

	  	}
	  RPMAvg_i++;

	  uint32_t StepperPosition=RPM/10;



	  	if(stepsToGo!=0){
	  		stepsToGo=(StepperPosition-lastPosition)+stepsToGo;
	  	}
	  	else{stepsToGo=(StepperPosition-lastPosition);}

	  	lastPosition=StepperPosition;

	  if(stepsToGo>0){
	  			StepperGoOneStep(1,2);
	  			stepsToGo-=1;
	  	}
	  if(stepsToGo<0){
	  			StepperGoOneStep(0,2);
	  			stepsToGo+=1;
	  	}
	  if(stepsToGo==0){
	  			osDelay(2);
		}
  }
  /* USER CODE END vStepp */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StepperGoOneStep(uint32_t dir, uint32_t speed){
	if(dir!=0){  //przeciwnie do wskazowek zegara

		if(lastStep==2){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //1
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
		lastStep=1;
		osDelay(speed);
		}
		if(lastStep==3){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //2
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=2;
		osDelay(speed);
		}
		if(lastStep==4){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //3
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=3;
		osDelay(speed);
		}
		if(lastStep==5){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //4
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=4;
		osDelay(speed);
		}
		if(lastStep==6){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //5
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=5;
		osDelay(speed);
		}
		if(lastStep==1){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //6
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
		lastStep=6;
		osDelay(speed);
		}
	}
	else{ //zgodnie z ruchem wzkazowek zegara

		if(lastStep==5){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //6
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
		lastStep=6;
		osDelay(speed);
		}

		if(lastStep==4){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //5
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=5;
		osDelay(speed);
		}

		if(lastStep==3){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //4
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=4;
		osDelay(speed);
		}

		if(lastStep==2){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //3
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=3;
		osDelay(speed);
		}

		if(lastStep==1){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //2
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
		lastStep=2;
		osDelay(speed);
		}

		if(lastStep==6){
		HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //1
		HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
		lastStep=1;
		osDelay(speed);
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
