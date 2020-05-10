/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MOTOHUD_CONFIG.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t input_capture;
uint32_t VelocityTime=0;
uint32_t lastVelocityTime=99999;
uint32_t VelocityAvgI=1;
uint32_t VelocityTimeSum=0;
uint32_t VelocityTimeAvg=1; //non-zero value because 0 means timer disabled, vehicle is stopped
uint8_t MotorcycleStopped=1;
uint16_t WheelSpinCounter=0;

int32_t lastPosition=0;
int32_t stepsToGo=0;
int32_t lastStepsToGo=0;
uint32_t lastStep=1;

uint16_t stepper_delay=70;
uint16_t lastStepperDelay=70;
int16_t stepperAccelDelay=0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//uint32_t Get_IC_Value();
//uint32_t Get_VelocityTime_Value();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */
extern double mileage;
extern double trip;


extern uint8_t EncNumberOfPulses;
extern float WheelCircumference;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */
	uint32_t pending = EXTI->PR;
	if(pending & (1 << 9)) {
		EXTI->PR = 1 << 9;

		/* USER CODE END EXTI9_5_IRQn 0 */
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
		/* USER CODE BEGIN EXTI9_5_IRQn 1 */

		//need to wait few uS between interrupt request and GPIO logic level read - signal needs to be stable LOW
		for(uint32_t i=0;i<500;i++){double l=log(sin(0.03f));}//delay in interrupt - bad idea but it works

		///////////////////////////////////VELOCITY TIME MEASUREMENT//////////////////////////

		if(__HAL_TIM_GET_COUNTER(&htim2)==0){//if timer was disabled enable it
			__HAL_TIM_ENABLE(&htim2);
			MotorcycleStopped=0;
		}
		else{
			if(__HAL_TIM_GET_COUNTER(&htim2)>100 && !HAL_GPIO_ReadPin(VPIN_GPIO_Port, VPIN_Pin)){       //noise filter-100=10ms for 1/4 wheel rev. Max speed possible to measure = 129kmh
				//if(lastVelocityTime-200>=__HAL_TIM_GET_COUNTER(&htim2)||lastVelocityTime<=__HAL_TIM_GET_COUNTER(&htim2)){//acceleration rate limit
				VelocityTimeSum+=__HAL_TIM_GET_COUNTER(&htim2);
				VelocityAvgI++;
				VelocityTime=__HAL_TIM_GET_COUNTER(&htim2);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

				/*if(VelocityAvgI==4){   //averaging done in Get_VelocityTimeAvg() now
				        		VelocityTime=VelocityTimeSum/4;
				        		VelocityTimeSum=0;
				        		VelocityAvgI=1;
				        	}*/
				lastVelocityTime=VelocityTime;
				__HAL_TIM_SET_COUNTER(&htim2,0);


				WheelSpinCounter++;
				if(WheelSpinCounter==((100*EncNumberOfPulses)-1)){
					WheelSpinCounter=0;
					mileage+=(WheelCircumference*100)/1000;
					trip+=(WheelCircumference*100)/1000;
					//EEPROM_SPI_WriteBuffer((uint8_t*) &mileage, (uint16_t)8, (uint16_t)8);   //can't write to EEPROM in interrupt - config_assert
					//EEPROM_SPI_WriteBuffer((uint8_t*) &trip, (uint16_t)16, (uint16_t)8);
				}
			}

		}

	}
	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_CC_IRQn 0 */
	input_capture = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1); //read TIM2 channel 1 capture value

	__HAL_TIM_SET_COUNTER(&htim1, 0); //reset counter after input capture interrupt occurs

	/* USER CODE END TIM1_CC_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_CC_IRQn 1 */

	/* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  //////////////////////disable timer if wheel didn't spin for 2 seconds/////////////////////////
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2,0);
  VelocityTimeAvg=0;
  MotorcycleStopped=1;
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */

	//calculate dial position
	uint32_t StepperPosition=RPM/22;

	if(stepsToGo!=0){
		stepsToGo=(StepperPosition-lastPosition)+stepsToGo;
	}
	else{stepsToGo=(StepperPosition-lastPosition);}
	lastPosition=StepperPosition;

	if(stepsToGo==0){stepper_delay=STEPPER_MAX_DELAY;}  //if dial on position just wait
	//DIAL DECCELARATION
	else{
		stepper_delay=(STEPPER_DECCELERATION/abs(stepsToGo))+STEPPER_MIN_DELAY;  //lower position error-lower the motor speed
		if(stepper_delay>STEPPER_MAX_DELAY){stepper_delay=STEPPER_MAX_DELAY;} //prevent delay from being too big
	}
	//DIAL ACCELERATION
	stepperAccelDelay=lastStepperDelay-stepper_delay; //calculate change in motor spped
	//if motor is accelerating add some delay between steps to slow it down
	if(stepperAccelDelay>10){stepper_delay+=stepperAccelDelay;lastStepperDelay-=STEPPER_ACCELERATION;}
	else{lastStepperDelay=stepper_delay;}
	//Make 1 step of the motor
	if(stepsToGo>0){
		StepperGoOneStep(1);
		stepsToGo-=1;
	}
	if(stepsToGo<0){
		StepperGoOneStep(0);
		stepsToGo+=1;
	}
	//wait for a calculated delay before making next step
	__HAL_TIM_SET_AUTORELOAD(&htim3,stepper_delay);
	lastStepsToGo=stepsToGo;

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */
	__HAL_TIM_SET_COUNTER(&htim3,0);
	/* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

uint32_t Get_IC_Value(){

	if(__HAL_TIM_GET_COUNTER(&htim1)>15000 || input_capture>15000){  //over 15000 signal is >60Hz
		return 0;
	}
	else{

		return input_capture;
	}

}

uint32_t Get_VelocityTimeAvg() {
	if (MotorcycleStopped == 0) {
		if (VelocityAvgI > 1) { //if captured at least 2 samples, recalculate average value
			VelocityTimeAvg = VelocityTimeSum / VelocityAvgI;
			VelocityTimeSum = VelocityTime; //add latest measurement to sum to avoid having sum value 0
			VelocityAvgI = 1;
		}
		return VelocityTimeAvg;
	} else {
		return 0; //return 0 to indicate motorcycle is stopped, timer is disabled
	}
}


void StepperGoOneStep(uint32_t dir){
	if(dir!=0){  //zgodnie z ruchem wskazowek zegara
		switch(lastStep){

		case 2:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //1
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
			lastStep=1;
			break;

		case 3:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //2
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=2;
			break;

		case 4:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //3
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=3;
			break;

		case 5:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //4
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=4;
			break;

		case 6:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //5
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=5;
			break;

		case 1:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //6
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
			lastStep=6;
			break;

		}
	}
	else{ //przeciwnie z ruchem wskazowek zegara

		switch(lastStep){

		case 5:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //6
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
			lastStep=6;
			break;

		case 4:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //5
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=5;
			break;

		case 3:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //4
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=4;
			break;

		case 2:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET); //3
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=3;
			break;

		case 1:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //2
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_RESET);
			lastStep=2;
			break;

		case 6:
			HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_SET); //1
			HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, GPIO_PIN_SET);
			lastStep=1;
			break;
		}
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
