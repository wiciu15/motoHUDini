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
#include "freertos.h"
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"
#include "stm32f1xx_it.h"
#include <math.h>
#include <stdlib.h>
#include "MOTOHUD_CONFIG.h"
#include "adc.h"
#include "usbd_cdc_if.h"
#include "rtc.h"
#include "ILI9341/derbilogo.h"
#include "ILI9341/blinkerIcon.h"
#include "ILI9341/highBeamIcon.h"
#include "EEPROM_SPI.h"
#include "spi.h"
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

/////////////////////////////
//////GLOBAL  VARIABLES//////
/////////////////////////////

//RPM CALCULATION AND AVERAGING
uint32_t RPM=0;
uint32_t RPMAvgSum=0;
float RPMfreq=0;
int RPMNow=0;
uint32_t RPMAvg_i=0;
uint32_t lastRPM=1;


//TEMPERATURE
uint32_t tempNowOil=0;
uint32_t tempNowAir=0;
double voltOil=0;
double voltAir=0;
uint32_t tempOilADC=0;
uint32_t tempAvgOil=0;
uint32_t tempAirADC=0;
uint32_t tempAvgAir=0;
uint32_t tempAvg_i=0;

//STRINGS FOR VALUES
char RPMString[4];
char VelocityString[3];
char tempOilString[4];
char tempAirString[4];
char mileageString[10];
char tripString[10];

//VELOCITY RELATED
uint8_t EncNumberOfPulses=NUM_OF_PULSES_PER_REV;   //number of pulses from hall sensor per 1 wheel rev
float WheelCircumference=WHEEL_CIRCUMFERENCE;
uint8_t lastVelocity=1;

//BUFFER FOR DATA FROM ADC
uint32_t ADCBUF[3];

//USB BUFFERS
uint8_t UsbReceivedData[40];
uint8_t UsbReceivedDataFlag;

//CLOCK REFRESH WHEN MINUTES CHANGE
uint8_t lastMinute=0;

//ODOMETER
double mileage=0;
double trip=0;
double lastTrip=0;

//EEPROM
	extern uint8_t RxBuffer[8];       //eeprom receive data buffer
	extern uint8_t EEPROM_StatusByte;


	//INDICATORS
uint8_t neutralState=0;
uint8_t lastNeutralState=1;
uint8_t highBeamState=1;
uint8_t lastHighBeamState=0;
uint8_t blinkerState=1;
uint8_t lastBlinkerState=0;

	//MENU
uint8_t btnModeSec=0;
uint8_t btnSetSec=0;
uint8_t timeSettingMode=0;
uint8_t timeSettingModeHour=0;
uint8_t timeSettingModeMinute=0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LCDMainHandle;
osThreadId StepperHandle;
osThreadId startupHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void SendUsbMessage(uint8_t message[]);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void vLCDMain(void const * argument);
void vStepp(void const * argument);
void vStartup(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LCDMain */
  osThreadDef(LCDMain, vLCDMain, osPriorityBelowNormal, 0, 1024);
  LCDMainHandle = osThreadCreate(osThread(LCDMain), NULL);

  /* definition and creation of Stepper */
  osThreadDef(Stepper, vStepp, osPriorityAboveNormal, 0, 64);
  StepperHandle = osThreadCreate(osThread(Stepper), NULL);

  /* definition and creation of startup */
  osThreadDef(startup, vStartup, osPriorityRealtime, 0, 64);
  startupHandle = osThreadCreate(osThread(startup), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  for(;;)
  {
osDelay(1000);

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


	ILI9341_Init();
	ILI9341_Fill_Screen(BLACK);  //LCD init and graphics memory cleanup
	osDelay(30);

	///////////BACKLIGHT ON////////////
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  //dial gauge backlight is delayed-switching on now
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  //tft backlight is delayed-switching on now



	//////////DRAWING SPLASH SCREEN WITH LOGO/////
	ILI9341_Draw_Rectangle(56, 50, 40, 120, LIGHTGREY);
	ILI9341_Draw_Rectangle(96, 50, 155, 120, RED);
	ILI9341_DrawBitmap((const char*)logo, 125,70,96,78,RED,WHITE);


	//ILI9341_Draw_Text("Welcome", ILI9341_SCREEN_WIDTH/2-70, ILI9341_SCREEN_HEIGHT/2+20, WHITE, 3, BLACK);
	//ILI9341_Draw_Text("on board", ILI9341_SCREEN_WIDTH/2-80, ILI9341_SCREEN_HEIGHT/2+45, WHITE, 3, BLACK);


	osDelay(3000);
	ILI9341_Draw_Rectangle(50, 50, 205, 130, BLACK);  //wait 2s and get rid of the logo

	//////////////////////////RTC power failure///////////////////
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)!=0x32FE){
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Constant power lost", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		ILI9341_Draw_Text("Set new time", ILI9341_SCREEN_WIDTH/2-60, ILI9341_SCREEN_HEIGHT/2+50, RED, 2, BLACK);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32FE);
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}

	//////////////ADC calibration and start/////////////////////////
	HAL_ADC_Stop(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBUF, 3);

	//////////////////////////////EEPROM CFG/////////////////////
	EEPROM_SPI_INIT(&hspi2);


	//double m=28622.70;    //write initial values to blank EEPROM
	//double t=1.61;
	//EEPROM_SPI_WriteBuffer((uint8_t*) &m, (uint16_t)16, (uint16_t)8);
	//EEPROM_SPI_WriteBuffer((uint8_t*) &t, (uint16_t)24, (uint16_t)8);

	//read saved mileage and trip from eeprom

	if(EEPROM_SPI_ReadBuffer(RxBuffer, (uint16_t)16, (uint16_t)8)==EEPROM_STATUS_COMPLETE){ //read mileage as uint8_t array from address 16
		memcpy(&mileage, RxBuffer, sizeof(double));	  //cast uint8_t array to double
	}
	else{                                                                                            //communication error
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Internal storage error", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		ILI9341_Draw_Text("Configure again", ILI9341_SCREEN_WIDTH/2-60, ILI9341_SCREEN_HEIGHT/2+50, RED, 2, BLACK);
		mileage=0;
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}
	if(mileage<0 || mileage >1000000){                                                                //mileage value incorrect
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Mileage value incorrect", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		mileage=0;
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}
	if(mileage!=mileage){                                                                               //EEPROM value not a number(NaN)
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Internal storage error", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		ILI9341_Draw_Text("Configure again", ILI9341_SCREEN_WIDTH/2-60, ILI9341_SCREEN_HEIGHT/2+50, RED, 2, BLACK);
		mileage=0;
		//EEPROM_SPI_WriteBuffer((uint8_t*) &mileage, (uint16_t)16, (uint16_t)8);
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}


	EEPROM_SPI_ReadBuffer(RxBuffer, (uint16_t)24, (uint16_t)8);   //read trip from eeprom
	memcpy(&trip, RxBuffer, sizeof(double));


	if(trip<0 || trip >10000){       //EEPROM value invalid
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Internal storage error", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		trip=0;
		EEPROM_SPI_WriteBuffer((uint8_t*) &trip, (uint16_t)24, (uint16_t)8);
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}
	if(trip!=trip){                                                                               //EEPROM value not a number(NaN)
		ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, ILI9341_SCREEN_HEIGHT/2-80, 60, 72,RED);
		ILI9341_Draw_Text("!", ILI9341_SCREEN_WIDTH/2, ILI9341_SCREEN_HEIGHT/2-80, WHITE, 9, RED);
		ILI9341_Draw_Text("Internal storage error", 40, ILI9341_SCREEN_HEIGHT/2+30, RED, 2, BLACK);
		trip=0;
		EEPROM_SPI_WriteBuffer((uint8_t*) &trip, (uint16_t)24, (uint16_t)8);
		osDelay(3000);
		ILI9341_Fill_Screen(BLACK);
	}



	///////DRAWING STATIC GRAPHICS///////
	ILI9341_Draw_Horizontal_Line(1, 49, 319, WHITE);     //horizontal lines
	ILI9341_Draw_Horizontal_Line(1, 74, 319, WHITE);
	ILI9341_Draw_Horizontal_Line(1, 170, 319, WHITE);
	ILI9341_Draw_Horizontal_Line(1, 205, 319, WHITE);
	ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-75, ILI9341_SCREEN_HEIGHT/2, 40, 16, WHITE);    //kmh white rectangle
	ILI9341_Draw_Text("kmh", ILI9341_SCREEN_WIDTH/2-75, ILI9341_SCREEN_HEIGHT/2, BLACK, 2, WHITE);



	////////FORCE TIME DRAWING AFTER STARTUP WHEN MINUTE IN RTC = 0/////////
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	lastMinute=sTime.Minutes+1;







	while(1){


		//calculation and drawing of temperature
		tempNowOil=ADCBUF[0];    //number of ADC samples
		tempAvgOil+=tempNowOil;
		tempNowAir=ADCBUF[1];
		tempAvgAir+=tempNowAir;
		if(tempAvg_i==19){
			tempOilADC=tempAvgOil/20;     //averaging samples from ADC
			tempAvgOil=0;
			tempAirADC=tempAvgAir/20;
			tempAvgAir=0;
			tempAvg_i=0;

			voltOil=((tempOilADC*3.29)/4095);    //calculate voltage value
			voltAir=((tempAirADC*3.29)/4095);

			double resistanceOil=6284*(1/((3.3/(voltOil))-1));    //measured resistor R22 value into formula
			double resistanceAir=158420*(1/((3.3/(voltAir))-1));  //measured resistor R23 value into formula

			int tempCelsiusOil=(1/((log(resistanceOil/98710)/(3900))+(1/298.15)))-273.15;   //calculating temperature based on thermistor T(R) characteristic
			int tempCelsiusAir=(1/((log(resistanceAir/98710)/(3900))+(1/298.15)))-273.15;

			char* pTempOil=tempOilString;
			char* pTempAir=tempAirString;

			if(tempCelsiusOil<10){pTempOil="--";} //blank if thermistor unplugged
			else{itoa(tempCelsiusOil,pTempOil,10);}
			if(tempCelsiusAir<-10){pTempAir="--";}
			else{itoa(tempCelsiusAir,pTempAir,10);}

			//drawing Oil temp
			ILI9341_Draw_Rectangle( 205, 208, 80, 60, BLACK);
			ILI9341_Draw_Text("oil", 320-(strlen(pTempOil)*19)-55, 215, WHITE, 1, BLACK);
			ILI9341_Draw_Text(pTempOil, 320-(strlen(pTempOil)*19)-28, 207, WHITE, 3, BLACK);
			ILI9341_Draw_Text("0", 320-28, 207, WHITE, 1, BLACK);
			ILI9341_Draw_Text("C", 320-19, 207, WHITE, 3, BLACK);
			//drawing Air temp
			ILI9341_Draw_Rectangle( 40, 207, 100, 60, BLACK);
			ILI9341_Draw_Text(pTempAir, 15, 207, WHITE, 3, BLACK);
			ILI9341_Draw_Text("0", 15+strlen(pTempAir)*19, 207, WHITE, 1, BLACK);
			ILI9341_Draw_Text("C", 23+strlen(pTempAir)*19, 207, WHITE, 3, BLACK);
			ILI9341_Draw_Text("air", 50+strlen(pTempAir)*19, 215, WHITE, 1, BLACK);
		}
		tempAvg_i++;


		//RPM value drawing
		if(RPM<20000 && RPM!=lastRPM){   //RPM is calculated in vStepp task
			lastRPM=RPM;
			char* pRPM=RPMString;
			itoa(RPM, pRPM, 10);
			if(RPM<100)ILI9341_Draw_Rectangle( ILI9341_SCREEN_WIDTH/2-80, 5, 60, 40, BLACK);
			if(RPM<1000){ILI9341_Draw_Rectangle( ILI9341_SCREEN_WIDTH/2-25, 5, 60, 40, BLACK);}
			else{
				ILI9341_Draw_Rectangle( ILI9341_SCREEN_WIDTH/2+8, 5, 60, 40, BLACK);
			}
			ILI9341_Draw_Text(pRPM, ILI9341_SCREEN_WIDTH/2-110, 5, BLACK, 5, WHITE);
			ILI9341_Draw_Text("rpm", ILI9341_SCREEN_WIDTH/2-146, 5, BLACK, 2, WHITE);

		}

		/////////////Velocity drawing//////////////////////

		//if in neutral gear draw N in GREEN
		if(neutralState==1 && lastNeutralState==0){
			ILI9341_Draw_Text("N", ILI9341_SCREEN_WIDTH/2-30, ILI9341_SCREEN_HEIGHT/2-40, GREEN, 10, BLACK);
			ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-80, ILI9341_SCREEN_HEIGHT/2, 45, 16, BLACK); //hide kmh text white background
			lastVelocity=255;
		}
		if(neutralState==0 && lastNeutralState==1){ //force kmh value drawing when changing from neutral to gear
			ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-75, ILI9341_SCREEN_HEIGHT/2, 40, 16, WHITE);    //kmh white rectangle
			ILI9341_Draw_Text("kmh", ILI9341_SCREEN_WIDTH/2-75, ILI9341_SCREEN_HEIGHT/2, BLACK, 2, WHITE);
			lastVelocity=255;
		}
		lastNeutralState=neutralState;
		// if in gear calculate and draw velocity value
		if(neutralState==0){
			uint8_t Velocity;
			char* pVelocity=VelocityString;
			if(Get_VelocityTimeAvg()==0){Velocity=0;}else{
				Velocity=((WheelCircumference/EncNumberOfPulses)/(Get_VelocityTimeAvg()*0.0001))*3.6;} //calculate velocity
			if(Velocity!=lastVelocity){  //if number is valid and different from last calculation convert it to string
				itoa(Velocity, pVelocity, 10);
				//clear last value on screen
				if(lastVelocity>=100 && Velocity <100){ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-30, ILI9341_SCREEN_HEIGHT/2-40, 200, 80, BLACK);}
				if(lastVelocity>=10 && Velocity <10){ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-30, ILI9341_SCREEN_HEIGHT/2-40, 200, 80, BLACK);}

				ILI9341_Draw_Text(pVelocity, ILI9341_SCREEN_WIDTH/2-30, ILI9341_SCREEN_HEIGHT/2-40, WHITE, 10, BLACK);
				lastVelocity=Velocity;
			}
		}


		//Indicators drawing
		if(blinkerState==1 && lastBlinkerState==0){ILI9341_DrawBitmap((const char*)blinkerIcon, 10, ILI9341_SCREEN_HEIGHT/2-30,64,48,BLACK,GREEN);}
		if(blinkerState==0 && lastBlinkerState==1){ILI9341_Draw_Rectangle(10, ILI9341_SCREEN_HEIGHT/2-30, 64, 48, BLACK);}

		if(highBeamState==1 && lastHighBeamState==0){ILI9341_DrawBitmap((const char*)highBeamIcon, ILI9341_SCREEN_WIDTH-48-5,5,48,34,BLACK,BLUE);}
		if(highBeamState==0 && lastHighBeamState==1){ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH-48-5,5,48,34, BLACK);}

		lastBlinkerState=blinkerState;
		lastHighBeamState=highBeamState;

		//read indicators state after drawing to show predefined state right after startup
		if(HAL_GPIO_ReadPin(BLINK_GPIO_Port, BLINK_Pin)==GPIO_PIN_SET){blinkerState=1;}else{blinkerState=0;}
		if(HAL_GPIO_ReadPin(NEUTRAL_GPIO_Port, NEUTRAL_Pin)==GPIO_PIN_SET){neutralState=1;}else{neutralState=0;}
		if(HAL_GPIO_ReadPin(HIBEAM_GPIO_Port, HIBEAM_Pin)==GPIO_PIN_SET){highBeamState=1;}else{highBeamState=0;}

		//time drawing
		if(timeSettingMode==0){
			RTC_TimeTypeDef sTime;
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

			if(lastMinute!=sTime.Minutes){
				char TimeString[6];
				if(sTime.Minutes<10){
					sprintf(TimeString,"%d:0%d",sTime.Hours,sTime.Minutes);
				}else{
					sprintf(TimeString,"%d:%d",sTime.Hours,sTime.Minutes);
				}

				char* pTime=TimeString;


				ILI9341_Draw_Rectangle(ILI9341_SCREEN_WIDTH/2-40, 50, 130,24, BLACK);
				ILI9341_Draw_Text(pTime, ILI9341_SCREEN_WIDTH/2-40, 50, WHITE, 3, BLACK);
				lastMinute=sTime.Minutes;
			}
		}


		//mileage and trip drawing
		if(trip!=lastTrip){

			if(lastTrip!=0){
				if(trip>1000){       //trip zeroes automatically every 1000km
					trip=trip-1000;
				}

				EEPROM_SPI_WriteBuffer((uint8_t*) &mileage, (uint16_t)16, (uint16_t)8);   //write mileage and trip to eeprom
				EEPROM_SPI_WriteBuffer((uint8_t*) &trip, (uint16_t)24, (uint16_t)8);     //after 100 wheel revolutions
			}

			char* pMileage=mileageString;
			itoa(mileage, pMileage, 10);
			ILI9341_Draw_Text(pMileage, 15, 175, WHITE, 3, BLACK);
			ILI9341_Draw_Text("km", 15+(strlen(pMileage)*19), 185, WHITE, 1, BLACK);


			char* pTrip=tripString;
			itoa(trip, pTrip, 10);
			ILI9341_Draw_Text("trip", (320-(strlen(pTrip)*19)-30-35), 180, WHITE, 1, BLACK);   //trip is right aligned so lenght of the string must be subtracted in X dir
			ILI9341_Draw_Text(pTrip, (320-(strlen(pTrip)*19)-30), 175, WHITE, 3, BLACK);
			ILI9341_Draw_Text("km",(320-30) , 190, WHITE, 1, BLACK);




			lastTrip=trip;
		}

		////TIME SETTING


		if(timeSettingMode==1){   //minute setting
			char TimeStringEdit[6];
			if(timeSettingModeMinute<10){
				sprintf(TimeStringEdit,"%d:0%d<",timeSettingModeHour,timeSettingModeMinute);
			}
			else{
				sprintf(TimeStringEdit,"%d:%d<",timeSettingModeHour,timeSettingModeMinute);
			}
			char* pTimeEdit=TimeStringEdit;
			ILI9341_Draw_Text(pTimeEdit, ILI9341_SCREEN_WIDTH/2-40, 50, RED, 3, BLACK);


			if(HAL_GPIO_ReadPin(BTN_SET_GPIO_Port, BTN_SET_Pin)==GPIO_PIN_RESET){
				timeSettingModeMinute++;
				if(timeSettingModeMinute==60)timeSettingModeMinute=0;
			}
			if(btnModeSec>4){timeSettingMode=2;btnModeSec=0;}

		}

		if(timeSettingMode==2){   //hour setting
			char TimeStringEdit[6];
			if(timeSettingModeMinute<10){
				sprintf(TimeStringEdit,">%d:0%d",timeSettingModeHour,timeSettingModeMinute);
			}
			else{
				sprintf(TimeStringEdit,">%d:%d",timeSettingModeHour,timeSettingModeMinute);
			}

			char* pTimeEdit=TimeStringEdit;
			ILI9341_Draw_Text(pTimeEdit, ILI9341_SCREEN_WIDTH/2-40, 50, RED, 3, BLACK);

			if(HAL_GPIO_ReadPin(BTN_SET_GPIO_Port, BTN_SET_Pin)==GPIO_PIN_RESET){
				timeSettingModeHour++;
				if(timeSettingModeHour==24)timeSettingModeHour=0;
			}


			if(btnModeSec>4){
				timeSettingMode=0;
				RTC_TimeTypeDef sTimeEdited;
				sTimeEdited.Hours=timeSettingModeHour;
				sTimeEdited.Minutes=timeSettingModeMinute;
				sTimeEdited.Seconds=0;
				HAL_RTC_SetTime(&hrtc, &sTimeEdited, RTC_FORMAT_BIN);
				lastMinute=timeSettingModeMinute+1;
				btnModeSec=0;
			}

		}

		if(HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin)==GPIO_PIN_RESET){
			btnModeSec++;
		}
		else{
			btnModeSec=0;
		}

		if(btnModeSec>10){
			timeSettingMode=1;
			btnModeSec=0;
			timeSettingModeHour=sTime.Hours;
			timeSettingModeMinute=sTime.Minutes;
			char TimeStringEdit[6];
			if(timeSettingModeMinute<10){
				sprintf(TimeStringEdit,"%d:0%d",timeSettingModeHour,timeSettingModeMinute);
			}
			else{
				sprintf(TimeStringEdit,"%d:%d",timeSettingModeHour,timeSettingModeMinute);
			}
			char* pTimeEdit=TimeStringEdit;
			ILI9341_Draw_Text(pTimeEdit, ILI9341_SCREEN_WIDTH/2-40, 50, RED, 3, BLACK);
			osDelay(1000);

		}

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  //status led blink
		osDelay(250);                             //main screen refresh delay in ms




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
	osDelay(4500);  //startup delay for dial calibration in vStartup task
  for(;;)
  {

	  if(Get_IC_Value()!=0){
		  RPMfreq=1000000/(Get_IC_Value());  //get RPM signal frequency
	  }
	  else{
		  RPMfreq=0;
	  }

	  RPMNow=(RPMfreq*10.27)-255;				//calculate RPM value
	  if(RPMNow>260)RPMAvgSum+=RPMNow;          //if RPM value is positive add it to average

	  RPMAvg_i++;
	  if(RPMAvg_i==19){					//after 20 measurements calculate average
	  	RPM=RPMAvgSum/20;
	  	RPM=RPM/25;                    //resolution of measurement is 25 RPM
	  	RPM=RPM*25;
	  	RPMAvgSum=0;
	  	RPMAvg_i=0;

	  	}
	  osDelay(2); //averaged RPM value every 40ms (20*2)

  }
  /* USER CODE END vStepp */
}

/* USER CODE BEGIN Header_vStartup */
/**
* @brief Function implementing the startup thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartup */
void vStartup(void const * argument)
{
  /* USER CODE BEGIN vStartup */

//////Dial calibration during startup////////

	RPM=4000;
	osDelay(700);

	RPM=8000;
	osDelay(700);

	RPM=12000;
	osDelay(700);

	RPM=15400;
	osDelay(700);


	RPM=0;
	vTaskDelete(startupHandle);
  /* USER CODE END vStartup */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void SendUsbMessage(uint8_t message[]){
	uint8_t mesLenght=0;
		uint8_t mesData [40];
		mesLenght=sprintf(mesData, "%s\n\r",message);
		CDC_Transmit_FS(mesData, mesLenght);
}
/*void StepperGoOneStep(uint32_t dir, uint32_t speed){
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
}*/

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
