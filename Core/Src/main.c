/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "constants.h"
#include "structs.h"
#include <string.h>
#include <stdio.h>
#include "bno055_stm32.h"
#include "bno055.h"
#include "LED.h"
#include "motor_message.h"

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

/* USER CODE BEGIN PV */
CANRxMessage can1_rx, can2_rx;
CANTxMessage FL_P_MSG,FL_W_MSG,FR_P_MSG,FR_W_MSG,BL_P_MSG,BL_W_MSG,BR_P_MSG,BR_W_MSG;
Robot_Struct robot;

LED_Struct leds;

uint8_t Serial2RxBuffer[1];
uint8_t sbus_rxbuffer[50];
uint8_t sbus2_rxbuffer[50];

uint32_t TxMailbox1, TxMailbox2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	printf("Swerve\r\n");

	uint32_t reset_cause = RCC->CSR;
	if (reset_cause & RCC_CSR_BORRSTF) {
	  printf("Reset cause: Brown-out reset\r\n");
	}
	else{
		printf("Power cycle please\r\n");
		while(1){
			//Force user to power cycle after reflashing firmware
		}
	}
    // Clear the reset flags after reading them
    __HAL_RCC_CLEAR_RESET_FLAGS();

	can_rx_init(&can1_rx, &CAN_H1);
	can_rx_init(&can2_rx, &CAN_H2);

	can_tx_init(&FL_P_MSG,FL_P_ID);
	can_tx_init(&FL_W_MSG,FL_W_ID);
	can_tx_init(&FR_P_MSG,FR_P_ID);
	can_tx_init(&FR_W_MSG,FR_W_ID);
	can_tx_init(&BL_P_MSG,BL_P_ID);
	can_tx_init(&BL_W_MSG,BL_W_ID);
	can_tx_init(&BR_P_MSG,BR_P_ID);
	can_tx_init(&BR_W_MSG,BR_W_ID);

	HAL_CAN_Start(&CAN_H1); //start CAN
	HAL_CAN_Start(&CAN_H2); //start CAN

	HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1); //Serial via STLink

	HAL_UART_Receive_DMA(&huart1, sbus_rxbuffer, SBUS_BUF_LEN);
	HAL_UART_Receive_DMA(&huart3, sbus2_rxbuffer, SBUS_BUF_LEN);

	//Loop timing
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((htim1.Instance->ARR))*0.5f);

	//3.3V Aux PA7
	LED_Init(&leds);
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, leds.fData, sizeof(leds.fData) / sizeof(uint32_t));
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

	//RC PWM 5V Aux
	HAL_TIM_PWM_Start(&htim3, PC6_PWM); //PC6
	HAL_TIM_PWM_Start(&htim3, PC7_PWM); //PC7
	HAL_TIM_PWM_Start(&htim3, PC8_PWM); //PC8
	HAL_TIM_PWM_Start(&htim3, PC9_PWM); //PC9

	//Zero Robot Commands
	robot.xVel = 0.0;
	robot.yVel = 0.0;
	robot.yawVel = 0.0;
	robot.lastBRAngle = 0.0;
	robot.lastBLAngle = 0.0;
	robot.lastFRAngle = 0.0;
	robot.lastFLAngle = 0.0;

	memset(&robot.FL_P_CTRL, 0, sizeof(robot.FL_P_CTRL));
	memset(&robot.FR_P_CTRL, 0, sizeof(robot.FR_P_CTRL));
	memset(&robot.BL_P_CTRL, 0, sizeof(robot.BL_P_CTRL));
	memset(&robot.BR_P_CTRL, 0, sizeof(robot.BR_P_CTRL));
	memset(&robot.FL_W_CTRL, 0, sizeof(robot.FL_W_CTRL));
	memset(&robot.FR_W_CTRL, 0, sizeof(robot.FR_W_CTRL));
	memset(&robot.BL_W_CTRL, 0, sizeof(robot.BL_W_CTRL));
	memset(&robot.BR_W_CTRL, 0, sizeof(robot.BR_W_CTRL));

	robot.FL_W_PWM.min = PWM_MIN;
	robot.FL_W_PWM.max = PWM_MAX;
	robot.FL_W_PWM.microseconds = PWM_ZERO;
	robot.FR_W_PWM.min = PWM_MIN;
	robot.FR_W_PWM.max = PWM_MAX;
	robot.FR_W_PWM.microseconds = PWM_ZERO;
	robot.BL_W_PWM.min = PWM_MIN;
	robot.BL_W_PWM.max = PWM_MAX;
	robot.BL_W_PWM.microseconds = PWM_ZERO;
	robot.BR_W_PWM.min = PWM_MIN;
	robot.BR_W_PWM.max = PWM_MAX;
	robot.BR_W_PWM.microseconds = PWM_ZERO;

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(2);
	  if(robot.sendCounter==1){
		bno055_vector_t e = bno055_getVectorEuler();
		bno055_vector_t g = bno055_getVectorGyroscope();
		robot.yaw = e.x;
		robot.roll = e.z;
		robot.pitch = -e.y;
		robot.yaw_vel = (0.1f*-g.z)+0.9f*robot.yaw_vel; //low pass
		robot.roll_vel = -g.x;
		robot.pitch_vel = g.y;
		robot.sendCounter = 0;
	  }
	  if(resetBlink == 1){
		  resetBlink = 0;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET );
		  HAL_Delay(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET );
	  }

	  switch(robot.radio_rx.right_select){
	  	  case(2):
	  			  leds.isBlue = 1;
	  	  	  	  leds.isRed = 0;
	  	  	  	  break;
	  	  case(1):
	  			  leds.isBlue = 0;
	  	  	  	  leds.isRed = 1;
	  	  	  	  break;
	  	  case(0):
	  			  leds.isBlue = 0;
	  	  	  	  leds.isRed = 0;
	  	  	  	  break;
	  }
	  if(leds.isBlue){
		  if(robot.is_headless){
			  LED_ShowRobotOrientation(&leds,-(robot.yaw-robot.headlessAngle),0,0,255);
		  }
		  else{
			  LED_ShowRobotOrientation(&leds,0.0,0,0,255);
		  }
	  }
	  else if(leds.isRed){
		  if(robot.is_headless){
			  LED_ShowRobotOrientation(&leds,-(robot.yaw-robot.headlessAngle),255,0,0);
		  }
		  else{
			  LED_ShowRobotOrientation(&leds,0.0,255,0,0);
		  }
	  }
	  else{
		  LED_RainbowRoll(&leds, leds.rainbowOffset);
		  leds.rainbowOffset += 1;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
