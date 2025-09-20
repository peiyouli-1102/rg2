/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_mpu.h"
#include "CAN_receive.h"
#include "dsptch.h"
#include "pid.h"
#include "ustreceive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t revbuf[15];
uint8_t Revbuf[1];
uint8_t message[20];
int8_t msg_pt;
prefix_type_t prefix_type;
int32_t result;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dsstep(int32_t step_num);

void DO_WHAT_YOU_SHOULD_DO_2(void){
	if( Revbuf[0] == '$'){
		msg_pt = -1 ;
		return ;
	}
	if( Revbuf[0] == '#'){
		message[msg_pt+1]='\0';
//		printf("ok\r\n");
		result = extract_number(message, msg_pt+1, &prefix_type);
		if ((result != 0) && (Time.tasks[4].task_name[0]=='s')){
			switch(prefix_type){
				case PREFIX_RIGHT :
					printf("right:%d \r\n",result);
					Time.tasks[4] = (struct Task){_4rgt, "right", 1, Time.ms};
					break ;
				case PREFIX_STEP :
					printf("step:%d \r\n",result);
					dsstep(result);
					break ;
				case PREFIX_FORWARD :
					printf("forward:%d \r\n",result);
					Time.tasks[4] = (struct Task){_4fwd, "forward", 1, Time.ms};
					break ;
				case PREFIX_TURN :
					printf("turn:%d \r\n",result);
					Time.tasks[4] = (struct Task){_4rnd, "round", 1, Time.ms};
					break ;
				case PREFIX_FISH :
					printf("fish:%d \r\n",result);
					Time.tasks[5] = (struct Task){_5get, "fishing", 1, Time.ms};
					get_stat = result ;
					break ;
				case PREFIX_SHOOT :
					printf("shoot:%d \r\n",result);
					Time.tasks[7] = (struct Task){_7shoot, "shoot", 1, Time.ms};
					break ;
				case PREFIX_UNKNOWN :
					printf("unknown:%d \r\n",result);
					break ;
			}
		}else {
			printf("WRONG!!\r\n");
		}
		msg_pt = -2 ;
	}
	if( msg_pt == -2 ){
		return ;
	}
	msg_pt ++ ;
	if((msg_pt >= 0 )&&(msg_pt < 20)){
		message[msg_pt]=Revbuf[0];
	}
}

void dsstep(int32_t step_num){
	printf("here");
  if(step_num == 1){
    Time.tasks[7] = (struct Task){_7step1, "step1", 1, Time.ms};
    printf("step1_begin");
  }else if(step_num == 2){
    Time.tasks[7] = (struct Task){_7step2, "step2", 1, Time.ms};
  }else if(step_num == 3){
    Time.tasks[7] = (struct Task){_7step3, "step3", 1, Time.ms};
  }else if(step_num == 4){
    Time.tasks[7] = (struct Task){_7step4, "step4", 1, Time.ms};
  }
}

void disshoot(int32_t shoot_num){
  if(shoot_num == 1){
    Time.tasks[7] = (struct Task){_6shoot, "shoot", 1, Time.ms};
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART2)
	{
		DO_WHAT_YOU_SHOULD_DO_2();
		HAL_UART_Receive_IT(&huart2, Revbuf, sizeof(Revbuf));//串口中断接收函数
	}
}

void Init_all(void){
//    Init_schdl();
    HAL_UART_Receive_IT(&huart2, Revbuf, sizeof(Revbuf));
    HAL_TIM_Base_Start_IT(&htim5);
    can_filter_init();
    pid_angle_init();

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//舵机
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//第一对发射轮
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);					//推杆

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 570);		//600
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 , GPIO_PIN_RESET);	//线盘
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 , GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 , GPIO_PIN_RESET);	//电磁铁
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9 , GPIO_PIN_SET);		//灯
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);	//推杆
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_RESET);	//伸缩杆  	后降
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);			//	后升
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);			//	前降
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);			//	前升
}

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Init_all();

  HAL_Delay(5000);
//  result =-2000 ;
  get_part = 2 ;
  printf("I am Here\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Init_schdl();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Time.ms = Time.timer / MAXTASK;  // 计算当前毫秒数
		Time.t = Time.timer % MAXTASK;   // 计算当前时间片索引 (0-7)

		if(check()){
			dispatch(); // 执行当前时间片的任务
		}
		watchdog_check();
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
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
