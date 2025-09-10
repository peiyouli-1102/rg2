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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../User/bsp_can.h"
#include "../User/CAN_receive.h"
#include "pid.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t TxBuf1[] = "111144" ;
uint8_t TxBuf2[] = "222233" ;
uint8_t Rxbuf[1] = {0};
uint8_t Rxbuf2[20] = {0};
uint8_t c ;

int t=0;
int32_t flag = 0 ;
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

uint8_t check_motor(uint8_t sign);
void fishing(int32_t t1);
void fishing2(int32_t t1);
void upstair(int16_t t1);
void upstair2(int16_t t1);
void downstair(int16_t t1);
void shoot(void);
void Init_all(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DO_WHAT_YOU_SHOULD_DO(void)
{
	HAL_UART_Transmit_IT(&huart1, Rxbuf, sizeof(Rxbuf));//串口中断发送函数
	HAL_UART_Transmit_IT(&huart1, TxBuf1, sizeof(TxBuf1));
	printf("111???");

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	if(UartHandle->Instance == USART1)
	{
		if(Rxbuf[0]=='$'){
			HAL_UART_Transmit_IT(&huart1, Rxbuf2, c );
			c = 0 ;
		}else{
			Rxbuf2[c] =  Rxbuf[0];
		}
		HAL_UART_Receive_IT(&huart1, Rxbuf, sizeof(Rxbuf));//串口中断接收函数
	}
}
uint8_t check_motor(uint8_t sign){
	int16_t speed ;
	speed = motor_chassis[sign].speed_rpm ;
	if(speed > 15){
		return 1 ;
	}
	if(speed < -15){
		return 1 ;
	}
	return 0 ;
}
void fishing(int32_t t1){
//	 拾取--------------------------------begin------------------------------------------------
//		  										舵机

    if(t >t1 && t< t1+ 500 ){
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (675*(t)+420*(500-t))/500);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (83*(t)+620*(500-t))/500);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
    }else if(t > t1+ 500 && t < t1+ 23200){
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 83);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
    }else if(t >t1+ 23200 && t< t1+ 23450 ){
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (593*(t-23200)+83*(23450-t))/250);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
    }else if(t >t1+ 23450 && t< t1+ 23700 ){
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (295*(t-23450)+675*(23700-t))/250);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
    }else if(t >t1+ 23300 && t< t1+ 33100 ){
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 295);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 450);
    }else {
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
    }
    //										电磁铁/收放线
    if(t > t1+5100 && t < t1+10900){				//放5800
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    	flag = 1 ;
    }else if(t > t1+10900 && t < t1+11900){
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    	flag = 2 ;
    }else if(t > t1+11900 && t < t1+23100){	//收11000
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    	flag = 3 ;
    }else if(t > t1+23100 && t < t1+27000){
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    	flag = 4 ;
    }else if(t > t1+27000 && t < t1+32000){	//放5000
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    	flag = 5 ;
    }else if(t > t1+32000 && t < t1+33000){
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    	flag = 6 ;
    }
//	 拾取------------------------------end--------------------------------------------------------
}

void fishing2(int32_t t1){
	//	 拾取--------------------------------begin------------------------------------------------
	//		  										舵机
		if(t >t1 && t< t1+ 5500 ){
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
		}else if(t >t1 + 5500 && t< t1+ 6000 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (675*(t-2500)+420*(3000-t))/500);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (83*(t-2500)+620*(3000-t))/500);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
	    }else if(t > t1+ 6000 && t < t1+ 17000){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 83);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
	    }else if(t >t1+ 17000 && t< t1+ 17250 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (593*(t-23200)+83*(23450-t))/250);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
	    }else if(t >t1+ 17250 && t< t1+ 17500 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (295*(t-23450)+675*(23700-t))/250);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
	    }else if(t >t1+ 17500 && t< t1+ 23500 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 290);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 450);
	    }else {if(t>t1){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);}
	    }
	    //										电磁铁/收放线
	    if(t > t1+500 && t <= t1+4900) {						//收4400
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	       	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
 	    	flag = 1 ;
	    }else if(t > t1+4900 && t <= t1+8000){				//停5500
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	       	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
 	    	flag = 2 ;
	    }else if(t > t1+8000 && t <= t1+10800){				//放3200
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 3 ;
	    }else if(t > t1+10800 && t <= t1+13000){		 	//停1500
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    	flag = 4 ;
	    }else if(t > t1+13000 && t <= t1+16900){			//收3800
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	flag = 5 ;
	    }else if(t > t1+16900 && t <= t1+21000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	flag = 4 ;
	    }else if(t > t1+21000 && t <= t1+23000){			//放4000
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 5 ;
	    }else if(t > t1+23000 && t <= t1+24000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    	flag = 6 ;
	    }else if(t > t1+24000 && t <= t1+25000){			//放4000
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 5 ;
	    }else if(t > t1+25000 && t <= t1+26000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    	flag = 6 ;
	    }
	//	 拾取------------------------------end--------------------------------------------------------
	}
void fishing3(int32_t t1){
	//	 拾取--------------------------------begin------------------------------------------------
	//		  										舵机
		if(t >t1 && t< t1+ 9500 ){
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
		    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
		}else if(t >t1 + 9500 && t< t1+ 10000 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (675*(t-2500)+420*(3000-t))/500);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (83*(t-2500)+620*(3000-t))/500);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
	    }else if(t > t1+ 10000 && t < t1+ 26000){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 83);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);
	    }else if(t >t1+ 26000 && t< t1+ 26250 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 675);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (593*(t-23200)+83*(23450-t))/250);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
	    }else if(t >t1+ 26250 && t< t1+ 26500 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (295*(t-23450)+675*(23700-t))/250);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (450*(t-23200)+650*(23700-t))/500);
	    }else if(t >t1+ 26500 && t< t1+ 32500 ){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 290);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 593);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 465);
	    }else {if(t>t1){
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);}
	    }
	    //										电磁铁/收放线
	    if(t > t1+500 && t <= t1+9500) {						//收4400
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	       	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
 	    	flag = 1 ;
	    }else if(t > t1+9500 && t <= t1+11000){				//停5500
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	       	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
 	    	flag = 2 ;
	    }else if(t > t1+11000 && t <= t1+17000){			//放2800
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 3 ;
	    }else if(t > t1+17000 && t <= t1+18000){		 	//停1500
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    	flag = 4 ;
	    }else if(t > t1+18000 && t <= t1+26000){			//收3900
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	flag = 5 ;
	    }else if(t > t1+26000 && t <= t1+27000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	flag = 4 ;
	    }else if(t > t1+27000 && t <= t1+32000){			//放2000
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 5 ;
	    }else if(t > t1+32000 && t <= t1+33000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    	flag = 6 ;
	    }else if(t > t1+33000 && t <= t1+34000){			//放4000
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	    	flag = 5 ;
	    }else if(t > t1+34000 && t <= t1+35000){
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    	flag = 6 ;
	    }
	//	 拾取------------------------------end--------------------------------------------------------
	}
void upstair(int16_t t1){
//	 上楼----------------------------------begin-------------------------------------------------------------
	if(t<t1){
		return ;
	}
	if(t<650+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(t<750+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	 	goal_ch[0] = goal_ch[3] = -3000 ;
		goal_ch[1] = goal_ch[2] = 3000 ;
	}else if(t<1400+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else if(t<1500+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	 	goal_ch[0] = goal_ch[3] = -1800 ;
		goal_ch[1] = goal_ch[2] = 1800 ;
	}else if(t<2150+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_SET);
	}else if(t<2180+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_RESET);
	}else if(t<2600+t1){
		goal_ch[0] = goal_ch[3] = -2000 ;
		goal_ch[1] = goal_ch[2] = 2000 ;
	}else if(t<2620+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
	}else if(t<2680+t1){
		goal_ch[0] = goal_ch[3] = -1000 ;
		goal_ch[1] = goal_ch[2] = 1000 ;
	}else{
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
	}
	pid_chassis();
//	上楼-------------------------------------end--------------------------------------------------------------------
}
void upstair2(int16_t t1){
//	 上楼----------------------------------begin-------------------------------------------------------------
	if(t<t1){
		return ;
	}
	if(t<650+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(t<750+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	}else if(t<850 + t1){
	 	goal_ch[0] = goal_ch[3] = -5000 ;
		goal_ch[1] = goal_ch[2] = 5000 ;
	}else if(t<1500+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else if(t<1700+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	 	goal_ch[0] = goal_ch[3] = -3000 ;
		goal_ch[1] = goal_ch[2] = 3000 ;
	}else if(t<2350+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_SET);
	}else if(t<2380+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , GPIO_PIN_RESET);
	}else if(t<2800+t1){
		goal_ch[0] = goal_ch[3] = -2000 ;
		goal_ch[1] = goal_ch[2] = 2000 ;
	}else if(t<2920+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
	}else if(t<2980+t1){
		goal_ch[0] = goal_ch[3] = -1000 ;
		goal_ch[1] = goal_ch[2] = 1000 ;
	}else{
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
	}
	pid_chassis();
//	上楼-------------------------------------end--------------------------------------------------------------------
}
void downstair(int16_t t1){
	if(t<t1){
		return ;
	}
	if(t<360+t1){
		goal_ch[0] = goal_ch[3] = 2000 ;
		goal_ch[1] = goal_ch[2] = -2000 ;
	}else if(t<1000+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}else if(t<1080+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		goal_ch[0] = goal_ch[3] = 2000 ;
		goal_ch[1] = goal_ch[2] = -2000 ;
	}else if(t<1730+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	}else if(t<1790+t1){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
		goal_ch[0] = goal_ch[3] = 2000 ;
		goal_ch[1] = goal_ch[2] = -2000 ;
	}else if(t<2440+t1){
		goal_ch[0] = goal_ch[3] = 0 ;
		goal_ch[1] = goal_ch[2] = 0 ;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		goal_ch[0] = goal_ch[3] = 2000 ;
		goal_ch[1] = goal_ch[2] = -2000 ;
	}
	pid_chassis();
}
void shoot(void){
	//发射----------------------------------begin-----------------------------------------------------------------------
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 420);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 620);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 650);

//	goal = 2470 ;
	goal = 3830 ;
	if(t<5000){
		if(check_motor(4)&check_motor(5)){
			pid_shoot();
		}else if(check_motor(4)){
			CAN_cmd_shoot(100, -600);
		}else if(check_motor(5)){
			CAN_cmd_shoot(600,-100);
		}else{
			CAN_cmd_shoot(600,-600);
		}
	}
//	if(check_motor(4)&check_motor(5)){
//		pid_shoot();
//	}else if(check_motor(4)){
//		CAN_cmd_shoot(100, -600);
//	}else if(check_motor(5)){
//		CAN_cmd_shoot(600,-100);
//	}else{
//		CAN_cmd_shoot(600,-600);
//	}

//	if((t > 1000) && (t < 1650)){
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
//		flag = 1 ;
//	}else if((t > 1650 ) && (t < 2300)){
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
//	}else {
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
//	}
	if((t > 1000) && (t < 2850)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
		flag = 1 ;
	}else if((t > 2850 ) && (t < 4700)){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 900);
	}else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	}

//	if(t < 7000){
////		goal = 3905 ;
//		goal = 2885 ;
//		pid_shoot();
//	}

	if((t > 1000) && (t < 5000)){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 600);
	}else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
	}
//发射------------------------------------end-------------------------------------------------------
}
void Init_all(void){
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

	  can_filter_init();
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//舵机
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//第一对发射轮
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);					//推杆

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 420);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 620);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 650);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

	  HAL_UART_Receive_IT(&huart1, Rxbuf, sizeof(Rxbuf));
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
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Init_all();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(10000);
  t = 0 ;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(Dt);
	  t+=Dt;

//运送------------------------------------------------------------------------------------

	  goal= 3000 ;
	  if((t/950)%12 == 1 ){			//右转90
		  goal_ch[0] = goal_ch[1] = 1920 ;
		  goal_ch[3] = goal_ch[2] = 1920 ;
		  flag = 1 ;
	  }else if((t/950)%12 == 3){		//前走
		  goal_ch[0] = goal_ch[3] = -2000 ;
		  goal_ch[1] = goal_ch[2] = 2000 ;
		  flag = 3 ;
	  }else if((t/950)%12 == 5){  	//右走
		  goal_ch[0] = goal_ch[1] = -2000 ;
		  goal_ch[3] = goal_ch[2] = 2000 ;
		  flag = 5 ;
	  }else if((t/950)%12 == 7){		//左转90
		  goal_ch[0] = goal_ch[1] = -1920 ;
		  goal_ch[3] = goal_ch[2] = -1920 ;
		  flag = 7 ;
	  }else if((t/950)%12 == 9){		//左走
		  goal_ch[0] = goal_ch[1] = 2000 ;
		  goal_ch[3] = goal_ch[2] = -2000 ;
		  flag = 9 ;
	  }else if((t/950)%12 == 11){	//前走
		  goal_ch[0] = goal_ch[3] = -2000 ;
		  goal_ch[1] = goal_ch[2] = 2000 ;
		  flag = 11 ;
	  }else if(((t/950)%2) == 0){
		  goal_ch[0] = goal_ch[1] = 0 ;
		  goal_ch[3] = goal_ch[2] = 0 ;
		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
		  flag = 0 ;
	  }else{
		  ;
	  }

//	  if(t<270){
//		  goal_ch[0] = goal_ch[3] = -2000 ;
//		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  }else if(t<300){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else if(t<470){
//		  goal_ch[0] = goal_ch[1] = -2000 ;
//		  goal_ch[3] = goal_ch[2] = -2000 ;
//	  }else if(t<500){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else if(t<1100){
//		  goal_ch[0] = goal_ch[3] = -2000 ;
//		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  }else if(t<1200){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else{
//		  ;
//	  }

//	  if(t<600){
//		  goal_ch[0] = goal_ch[3] = -2000 ;
//		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  }else if(t<700){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else if(t<1090 ){
//		  goal_ch[0] = goal_ch[1] = -2000 ;
//		  goal_ch[3] = goal_ch[2] = -2000 ;
//	  }else if(t<1200){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else if(t<3200){
//		  goal_ch[0] = goal_ch[3] = -2000 ;
//		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  }else if(t<3300){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else if(t<3050){
//		  goal_ch[0] = goal_ch[3] = 2000 ;
//		  goal_ch[1] = goal_ch[2] = -2000 ;
//	  }else if(t<3500){
//		  goal_ch[0] = goal_ch[1] = 0 ;
//		  goal_ch[3] = goal_ch[2] = 0 ;
//		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  }else{
//		  ;
//	  }

//	  if(t<12000){
//		  pid_chassis();
//	  }


//		if(t<10000){shoot();}
//		if(t<2700){
//			upstair(0);
//		}else if(t<6000){
//			downstair(3000);
//		}else{
//			;
//		}
//	  			upstair(0);
//	  if(t<200){
//		  goal_ch[0] = goal_ch[1] = -2000 ;
//		  goal_ch[3] = goal_ch[2] = 2000 ;
//	  }else{
//		  goal_ch[0] = goal_ch[1] = -2000 ;
//		  goal_ch[3] = goal_ch[2] = 2000 ;
//	  }
//	  if(t<100){
//  		  goal_ch[0] = goal_ch[1] = 2000 ;
//  		  goal_ch[3] = goal_ch[2] = -2000 ;
//	  }else{
//  		  goal_ch[0] = goal_ch[1] = 0 ;
//  		  goal_ch[3] = goal_ch[2] = 0 ;
//	  }
//	  	  if(t<1200){
//	  		  goal_ch[0] = goal_ch[3] = -2000 ;
//	  		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  		  pid_chassis();
//	  	  }else if(t<1100){
//	  		  goal_ch[0] = goal_ch[1] = 0 ;
//	  		  goal_ch[3] = goal_ch[2] = 0 ;
//	  		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  		  pid_chassis();
//	  	  }else if(t<5000 ){
//	  		  upstair2(1300);
//	  	  }else if(t<5700){
//	  		  goal_ch[0] = goal_ch[3] = -2000 ;
//	  		  goal_ch[1] = goal_ch[2] = 2000 ;
//	  		  pid_chassis();
//	  	  }else if(t<5800){
//	  		  goal_ch[0] = goal_ch[1] = 0 ;
//	  		  goal_ch[3] = goal_ch[2] = 0 ;
//	  		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  		  pid_chassis();
//	  	  }else if(t<5850){
//	  		  goal_ch[0] = goal_ch[3] = 2000 ;
//	  		  goal_ch[1] = goal_ch[2] = -2000 ;
//	  		  pid_chassis();
//	  	  }else if(t<6000){
//	  		  goal_ch[0] = goal_ch[1] = 0 ;
//	  		  goal_ch[3] = goal_ch[2] = 0 ;
//	  		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  		  pid_chassis();
//	  	  }else if(t<6900){
//	  		  goal_ch[0] = goal_ch[1] = 2000 ;
//	  		  goal_ch[3] = goal_ch[2] = -2000 ;
//	  		  pid_chassis();
//	  	  }else if(t<7000){
//	  		  goal_ch[0] = goal_ch[1] = 0 ;
//	  		  goal_ch[3] = goal_ch[2] = 0 ;
//	  		  Integral[0] = Integral[1] = Integral[2] = Integral[3] = 0 ;
//	  		  pid_chassis();
//	  	  }else{
//	  		  ;
//	  	  }

//	  shoot();

//		if(t<650){
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
//		}else if(t<33800){
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
////			fishing3(700);
//
//		}else if(t<35000){
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
//		}else {
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
//		}


//2=-1
//	  2112 前
//    2211 右
//	  1111 顺
	  pid_chassis();

	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	  printf("%d",t);
//	  printf("Hello, STM32!\r\n");
//	  printf("time:%d 4:rpm:%d,curt:%d,givn:%d 5:rpm:%d,curt:%d,givn:%d \t 0:rpm:%d,curt:%d,givn:%d 1:rpm:%d,curt:%d,givn:%d 2:rpm:%d,curt:%d,givn:%d 3:rpm:%d,curt:%d,givn:%d\r\n",t,
//	  printf("time:%d  \t 0:rpm:%d,curt:%d,givn:%d \t 1:rpm:%d,curt:%d,givn:%d \t 2:rpm:%d,curt:%d,givn:%d \t 3:rpm:%d,curt:%d,givn:%d\r\n",t,
//			  motor_chassis[4].speed_rpm,motor_chassis[4].given_current,crt_right,
//			  motor_chassis[5].speed_rpm,motor_chassis[5].given_current,crt_left,
//			  motor_chassis[0].speed_rpm,motor_chassis[0].given_current,crt[0],
//			  motor_chassis[1].speed_rpm,motor_chassis[1].given_current,crt[1],
//			  motor_chassis[2].speed_rpm,motor_chassis[2].given_current,crt[2],
//			  motor_chassis[3].speed_rpm,motor_chassis[3].given_current,crt[3]);
//	  printf("t:%d,  \t\t\t\t flag%d\r\n",t,flag );
//	  printf("t:%d   \t %d   %d    %d   %d   %d   %d   `\t\t %d  %d  %d \r\n",
//			  t,crt_right,crt_left,motor_chassis[4].speed_rpm,motor_chassis[4].given_current,motor_chassis[5].speed_rpm,motor_chassis[5].given_current,
//			  flag,Integral_right,Integral_left);
//	  printf("t:%d  \t flag:%d  %d \t %d %d %d %d \t %d %d %d %d \r\n",t,flag,flag2,
//			  crt_right, motor_chassis[4].speed_rpm, motor_chassis[4].given_current, Integral_right,
//			  crt_left , motor_chassis[5].speed_rpm, motor_chassis[5].given_current, Integral_left );
//	  HAL_UART_Transmit_IT(&huart1, TxBuf1, sizeof(TxBuf1));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
