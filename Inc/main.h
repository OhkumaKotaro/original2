/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define led5_Pin GPIO_PIN_13
#define led5_GPIO_Port GPIOC
#define led4_Pin GPIO_PIN_14
#define led4_GPIO_Port GPIOC
#define led6_Pin GPIO_PIN_15
#define led6_GPIO_Port GPIOC
#define gyro_cs_Pin GPIO_PIN_12
#define gyro_cs_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_8
#define led3_GPIO_Port GPIOC
#define led2_Pin GPIO_PIN_9
#define led2_GPIO_Port GPIOC
#define push1_Pin GPIO_PIN_11
#define push1_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_12
#define led1_GPIO_Port GPIOA
#define led0_Pin GPIO_PIN_13
#define led0_GPIO_Port GPIOA
#define push3_Pin GPIO_PIN_15
#define push3_GPIO_Port GPIOA
#define push2_Pin GPIO_PIN_10
#define push2_GPIO_Port GPIOC
#define stby_Pin GPIO_PIN_11
#define stby_GPIO_Port GPIOC
#define bin2_Pin GPIO_PIN_12
#define bin2_GPIO_Port GPIOC
#define bin1_Pin GPIO_PIN_2
#define bin1_GPIO_Port GPIOD
#define ain2_Pin GPIO_PIN_3
#define ain2_GPIO_Port GPIOB
#define ain1_Pin GPIO_PIN_4
#define ain1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

void Control_motor(int r_accel,int l_accel);
void battery_check(void);

// encoder
typedef struct {
  unsigned int pattern;
  unsigned int rp_pattern; 
  unsigned int l;
  unsigned int r;
  unsigned int goal;
  unsigned int l_now;
  unsigned int r_now;
  unsigned int center_now;
  unsigned int center_total;
  unsigned int cr;
  unsigned int rp;
}ENC;

extern ENC enc;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
