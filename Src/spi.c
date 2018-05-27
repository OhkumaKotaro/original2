/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/********************************************************************
 * Overview : spi read register
 * Argument : register
 * Return : 1byte data
********************************************************************/
uint8_t read_byte(uint8_t reg){
  uint8_t ret,val;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  ret = reg|SETTING ;
  //HAL_SPI_TransmitReceive(&hspi2,&ret,&val,1,400); //not work
  HAL_SPI_Transmit(&hspi2,&ret,1,100);
  HAL_SPI_Receive(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val;
}


/****************************************************************
 * Overview : shift 8bit and spi read register
 * Argument : register
 * Return : 2byte data (shift 8bit)
 ************************************************************/
int16_t read_shift_byte(uint8_t reg){
  uint8_t address,val_1;
  int16_t val_2;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  address = reg | SETTING ;
  //HAL_SPI_TransmitReceive(&hspi2,&address,&val_1,1,100); //not work
  HAL_SPI_Transmit(&hspi2,&address,1,100);
  HAL_SPI_Receive(&hspi2,&val_1,1,100);
  val_2 = (int16_t)(val_1 << 8);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val_2;
}


/**************************************************************
 * Overview : spi write 1byte
 * Argument : register
 * Return : 
 *************************************************************/
void write_byte( uint8_t reg,uint8_t val){
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2,&reg,1,100);
  HAL_SPI_Transmit(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
}


/****************************************************************
 * Overview : set up l3gd20
 * argument : 
 * return :
 ****************************************************************/
void set_l3gd20(void){
  uint8_t val;
  val = read_byte(WHO_AM_I);
  printf("\nYou are 0x%x\r\n",val );
  if(val != Certain){
    control_LED(1,OFF);
  }
  write_byte(CTRL_REG1,POWER_ON);
  write_byte(CTRL_REG4,L3GD20_2000dps);
}


/*****************************************************************
 * Overview : read l3gd20
 * argument :
 * Return : int data (2000 deg/sec)
 ****************************************************************/
int get_gyro(void){
  int val;
  val = (read_shift_byte(OUT_Z_H) | read_byte(OUT_Z_L)) * 0.070;
  return val;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
