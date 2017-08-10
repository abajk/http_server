/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IN13_Pin GPIO_PIN_13
#define IN13_GPIO_Port GPIOC
#define IN13_EXTI_IRQn EXTI15_10_IRQn
#define IN14_Pin GPIO_PIN_14
#define IN14_GPIO_Port GPIOC
#define IN14_EXTI_IRQn EXTI15_10_IRQn
#define IN15_Pin GPIO_PIN_15
#define IN15_GPIO_Port GPIOC
#define IN15_EXTI_IRQn EXTI15_10_IRQn
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOH
#define IN1_EXTI_IRQn EXTI1_IRQn
#define SPI_CS_Pin GPIO_PIN_1
#define SPI_CS_GPIO_Port GPIOC
#define IN0_Pin GPIO_PIN_0
#define IN0_GPIO_Port GPIOA
#define IN0_EXTI_IRQn EXTI0_IRQn
#define IN4_Pin GPIO_PIN_4
#define IN4_GPIO_Port GPIOA
#define IN4_EXTI_IRQn EXTI4_IRQn
#define OUT1_Pin GPIO_PIN_1
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_13
#define OUT2_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_14
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_15
#define OUT4_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define IN10_Pin GPIO_PIN_10
#define IN10_GPIO_Port GPIOC
#define IN10_EXTI_IRQn EXTI15_10_IRQn
#define IN11_Pin GPIO_PIN_11
#define IN11_GPIO_Port GPIOC
#define IN11_EXTI_IRQn EXTI15_10_IRQn
#define IN12_Pin GPIO_PIN_12
#define IN12_GPIO_Port GPIOC
#define IN12_EXTI_IRQn EXTI15_10_IRQn
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOD
#define IN2_EXTI_IRQn EXTI2_IRQn
#define IN3_Pin GPIO_PIN_3
#define IN3_GPIO_Port GPIOB
#define IN3_EXTI_IRQn EXTI3_IRQn
#define IN7_Pin GPIO_PIN_7
#define IN7_GPIO_Port GPIOB
#define IN7_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
