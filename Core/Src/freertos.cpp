/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "arm.h"
#include "usart.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RECEIVE_BYTES_LENGTH 51
#define STEP 2
#define RECEIVE_ANGLE_LENGTH 24
#define RECEIVE_VELOCITY_LENGTH 24

xTaskHandle Ctrl_Task_Handler;
xTaskHandle Receive_Task_Handler;
void Ctrl_Task(void *argument);
void Receive_Task(void *argument);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char ids[MAX_NUM_POINTS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
int reduction[MAX_NUM_POINTS] = {50, 51, 51, 51, 51, 51};
int inverse_[MAX_NUM_POINTS] = {-1,-1, -1, -1, +1, +1};
float joint_offset[MAX_NUM_POINTS] = {-18.0, 0.0, 0.0, 0.0, 0.0, 0.0};
size_t len = 6;
Arm robot(ids, reduction, inverse_, joint_offset, 6);

uint8_t receiveBuffer[RECEIVE_BYTES_LENGTH];
uint8_t receiveAngleBuffer[RECEIVE_ANGLE_LENGTH];
uint8_t receiveFloatBuffer[RECEIVE_VELOCITY_LENGTH];
volatile bool setTraject = false;
static CAN_RxHeaderTypeDef rxHeader;
static uint8_t rxData[8];

uint8_t floatbuff[4];
float receivefloat;

const float degree_to_radian =  3.1425 / 180.0 ;
const float radian_to_degree = 180.0 / 3.1425;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//	xTaskCreate(Ctrl_Task, "Ctrl_Task", 512, NULL, 2, &Ctrl_Task_Handler);
	xTaskCreate(Receive_Task, "Receive_Task", 512, NULL, 2, &Receive_Task_Handler);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  robot.init();
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
extern void MX_USART1_UART_Init(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		uint8_t crc = robot.checksum(receiveBuffer, RECEIVE_BYTES_LENGTH - 1);
		if(crc == receiveBuffer[50])
		{
			short step = (int16_t)receiveBuffer[1] << 8 | receiveBuffer[0];
			memcpy(receiveAngleBuffer, receiveBuffer + STEP, 24 * sizeof(uint8_t));
			memcpy(receiveFloatBuffer, (receiveBuffer + STEP + RECEIVE_ANGLE_LENGTH), 24 * sizeof(uint8_t));
			robot.updateSetpoints(receiveAngleBuffer);
			robot.updateSetVelocity(receiveFloatBuffer);
			float* angle = robot.getSetPoint();
			float* velocity = robot.getSetVelocity();
			for(int i = 0; i < 6; ++i)
			{
				angle[i] *= degree_to_radian;
			}
			LOG("receive id: %d\n", step);
//			LOG("receive angle[0..5]: %.3f %.3f  %.3f  %.3f  %.3f  %.3f\n", angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);
//			LOG("receive velocity[0..5]: %.3f %.3f  %.3f  %.3f  %.3f  %.3f\n", velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
			if(step == 0)
			{
				setTraject = false;
			}
			else
			{
				setTraject = true;
			}
		}
		HAL_UART_Receive_DMA(&huart1, receiveBuffer, sizeof(receiveBuffer));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uint32_t error = HAL_UART_GetError(huart);

        // Log the error (limited to avoid flooding)
        static uint32_t err_count = 0;
        if (err_count < 10)
        {
            LOG("UART Error on F1: 0x%08lX (FE=%d, NE=%d, ORE=%d, PE=%d)\r\n",
                error,
                !!(error & HAL_UART_ERROR_FE),
                !!(error & HAL_UART_ERROR_NE),
                !!(error & HAL_UART_ERROR_ORE),
                !!(error & HAL_UART_ERROR_PE));
            err_count++;
        }

        // === Aggressive recovery specifically for STM32F1 ===
        HAL_UART_DMAStop(huart);     // Stop DMA first

        // Clear all error flags
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // Dummy read to clear status register
        volatile uint32_t tmp = huart->Instance->SR;
        tmp = huart->Instance->DR;

        huart->ErrorCode = HAL_UART_ERROR_NONE;

        // Re-initialize the entire UART peripheral (this fixes most stubborn F1 cases)
        MX_USART1_UART_Init();

        // Restart DMA reception
        HAL_UART_Receive_DMA(huart, receiveBuffer, sizeof(receiveBuffer));

        // Reset trajectory flag
        setTraject = false;
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
	{
		uint8_t id = rxHeader.StdId >> 7;
		uint8_t cmd = rxHeader.StdId & 0x7F;
		switch (cmd)
		{
			case 0x23:
			{
				float pos = *(float*)rxData;
				robot.updateJointState(id, pos);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void Ctrl_Task(void *argument)
{
	if(!setTraject)
	{
		robot.getInstantAngle(pdMS_TO_TICKS(100));
		robot.sendToHost();
	}
	vTaskDelay(pdMS_TO_TICKS(100));
}

void Receive_Task(void *argument)
{
	HAL_UART_Receive_DMA(&huart1, receiveBuffer, sizeof(receiveBuffer));
	while(1)
	{
		robot.getInstantAngle(pdMS_TO_TICKS(30));
		robot.sendToHost();
//		float* jointstate = robot.getJointState();
//		LOG("joint state[0..5]: %.3f %.3f  %.3f  %.3f  %.3f  %.3f\n", jointstate[0], jointstate[1], jointstate[2], jointstate[3], jointstate[4], jointstate[5]);
		if(setTraject)
		{
			bool finished = robot.setAngles(pdMS_TO_TICKS(30));
			if(finished)
			{
				setTraject = false;
				LOG("Segment finished successfully\n");
			}
		}
		vTaskDelay(pdMS_TO_TICKS(30));
//		HAL_UART_Receive_DMA(&huart1, receiveBuffer, sizeof(receiveBuffer));
	}
}
/* USER CODE END Application */

