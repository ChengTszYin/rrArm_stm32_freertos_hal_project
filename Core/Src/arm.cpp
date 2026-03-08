/*
 * arm.cpp
 *
 *  Created on: Jan 21, 2026
 *      Author: chengty
 */
#include "arm.h"

SemaphoreHandle_t canCommandSemaph = NULL;

Arm::Arm(char* _ids, int* _redunction, bool* inverse_, size_t len)
{
	if(len > MAX_NUM_POINTS || len < 0)
	{
		state = constructError;
	};
	if(_ids == nullptr || _redunction == nullptr || inverse_ == nullptr)
	{
		state = constructError;
	};
	memcpy(ids, _ids, len * sizeof(char));
	memcpy(reduction, _redunction, len * sizeof(int));
	memcpy(inverse, inverse_, len * sizeof(int));
	dof = len;
	state = setupSuccess;
};

void Arm::init()
{
	BaseType_t xReturn = pdPASS;
	canCommandSemaph = xSemaphoreCreateBinary();
	if(canCommandSemaph == NULL)
	{
		LOG("Failed to create Semaphore\n");
	}
	xSemaphoreGive(canCommandSemaph);
	for(int i=0; i < dof; ++i)
	{
		controllers[i] = new Ctrl(hcan, ids[i], inverse[i], reduction[i], -180, 180);
		controllers[i] -> SetEnable(true);
//		controllers[i]->SetVelocitySetPoint(0.0f);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	state = initSuccess;
};

//void Arm::updateSetpoints(uint8_t*_setpoint)
//{
//	memcpy(setpoints, _setpoint, 6 * sizeof(float));
//};

void Arm::updateSetpoints(uint8_t*_setpoint)
{
	memcpy(setpoints, _setpoint, 6 * sizeof(float));
};

void Arm::updateSetVelocity(uint8_t*_setpoint)
{
	memcpy(setvelocity, _setpoint, 6 * sizeof(float));
};

float* Arm::getSetPoint()
{
	return setpoints;
};

float* Arm::getSetVelocity()
{
	return setvelocity;
};

float* Arm::getJointState()
{
	return joint_state;
};

void Arm::updateJointState(char _ids, float _angle)
{
	for(int i=0; i < dof; ++i)
	{
		if(ids[i] == _ids)
		{
			joint_state[i] = _angle * (360.0f / (float)reduction[i]);
		}
	}
};

bool Arm::setAngles(TickType_t timeoutTicks)
{
	if(xSemaphoreTake(canCommandSemaph, timeoutTicks) == pdTRUE)
	{
		float angles[6];
		memcpy(angles, setpoints, 6 * sizeof(float));
		for(int i=0; i < dof; ++i)
		{
			float _setVelocity = setvelocity[i];
			float _setAngle = angles[i];
//			controllers[i] -> SetVelocitySetPoint(_setVelocity);
			controllers[i] -> SetAngle(_setAngle);
			vTaskDelay(pdMS_TO_TICKS(5));
		}
		xSemaphoreGive(canCommandSemaph);
		return 1;
	}
	else
	{
		LOG("setAngles timeout - semaphore busy\n");
		return 0;
	}

};

void Arm::getInstantAngle(TickType_t timeoutTicks)
{
	if(xSemaphoreTake(canCommandSemaph, timeoutTicks) == pdTRUE)
	{
		for(int i=0; i<dof; ++i)
		{
			controllers[i]->RequestPosition();
			vTaskDelay(pdMS_TO_TICKS(5));
		}
		xSemaphoreGive(canCommandSemaph);
	}
	else
	{
		LOG("getInstantAngle timeout - semaphore busy\n");
	}

};


void Arm::finishSegment()
{
	uint8_t ack = 'D';
	HAL_UART_Transmit(&huart1, &ack, sizeof(uint8_t), HAL_MAX_DELAY);
};

void Arm::sendToHost()
{
	float* _joint_state = joint_state;
//	for(int i=0; i < dof; ++i)
//	{
//		_joint_state[i] = _joint_state[i] * 3.14159 / 180.0;
//	}
	HAL_UART_Transmit(&huart1, (uint8_t*)_joint_state, 6 * sizeof(float), HAL_MAX_DELAY);
};

float Arm::printState(char _ids)
{
	for(int i=0; i < dof; ++i)
	{
		if(ids[i] == _ids)
		{
			return joint_state[i];
		}
	}
};


