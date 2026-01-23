/*
 * arm.cpp
 *
 *  Created on: Jan 21, 2026
 *      Author: chengty
 */
#include "arm.h"

Arm::Arm(char* _ids, int* _redunction, size_t len)
{
	if(len > MAX_NUM_POINTS || len < 0)
	{
		state = constructError;
	};
	if(_ids == nullptr || _redunction == nullptr)
	{
		state = constructError;
	};
	memcpy(ids, _ids, len * sizeof(char));
	memcpy(reduction, _redunction, len * sizeof(int));
	dof = len;
	state = setupSuccess;
};

void Arm::init()
{
	for(int i=0; i < dof; ++i)
	{
		controllers[i] = new Ctrl(hcan1, ids[i], false, reduction[i], -180, 180);
		controllers[i] -> SetEnable(true);
	}
	state = initSuccess;
};

void Arm::updateSetpoints(uint8_t*_setpoint)
{
	memcpy(setpoints, _setpoint, 6 * sizeof(float));
};

float* Arm::getStatePoint()
{
	return setpoints;
};

void Arm::updateJointState(char _ids, float _angle)
{
	for(int i=0; i < dof; ++i)
	{
		if(ids[i] == _ids)
		{
			joint_state[i] = _angle / (float) reduction[i] * 360;
		}
	}
};

void Arm::setAngles()
{
	float angles[6];
	memcpy(angles, setpoints, 6 * sizeof(float));
	for(int i=0; i < dof; ++i)
	{
		controllers[i]->SetAngle(angles[i]);
		HAL_Delay(5);
	}
};

void Arm::sendToHost()
{
	float* _joint_state = joint_state;
	HAL_UART_Transmit(&huart1, (uint8_t*)_joint_state, 6 * sizeof(float), HAL_MAX_DELAY);
};

void Arm::printState()
{
//	float* _joint_state = joint_state;
//	LOG("set joint_state[0..5]: %.3f %.3f\n", joint_state[0], joint_state[1]);
};


