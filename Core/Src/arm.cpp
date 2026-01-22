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

void Arm::printState(uint32_t angle)
{
	for(int i=0; i < dof; ++i)
	{
		controllers[i]->SetAngle(angle);
	}
}


