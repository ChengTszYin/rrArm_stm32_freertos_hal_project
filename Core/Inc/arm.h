/*
 * arm.h
 *
 *  Created on: Jan 21, 2026
 *      Author: chengty
 */

#ifndef INC_ARM_H_
#define INC_ARM_H_
#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include "ctrl.h"
#include "usart.h"
#include "can.h"
#include <string.h>

#define MAX_NUM_POINTS 6

enum RobotState
{
	constructError = 0,
	setupSuccess,
	initError,
	initSuccess
};

class Arm
{
	public:
	enum RobotState state;
	Arm(char* _ids, int* _redunction, size_t len);
	void init();
	void updateSetpoints(uint8_t* _setpoint);
	float* getSetPoint();
	float* getJointState();
	void updateJointState(char ids, float _angle);
	bool setAngles(TickType_t timeoutTicks);
	void getInstantAngle(TickType_t timeoutTicks);
	void sendToHost();
	float printState(char _ids);

	private:
	char ids[MAX_NUM_POINTS] = {};
	int reduction[MAX_NUM_POINTS] = {};
	size_t dof;
	Ctrl* controllers[MAX_NUM_POINTS];
	float setpoints[MAX_NUM_POINTS] = {};
	float joint_state[MAX_NUM_POINTS] = {};
};

#endif /* INC_ARM_H_ */
