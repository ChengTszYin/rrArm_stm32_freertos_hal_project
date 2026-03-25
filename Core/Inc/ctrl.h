/*
 * ctrl.h
 *
 *  Created on: Jan 21, 2026
 *      Author: chengty
 */

#ifndef INC_CTRL_H_
#define INC_CTRL_H_

#include "can.h"
#include <stdbool.h>
#include <stdint.h>
#include <cstring>

class Ctrl {
public:
	Ctrl(CAN_HandleTypeDef& _hcan, uint8_t _id, bool _inverse = false,
			uint8_t _reduction = 1, float _angleLimitMin = -180.0f,
			float _angleLimitMax = 180.0f);
	void SetEnable(bool _enable);
	void SetVelocitySetPoint(float _val);
	void SetPositionSetPoint(float _val);
	void SetCurrentLimit(float _val);
	void SetAcceleration(float _val);
	void SetAngle(float _angle);
	void UpdateAngleCallback(float _pos, bool _isFinished);
	void OnCanReceiveMsg(CAN_RxHeaderTypeDef *_rxHeader, uint8_t *_data);
	void HAL_CAN_RxFifo0MsgPendingCallback();
	void RequestPosition();
	~Ctrl();

	uint8_t nodeID;
	float angle = 0.0f;
	float angleLimitMax;
	float angleLimitMin;
	bool inverseDirection;
	uint8_t reduction;
private:
	CAN_HandleTypeDef& hcan;
	uint8_t canBuf[8] = {};
	uint8_t data[8];
	CAN_TxHeaderTypeDef txHeader = {};
	CAN_RxHeaderTypeDef headerRx;
};



#endif /* INC_CTRL_H_ */
