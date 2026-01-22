/*
 * ctrl.cpp
 *
 *  Created on: Jan 21, 2026
 *      Author: chengty
 */
#include "../Inc/ctrl.h"
#include "usart.h"



Ctrl::Ctrl(CAN_HandleTypeDef& _hcan, uint8_t _id, bool _inverse,
           uint8_t _reduction, float _angleLimitMin, float _angleLimitMax):
		   hcan(_hcan)
{
	nodeID = _id;
	inverseDirection=_inverse;
	reduction=_reduction;
	angleLimitMin=_angleLimitMin;
	angleLimitMax=_angleLimitMax;
	txHeader =	{
		.StdId = 0,
		.ExtId = 0,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
	};
}

void Ctrl::SetEnable(bool _enable)
{
	uint8_t mode = 0x01;
	txHeader.StdId = nodeID << 7 | mode;

	// Int to Bytes
	uint32_t val = _enable ? 1 : 0;
	auto* b = (unsigned char*) &val;
	for (int i = 0; i < 4; i++)
		canBuf[i] = *(b + i);
	uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
		// Simple debug: toggle a LED or send via UART
		// For now, just break or log
		LOG("CAN TX ERROR: %d\r\n", status);
	}
}


void Ctrl::SetVelocitySetPoint(float _val)
{
	uint8_t mode = 0x04;
	txHeader.StdId = nodeID << 7 | mode;

	// Float to Bytes
	auto* b = (unsigned char*) &_val;
	for (int i = 0; i < 4; i++)
		canBuf[i] = *(b + i);

	uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
	    // Simple debug: toggle a LED or send via UART
	    // For now, just break or log
	    LOG("CAN TX ERROR: %d\r\n", status);
	}
}

void Ctrl::SetPositionSetPoint(float _val)
{
	uint8_t mode = 0x05;
	txHeader.StdId = nodeID << 7 | mode;

	// Float to Bytes
	auto* b = (unsigned char*) &_val;
	for (int i = 0; i < 4; i++)
		canBuf[i] = *(b + i);
	canBuf[4] = 1; // Need ACK

	uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
		// Simple debug: toggle a LED or send via UART
		// For now, just break or log
		LOG("CAN TX ERROR: %d\r\n", status);
	}
}

void Ctrl::RequestPosition()
{
	uint8_t mode = 0x23;
	txHeader.StdId = nodeID << 7 | mode;
	txHeader.DLC = 0;  // No data

	uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
		// Simple debug: toggle a LED or send via UART
		// For now, just break or log
		LOG("CAN TX ERROR: %d\r\n", status);
	}
}

void Ctrl::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}


void Ctrl::SetCurrentLimit(float _val)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
		// Simple debug: toggle a LED or send via UART
		// For now, just break or log
		LOG("CAN TX ERROR: %d\r\n", status);
	}
}

void Ctrl::SetAcceleration(float _val)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not

    uint32_t mailbox = 0;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, canBuf, &mailbox);
	if (status != HAL_OK) {
		// Simple debug: toggle a LED or send via UART
		// For now, just break or log
		LOG("CAN TX ERROR: %d\r\n", status);
	}
}

void Ctrl::UpdateAngleCallback(float _pos, bool _isFinished)
{

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}


void Ctrl::HAL_CAN_RxFifo0MsgPendingCallback()
{

    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &headerRx, data);
    if (status != HAL_OK)
    {
        LOG("cannot receive message\n");
        return;
    }
    else{
    	LOG("receive message\n");
    	uint8_t id = headerRx.StdId >> 7;
    	uint8_t cmd = headerRx.StdId & 0x7F;
    	 switch (cmd)
		{
			case 0x23:
				UpdateAngleCallback(*(float*) (data), data[4]);
//				LOG("Angle: %.1f\n", angle);
				break;
			default:
				break;
		}
    }

}







