#ifndef __FTC_FLYCONTROL_H
#define __FTC_FLYCONTROL_H

#include "FTC_Config.h"

#define FLYANGLE_MAX 350  //���������35��

enum {
    PIDROLL = 0,
    PIDPITCH = 1,
    PIDYAW = 2,
    PIDANGLE = 3,
    PIDMAG = 4,
    PIDVELZ = 5,
    PIDALT = 6,
		PIDITEMS
};

class FTC_FlyControl
{

public:
	
	FTC_PID pid[7];

	Vector3i setVelocity;
	uint8_t velocityControl;
	int32_t errorVelocityI;
   
  //Vector3f expected_angle;
	//int32_t angle_error[3];

	Vector3i velPIDTerm;

	int32_t AltHold;
	FTC_FlyControl();

	void PID_Reset(void);
	void AltHoldReset(void);

	//��̬�⻷����
	void Attitude_Outter_Loop(void);

	//��̬�ڻ�����
	void Attitude_Inner_Loop(void);

	//�߶��⻷����
	void Altitude_Outter_Loop(void);

	//�߶��ڻ�����
	void Altitude_Inner_Loop(void);

private:
	
	uint8_t rollPitchRate;
	uint8_t yawRate;
	int32_t RateError[3];

	Vector3i velError;
	int16_t altHoldDeadband;

};

extern FTC_FlyControl fc;

#endif























