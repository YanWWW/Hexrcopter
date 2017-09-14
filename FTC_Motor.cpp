/******************** (C) COPYRIGHT 2015 FTC ***************************
 * ����		 ��FTC
 * �ļ���  ��FTC_Motor.cpp
 * ����    �����������غ���
**********************************************************************************/
#include "FTC_Motor.h"

FTC_Motor motor;
static int counter1;
void FTC_Motor::writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	if(rc.rawData[AUX2] > 0 && rc.rawData[AUX2] < 1300){
		counter1=0;

	}
	if(counter1 <= 400 && rc.rawData[AUX2] < 1600 && rc.rawData[AUX2] > 1400)
	{	
		 // if(counter==0) throttle = 1100;
			//counter++;
			//if(counter % 100) throttle = throttle + 5; 
		counter1++;
		throttle = 1500;
		
	}
	if(counter1 > 400 &&counter1<=1200&& rc.rawData[AUX2] < 1600 && rc.rawData[AUX2] > 1400)
	{	
		 // if(counter==0) throttle = 1100;
			//counter++;
			//if(counter % 100) throttle = throttle + 5; 
		counter1++;
		throttle = 1500;
		if(counter1<700)
			pidTermRoll += 100;
		else
			pidTermRoll -= 150;
		
	}
	
	if(counter1 > 1200 && counter1 <= 1500  && rc.rawData[AUX2] < 1600 && rc.rawData[AUX2] > 1400)
	{
		counter1++;
		if(counter1>1200&&counter1<=1300)
					throttle = 1200;
		else if(counter1>1300&&counter1<=1500)
					throttle = 1270;
	//	else if(counter1>900&&counter1<=1000)
	//				throttle = 1300;
	
	}
	if(counter1>1600&&counter1<1700&& rc.rawData[AUX2] < 1600 && rc.rawData[AUX2] > 1400)
	{
		throttle = 500;
		counter1++;
	}
	//����X��
	motorPWM[2] = throttle - 0.5 * pidTermRoll + 0.866 *  pidTermPitch + pidTermYaw; //����
	motorPWM[1] = throttle - 0.5 * pidTermRoll - 0.866 *  pidTermPitch + pidTermYaw; //ǰ��
	motorPWM[0] = throttle + 0.5 * pidTermRoll + 0.866 *  pidTermPitch - pidTermYaw; //����
	motorPWM[3] = throttle + 0.5 * pidTermRoll - 0.866 *  pidTermPitch - pidTermYaw; //ǰ��
	motorPWM[5] = throttle - pidTermRoll - pidTermYaw;	//��
	motorPWM[4] = throttle + pidTermRoll + pidTermYaw;	//��

	int16_t maxMotor = motorPWM[0];
	for (u8 i = 1; i < MAXMOTORS; i++)
	{
		if (motorPWM[i] > maxMotor)
					maxMotor = motorPWM[i];				
	}
	
	for (u8 i = 0; i < MAXMOTORS; i++) 
	{
		if (maxMotor > MAXTHROTTLE)    
			motorPWM[i] -= maxMotor - MAXTHROTTLE;	
		//���Ƶ��PWM����С�����ֵ
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
	}

	//���δ�������򽫵���������Ϊ���
	if(!ftc.f.ARMED)	
		ResetPWM();


	if(!ftc.f.ALTHOLD && rc.rawData[THROTTLE] < RC_MINCHECK)
		ResetPWM();

	//д����PWM
	pwm.SetPwm(motorPWM);
	
}
void FTC_Motor::getPWM(int16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
	*(pwm+4) = motorPWM[4];
	*(pwm+5) = motorPWM[5];	
}

void FTC_Motor::ResetPWM(void)
{
	for(u8 i=0; i< MAXMOTORS ; i++)
		motorPWM[i] = 800;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
