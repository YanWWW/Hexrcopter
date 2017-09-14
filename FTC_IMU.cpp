/******************** (C) COPYRIGHT 2015 FTC *******************************
 * ����		 ��FTC
 * �ļ���  ��FTC_IMU.cpp
 * ����    ����������̬����
**********************************************************************************/
#include "FTC_IMU.h"

Vector3f gyro1(0,0,0);
Vector3f temp(0,0,0);


FTC_IMU imu; 

FTC_IMU::FTC_IMU()
{
}

//IMU��ʼ��
void FTC_IMU::Init()
{
	//�˲���������ʼ��
	filter_Init();
	//��������ʼ��
	sensor_Init();	
}

//���´���������
void FTC_IMU::updateSensor()
{
	//��ȡ���ٶ�
	mpu6050.Read_Acc_Data();
	//��ȡ���ٶ�
	mpu6050.Read_Gyro_Data();	
	//��ȡ���ٶȣ���λΪ��ÿ��
	Gyro = mpu6050.Get_Gyro();
	//��ȡ���ٶȲ���ֵ
	Acc = mpu6050.Get_Acc();
}


//�����������̬
void FTC_IMU::getAttitude()
{
	float deltaT;
	Vector3d accTemp, gyroTemp;
	
#ifdef FTC_IMU_USE_LPF_1st	
	//���ٶ�����һ�׵�ͨ�˲�
	Acc_lpf = LPF_1st(Acc_lpf, Acc, ftc.factor.acc_lpf);
#endif	
	
#ifdef FTC_IMU_USE_LPF_2nd	
	//���ٶ����ݶ��׵�ͨ�˲�
	Acc_lpf = LPF_2nd(&Acc_lpf_2nd, Acc);
#endif
	
	//���������ݶ��׵�ͨ�˲�
	Gyro_lpf = LPF_2nd(&Gyro_lpf_2nd, Gyro);
	
#ifdef FTC_IMU_USE_LPF_4th
	//���ٶ������Ľ׵�ͨ�˲�
	accTemp(double(Acc.x), double(Acc.y), double(Acc.z));
	accTemp = LPF_Butterworth_4th(accTemp, &Acc_lpf_4th);
	Acc_lpf(float(accTemp.x),float(accTemp.y),float(accTemp.z));
#endif
	

	//����ʵ�ʲ����ļ��ٶȺ��������ٶȵı�ֵ
	accRatio = Acc_lpf.length_squared() * 100 / (ACC_1G * ACC_1G);		
	
	deltaT = getDeltaT(GetSysTime_us());
	
#ifdef FTC_IMU_USE_DCM_CF
	DCM_CF(mpu6050.Get_Gyro_in_dps(Gyro_lpf),Acc_lpf,deltaT);
#endif
#ifdef FTC_IMU_USE_Quaternions_CF
	Quaternion_CF(mpu6050.Get_Gyro_in_dps(Gyro_lpf),Acc_lpf,deltaT);
#endif
}

//��ȡ�������ļ��ٶ��ڵ�������ϵ��ͶӰ
Vector3f FTC_IMU::Get_Accel_Ef(void)
{
	Matrix3f dcm;
	Vector3f anglerad;
	
	//��̬��ת����
	anglerad(-radians(angle.x), -radians(angle.y), radians(angle.z));	//ûд��z�����Ϊ+���ǶԵ�

	//�����ʾ��ת�����Ҿ���
	dcm.from_euler(anglerad);
	
	return dcm * Acc_lpf;
}



//���Ҿ��������̬
void FTC_IMU::DCM_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	//Get_Accel_Ef();
	//getAttitude();//to do
	
	//imu.angle.x=atan2f(acc.x,sqrtf(sq(acc.y)+sq(acc.z)));
	//imu.angle.y=atan2f(acc.y,sqrtf(sq(acc.x)+sq(acc.z)));
	//imu.angle.z=atan2f(acc.z,sqrtf(sq(acc.y)+sq(acc.x)));
	//Vector3f temp1;
 // acc.get_rollpitch(temp1);
	//acc.get_yaw(temp1);
	
	//Vector3f temp2;
	//temp2.x = 0;
//	gyro.get_rollpitch(temp2);
//	gyro.get_yaw(temp2);
	
	temp = (gyro + gyro1) * deltaT * 0.5;
	gyro1=gyro;
	Matrix3f dcm;
	dcm.from_euler(temp);
	static Vector3f gravity(0,0,ACC_1G);
	gravity = dcm*gravity;
	gravity = CF_1st(gravity,acc,0.98);
	gravity.get_rollpitch(angle);
	
	static Vector3f hori(300,0,0);
	hori = dcm * hori;
	hori.get_yaw(angle);
}

#define Kp 2.0f        //���ٶ�Ȩ�أ�Խ��������ٶȲ���ֵ����Խ��
#define Ki 0.001f      //����������
//��Ԫ��������̬
void FTC_IMU::Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	//to do
	Vector3f V_gravity,V_error,V_error_I;
	//�������ٶȹ�һ��
	acc.normalize();
	
	Q.vector_gravity(V_gravity);
	
	V_error=acc%V_gravity;
	
	V_error_I+=V_error*Ki;
	
	gyro+=V_error*Kp+V_error_I;
	
	Q.Runge_Kutta_1st(gyro,deltaT);
	
	Q.normalize();
	
	Q.to_euler(&angle.x,&angle.y,&angle.z);

}

void FTC_IMU::filter_Init()
{
	//���ٶ�һ�׵�ͨ�˲���ϵ������
	ftc.factor.acc_lpf = LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT);
	
	//���ٶȶ��׵�ͨ�˲���ϵ������
	LPF_2nd_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT, &Acc_lpf_2nd);
	
	//�����Ƕ��׵�ͨ�˲���ϵ������	
	LPF_2nd_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_LPF_CUT, &Gyro_lpf_2nd);	
	
	//�����˲���ϵ������
	ftc.factor.gyro_cf = CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);	
}

void FTC_IMU::sensor_Init()
{
	//��ʼ��MPU6050��1Khz�����ʣ�98Hz��ͨ�˲�
	mpu6050.Init(1000,98);
}

float FTC_IMU::getDeltaT(uint32_t currentT)
{
	static uint32_t previousT;
	float	deltaT = (currentT - previousT) * 1e-6;	
	previousT = currentT;
	
	return deltaT;
}

/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/
