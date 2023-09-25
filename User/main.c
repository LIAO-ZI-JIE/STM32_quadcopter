#include "stm32f10x.h"                  // Device head
#include <Math.h>
#include "OLED.h"
#include "Delay.h"
#include "IC.h"
#include "Struct.h"
#include "MPU9250.h"
#include "MPU9250_Reg.h"
#include "Serial.h"
#include "PWM.h"
#include "Timer.h"
#include "LED.h"
#include "MahonyAHRS.h"
#include "PID.h"

uint8_t ID;
uint8_t imu_Flag,serial_flag;
int16_t AX, AY, AZ, GX, GY, GZ;
float a12,a22,a31,a32,a33;
float feedbackValue;

int main(void)
{
	PWM_Init();
	OLED_Init();
	IC_Init();
	MPU9250_Init();
	Serial_Init();
	Timer_Init();
	LED_Init();
	PID_Init(&PID_Structure,1.655,0.022,1.792,50,250,100,20);//初始化PID参数
	PID_Init(&PID_Roll_Structure.inner,0,0,0,200,500,100,30);
	PID_Init(&PID_Roll_Structure.outer,0,0,0,20,250,100,45);
	while(1)
	{
		
	

		
		ID=AK8963_GetID();
		
		if(imu_Flag==1)
		{	
			ID=AK8963_GetID();
			MPU9250_GetData_continuous(&IMU_Structure);
			Delay_us(10);
			READ_MPU9250_MAG(&IMU_Structure);
			MPU9250_Calibrate();
			Get_Remote_Control();
			ANO_DT_Send_Senser(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,
							   IMU_Structure.GyroX,IMU_Structure.GyroY,IMU_Structure.GyroZ,
							   IMU_Structure.MagX,IMU_Structure.MagY,IMU_Structure.MagZ,
							   0);
			MahonyAHRSupdate(Result_Structure.Gyro.X,-Result_Structure.Gyro.Y,-Result_Structure.Gyro.Z,-Result_Structure.Acc.X,Result_Structure.Acc.Y,Result_Structure.Acc.Z,Result_Structure.Mag.Y,-Result_Structure.Mag.X,Result_Structure.Mag.Z);
//			MahonyAHRSupdate(Result_Structure.Gyro.X,Result_Structure.Gyro.Y,Result_Structure.Gyro.Z,Result_Structure.Acc.X,Result_Structure.Acc.Y,Result_Structure.Acc.Z,0,0,0);

			a12 =   2.0f * (q1 * q2 + q0 * q3);
			a22 =   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
			a31 =   2.0f * (q0 * q1 + q2 * q3);
			a32 =   2.0f * (q1 * q3 - q0 * q2);
			a33 =   q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
			Attitude_Structure.Pitch = -asinf(a32)*57.29577;
			Attitude_Structure.Roll  = atan2f(a31, a33)*57.29577;
			Attitude_Structure.Yaw   = atan2f(a12, a22)*57.29577;
			
			
			
			
			if(Remote_Control_Structure.THROTTLE>1100)
			{
//				PID_Calc(&PID_Structure,0,Result_Structure.Gyro.X*RadtoDeg);
//				Motor_Structure.Motor1=Remote_Control_Structure.THROTTLE+PID_Structure.output;
//				Motor_Structure.Motor2=Remote_Control_Structure.THROTTLE-PID_Structure.output;
//				Motor_Structure.Motor3=Remote_Control_Structure.THROTTLE-PID_Structure.output;
//				Motor_Structure.Motor4=Remote_Control_Structure.THROTTLE+PID_Structure.output;	
				
				
				PID_CascadeCalc(&PID_Roll_Structure,(float)(Remote_Control_Structure.ROLL-1500)/12,Attitude_Structure.Roll,Result_Structure.Gyro.X*RadtoDeg);
				Motor_Structure.Motor1=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.output;
				Motor_Structure.Motor2=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.output;
				Motor_Structure.Motor3=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.output;
				Motor_Structure.Motor4=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.output;	
			}
			else
			{
				Motor_Structure.Motor1=1100;
				Motor_Structure.Motor2=1100;
				Motor_Structure.Motor3=1100;
				Motor_Structure.Motor4=1100;
				PID_Roll_Structure.inner.integral=0;
				PID_Roll_Structure.outer.integral=0;
			}

			Motor_Output();
			imu_Flag=0;
		}
		if(serial_flag==1)
		{
			ANO_DT_Data_Exchange();

			serial_flag=0;
		}

//		Motor_Output();
//		ANO_DT_Send_Senser(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,
//						   IMU_Structure.GyroX,IMU_Structure.GyroY,IMU_Structure.GyroZ,
//						   IMU_Structure.MagX,IMU_Structure.MagY,IMU_Structure.MagZ,
//						   0);
		
//		ID=MPU9250_GetID();
//		READ_MPU9250_MAG();
//		ID=AK8963_ReadReg(0x00);
//		AY++;
//		MPU9250_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
//		OLED_ShowHexNum(1, 1, ID,2);
				
//		OLED_ShowSignedNum(2, 1, IMU_Structure.GyroX ,5);
//		OLED_ShowSignedNum(3, 1, IMU_Structure.GyroY, 5);
//		OLED_ShowSignedNum(4, 1, IMU_Structure.GyroZ, 5);	
//		OLED_ShowSignedNum(2, 8, IMU_Structure.AccX, 5);
//		OLED_ShowSignedNum(3, 8, IMU_Structure.AccY, 5);
//		OLED_ShowSignedNum(4, 8, IMU_Structure.AccZ, 5);
				
//		OLED_ShowSignedNum(2, 1, Remote_Control_Structure.THROTTLE ,5);
//		OLED_ShowSignedNum(3, 1, Remote_Control_Structure.YAW, 5);
//		OLED_ShowSignedNum(4, 1, Remote_Control_Structure.PITCH, 5);	
//		OLED_ShowSignedNum(2, 8, Remote_Control_Structure.ROLL, 5);
//		OLED_ShowSignedNum(3, 8, IMU_Structure.AccY, 5);
//		OLED_ShowSignedNum(4, 8, IMU_Structure.AccZ, 5);
//		OLED_ShowSignedNum(1, 1, Attitude_Structure.Pitch, 4);
//		OLED_ShowSignedNum(1,6,(int32_t)(Attitude_Structure.Pitch*1000)%1000,4);
//		OLED_ShowSignedNum(2, 1, Attitude_Structure.Roll, 5);
//		OLED_ShowSignedNum(3, 1, Attitude_Structure.Yaw, 5);

//		OLED_ShowSignedNum(2, 8, Calibrate_Structure_Acc.Rx, 5);
//		OLED_ShowSignedNum(3, 8, Calibrate_Structure_Acc.Ry, 5);
//		OLED_ShowSignedNum(4, 8, Calibrate_Structure_Acc.Rz, 5);

	}


}

void TIM3_IRQHandler(void)
{
	static int16_t serial_time, imu_time;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		imu_time++;
		serial_time++;
		if(imu_time>=10)
		{
			imu_Flag=1;

			imu_time=0;
		}
		if(serial_time>=5)
		{

			serial_flag=1;
			serial_time=0;
		}
		
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

