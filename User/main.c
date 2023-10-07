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
#include "filter.h"
#include "SortAver_Filter.h"

uint8_t ID;
uint8_t imu_Flag,serial_flag;
int16_t AX, AY, AZ, GX, GY, GZ;
int32_t nowtime,lasttime,sum,peial;
float a12,a22,a31,a32,a33;
float feedbackValue;
int16_t unfiltergyrox;

	BWLowPass *Gyrox_filter;
	BWLowPass *Gyroy_filter;
	BWLowPass *Gyroz_filter;
	BWLowPass *Accx_filter; 
	BWLowPass *Accy_filter; 
	BWLowPass *Accz_filter; 






int main(void)
{
	
	Gyrox_filter=create_bw_low_pass_filter(4,200,20);
	Gyroy_filter=create_bw_low_pass_filter(4,200,20);
	Gyroz_filter=create_bw_low_pass_filter(4,200,20);
	Accx_filter =create_bw_low_pass_filter(2,200,20);
	Accy_filter =create_bw_low_pass_filter(2,200,20);
	Accz_filter =create_bw_low_pass_filter(2,200,20);
	PWM_Init();
	OLED_Init();
	IC_Init();
	MPU9250_Init();
	Serial_Init();
	Timer_Init();
	LED_Init();
	
	PID_Init(&PID_Structure,1.655,0.022,1.792,50,250,100,20);//初始化PID参数
	PID_Init(&PID_Roll_Structure.inner,0,0,0,200,500,100,20);
	PID_Init(&PID_Roll_Structure.outer,0,0,0,20,250,100,45);

	while(1)
	{
		
		



		
//		if(imu_Flag==1)
//		{	


//			imu_Flag=0;
//		}
		if(serial_flag==1)
		{
			

			
//			ANO_DT_Data_Exchange();
      Data_Send_Angle2Rate(Result_Structure.Gyro.X*RadtoDeg,Attitude_Structure.Roll,PID_Roll_Structure.outer.output,PID_Roll_Structure.output,peial,sum,1,1);
			ANO_DT_Send_Senser(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,
							   IMU_Structure.GyroX,unfiltergyrox,IMU_Structure.GyroZ,
							   IMU_Structure.MagX,IMU_Structure.MagY,IMU_Structure.MagZ,
							   0);

			serial_flag=0;
		}

	}


}

void TIM3_IRQHandler(void)
{
	static int16_t serial_time, imu_time;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		imu_time++;
		serial_time++;
		if(imu_time>=5)
		{

			MPU9250_GetData(&IMU_Structure);
			READ_MPU9250_MAG(&IMU_Structure);
			unfiltergyrox=IMU_Structure.GyroX;
			IMU_Structure.GyroX=bw_low_pass(Gyrox_filter,IMU_Structure.GyroX);
			IMU_Structure.GyroY=bw_low_pass(Gyroy_filter,IMU_Structure.GyroY);
			IMU_Structure.GyroZ=bw_low_pass(Gyroz_filter,IMU_Structure.GyroZ);
			
			IMU_Structure.AccX=bw_low_pass(Accx_filter,IMU_Structure.AccX);
			IMU_Structure.AccY=bw_low_pass(Accy_filter,IMU_Structure.AccY);
			IMU_Structure.AccZ=bw_low_pass(Accz_filter,IMU_Structure.AccZ);
						
			
			
			MPU9250_Calibrate();
			Get_Remote_Control();


	
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

			peial=nowtime-lasttime;
			lasttime=nowtime;
			imu_time=0;
		}
		if(serial_time>=10)
		{
			ANO_DT_Send_Status(Attitude_Structure.Roll,Attitude_Structure.Pitch,Attitude_Structure.Yaw,0,0,1);
			Data_Send_AngleRate(PID_Roll_Structure.outer.error,PID_Roll_Structure.outer.P_Out,PID_Roll_Structure.outer.integral,PID_Roll_Structure.outer.D_Out,PID_Roll_Structure.inner.error,PID_Roll_Structure.inner.P_Out,PID_Roll_Structure.inner.integral,PID_Roll_Structure.inner.D_Out);

			serial_flag=1;
			serial_time=0;
		}
		nowtime++;
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

