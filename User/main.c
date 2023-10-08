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


#define PID_1D


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
CascadePID_Struct PID_Roll_Structure;
CascadePID_Struct PID_Pitch_Structure;
CascadePID_Struct PID_Yaw_Structure;



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
	
//	PID_Init(&PID_Structure,1.655,0.022,1.792,50,250,100,20);//初始化PID参数
	
	
//	PID_Init_inner(&PID_Roll_Structure.inner,0.585,0.018,4.5,200,500,100,20);
//	PID_Init_outer(&PID_Roll_Structure.outer,3.5,0,0,20,250);
	PID_Init_inner(&PID_Roll_Structure.inner,0.57,0.018,5.3,200,500,100,20);
	PID_Init_outer(&PID_Roll_Structure.outer,3.5,0,0,20,250);


	PID_Init_inner(&PID_Pitch_Structure.inner,0.57,0.018,5.8,200,500,100,20);
	PID_Init_outer(&PID_Pitch_Structure.outer,0,0,0,20,250);
	
	PID_Init_inner(&PID_Yaw_Structure.inner,0,0,0,200,300,100,20);
	PID_Init_outer(&PID_Yaw_Structure.outer,0,0,0,20,250);

	while(1)
	{
//		
//		 if(f.send_pid1)
//	{
//		f.send_pid1 = 0;
////		ANO_DT_Send_PID(1,PID_ROL_Rate.P,PID_ROL_Rate.I,PID_ROL_Rate.D,
////		                  PID_PIT_Rate.P,PID_PIT_Rate.I,PID_PIT_Rate.D,
////		                  PID_YAW_Rate.P,PID_YAW_Rate.I,PID_YAW_Rate.D);
//		ANO_DT_Send_PID(1,PID_Roll_Structure.inner.kp,PID_Roll_Structure.inner.ki,PID_Roll_Structure.inner.kd,
//		                  PID_Pitch_Structure.inner.kp,PID_Pitch_Structure.inner.ki,PID_Pitch_Structure.inner.kd,
//		                  PID_Yaw_Structure.inner.kp,PID_Yaw_Structure.inner.ki,PID_Yaw_Structure.inner.kd);

//	}	

//			if(f.send_pid2)
//				{
//					f.send_pid2 = 0;
//			//		ANO_DT_Send_PID(2,PID_ROL_Angle.P,PID_ROL_Angle.I,PID_ROL_Angle.D,
//			//						  PID_PIT_Angle.P,PID_PIT_Angle.I,PID_PIT_Angle.D,
//			//		                  PID_YAW_Angle.P,PID_YAW_Angle.I,PID_YAW_Angle.D);
//					ANO_DT_Send_PID(2,PID_Roll_Structure.outer.kp,PID_Roll_Structure.outer.ki,PID_Roll_Structure.outer.kd,
//														PID_Pitch_Structure.outer.kp,PID_Pitch_Structure.outer.ki,PID_Pitch_Structure.outer.kd,
//														PID_Yaw_Structure.outer.kp,PID_Yaw_Structure.outer.ki,PID_Yaw_Structure.outer.kd);
//				}
//		
//		if(imu_Flag==1)
//		{	


//			imu_Flag=0;
//		}
//		if(serial_flag==1)
//		{
//			

//			
////			ANO_DT_Data_Exchange();
//      Data_Send_Angle2Rate(Result_Structure.Gyro.X*RadtoDeg,Attitude_Structure.Roll,PID_Roll_Structure.outer.output,PID_Roll_Structure.output,peial,sum,1,1);

//			ANO_DT_Send_Status(Attitude_Structure.Roll,Attitude_Structure.Pitch,Attitude_Structure.Yaw,0,0,1);
//			Data_Send_AngleRate(PID_Roll_Structure.outer.error,PID_Roll_Structure.outer.P_Out,PID_Roll_Structure.outer.integral,PID_Roll_Structure.outer.D_Out,PID_Roll_Structure.inner.error,PID_Roll_Structure.inner.P_Out,PID_Roll_Structure.inner.integral,PID_Roll_Structure.inner.D_Out);
//			ANO_DT_Send_Status(Attitude_Structure.Roll,Attitude_Structure.Pitch,Attitude_Structure.Yaw,0,0,1);

//			serial_flag=0;
//		}

	}


}

void TIM3_IRQHandler(void)
{
	static int16_t serial_time;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		serial_time++;


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
				
				#ifdef PID_2D
				PID_CascadeCalc(&PID_Roll_Structure,(float)(Remote_Control_Structure.ROLL-1500)/12,Attitude_Structure.Roll,Result_Structure.Gyro.X*RadtoDeg);
				PID_CascadeCalc(&PID_Pitch_Structure,(float)(Remote_Control_Structure.ROLL-1500)/12,Attitude_Structure.Pitch,Result_Structure.Gyro.Y*RadtoDeg);
				

				#endif
				#ifdef PID_1D
				PID_Calc_filter(&PID_Roll_Structure.inner,(float)(Remote_Control_Structure.ROLL-1500)/12,Result_Structure.Gyro.X*RadtoDeg);
				PID_Calc_filter(&PID_Pitch_Structure.inner,(float)-(Remote_Control_Structure.PITCH-1500)/12,Result_Structure.Gyro.Y*RadtoDeg);
				PID_Calc_filter(&PID_Yaw_Structure.inner,(float)(Remote_Control_Structure.YAW-1500)/12,Result_Structure.Gyro.Z*RadtoDeg);
				#endif

//				Motor_Structure.Motor1=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.inner.output;
//				Motor_Structure.Motor2=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.inner.output;
//				Motor_Structure.Motor3=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.inner.output;
//				Motor_Structure.Motor4=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.inner.output;	


				Motor_Structure.Motor1=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.inner.output-PID_Pitch_Structure.inner.output;
				Motor_Structure.Motor2=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.inner.output-PID_Pitch_Structure.inner.output;
				Motor_Structure.Motor3=Remote_Control_Structure.THROTTLE-PID_Roll_Structure.inner.output+PID_Pitch_Structure.inner.output;
				Motor_Structure.Motor4=Remote_Control_Structure.THROTTLE+PID_Roll_Structure.inner.output+PID_Pitch_Structure.inner.output;	
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
			
		if(serial_time>=10)
		{
//			ANO_DT_Send_Status(Attitude_Structure.Roll,Attitude_Structure.Pitch,Attitude_Structure.Yaw,0,0,1);
//			Data_Send_AngleRate(PID_Roll_Structure.outer.error,PID_Roll_Structure.outer.P_Out,PID_Roll_Structure.outer.integral,PID_Roll_Structure.outer.D_Out,PID_Roll_Structure.inner.error,PID_Roll_Structure.inner.P_Out,PID_Roll_Structure.inner.integral,PID_Roll_Structure.inner.D_Out);
			ANO_DT_Send_Senser(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,
							   IMU_Structure.GyroX,IMU_Structure.GyroY,IMU_Structure.GyroZ,
							   IMU_Structure.MagX,IMU_Structure.MagY,IMU_Structure.MagZ,
							   0);
			ANO_DT_Send_Status(Attitude_Structure.Roll,Attitude_Structure.Pitch,Attitude_Structure.Yaw,0,0,1);

//			serial_flag=1;
			serial_time=0;
		}
		nowtime++;
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

