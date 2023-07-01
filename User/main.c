#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "IC.h"
#include "Struct.h"
#include "MPU9250.h"
#include "MPU9250_Reg.h"
#include "Serial.h"
#include "PWM.h"
#include "Timer.h"


uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
IMU_Struct IMU_Structure;
Attitude_Struct Attitude_Structure;

int main(void)
{
	PWM_Init();
	OLED_Init();
	IC_Init();
	MPU9250_Init();
	Serial_Init();
	Timer_Init();
	while(1)
	{
		

		
		Test_Send_User(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,1,1,1,1,1,1,1);
		Test_Send_User1(IMU_Structure.AccX,IMU_Structure.AccY,IMU_Structure.AccZ,IMU_Structure.GyroX,IMU_Structure.GyroY,IMU_Structure.GyroZ,2000,30,1);
		TIM_SetCompare1(TIM2,(TIM_GetCapture2(TIM1)+1));
		TIM_SetCompare2(TIM2,(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1));
		TIM_SetCompare3(TIM2,(TIM_GetCapture2(TIM4)+1));
		TIM_SetCompare4(TIM2,(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1));
		MPU9250_GetData_continuous(&IMU_Structure);
		ID=AK8963_GetID();
		AY++;
//		AY=AK8963_GetID();
//		MPU9250_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		OLED_ShowHexNum(1, 1, ID,2);
		OLED_ShowHexNum(1, 8, AY,5);		
		OLED_ShowSignedNum(2, 1, IMU_Structure.AccX, 5);
		OLED_ShowSignedNum(3, 1, IMU_Structure.AccY, 5);
		OLED_ShowSignedNum(4, 1, IMU_Structure.AccZ, 5);
		OLED_ShowSignedNum(2, 8, IMU_Structure.GyroX, 5);
		OLED_ShowSignedNum(3, 8, IMU_Structure.GyroY, 5);
		OLED_ShowSignedNum(4, 8, IMU_Structure.GyroZ, 5);
		
//		OLED_ShowString(1,1,"Time:");
//		OLED_ShowNum(1,6,(TIM_GetCapture2(TIM1)+1),5);
//		OLED_ShowString(2,1,"Time:");
//		OLED_ShowNum(2,6,(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1),5);
//		OLED_ShowString(3,1,"Time:");
//		OLED_ShowNum(3,6,(TIM_GetCapture2(TIM4)+1),5);
//		OLED_ShowString(4,1,"Time:");
//		OLED_ShowNum(4,6,(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1),5);
	}


}

void TIM3_IRQHandler(void)
{
	static int16_t serial_time, imu_time;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		imu_time++;
		serial_time++;
		if(imu_time>=20)
		{
			MPU9250_GetData_continuous(&IMU_Structure);
			MPU9250_Calibrate();
			imu_time=0;
		}
		if(serial_time>=50)
		{
		
			serial_time=0;
		}
		
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

