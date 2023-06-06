#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "IC.h"
#include "Struct.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "Serial.h"
#include "PWM.h"


uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;
int main(void)
{
	PWM_Init();
	OLED_Init();
	IC_Init();
	MPU6050_Init();
	Serial_Init();
	while(1)
	{

		IMU_Struct IMU_structure;
		Test_Send_User(IMU_structure.AccX,IMU_structure.AccY,IMU_structure.AccZ,1,1,1,1,1,1,1);
		Test_Send_User1(IMU_structure.AccX,IMU_structure.AccY,IMU_structure.AccZ,IMU_structure.GyroX,IMU_structure.GyroY,IMU_structure.GyroZ,2000,30,1);
		TIM_SetCompare1(TIM2,(TIM_GetCapture2(TIM1)+1));
		TIM_SetCompare2(TIM2,(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1));
		TIM_SetCompare3(TIM2,(TIM_GetCapture2(TIM4)+1));
		TIM_SetCompare4(TIM2,(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1));
		MPU6050_ReadReg_continuous(MPU6050_ACCEL_XOUT_H,&IMU_structure);

//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		OLED_ShowSignedNum(2, 1, IMU_structure.AccX, 5);
		OLED_ShowSignedNum(3, 1, IMU_structure.AccY, 5);
		OLED_ShowSignedNum(4, 1, IMU_structure.AccZ, 5);
		OLED_ShowSignedNum(2, 8, IMU_structure.GyroX, 5);
		OLED_ShowSignedNum(3, 8, IMU_structure.GyroY, 5);
		OLED_ShowSignedNum(4, 8, IMU_structure.GyroZ, 5);
		
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
