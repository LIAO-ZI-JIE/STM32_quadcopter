#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "IC.h"
#include "MPU6050.h"
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

		TIM_SetCompare1(TIM2,(TIM_GetCapture2(TIM1)+1));
		TIM_SetCompare2(TIM2,(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1));
		TIM_SetCompare3(TIM2,(TIM_GetCapture2(TIM4)+1));
		TIM_SetCompare4(TIM2,(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1));
//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//		OLED_ShowSignedNum(2, 1, AX, 5);
//		OLED_ShowSignedNum(3, 1, AY, 5);
//		OLED_ShowSignedNum(4, 1, AZ, 5);
//		OLED_ShowSignedNum(2, 8, GX, 5);
//		OLED_ShowSignedNum(3, 8, GY, 5);
//		OLED_ShowSignedNum(4, 8, GZ, 5);
		
		OLED_ShowString(1,1,"Time:");
		OLED_ShowNum(1,6,(TIM_GetCapture2(TIM1)+1),5);
		OLED_ShowString(2,1,"Time:");
		OLED_ShowNum(2,6,(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1),5);
		OLED_ShowString(3,1,"Time:");
		OLED_ShowNum(3,6,(TIM_GetCapture2(TIM4)+1),5);
		OLED_ShowString(4,1,"Time:");
		OLED_ShowNum(4,6,(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1),5);
	}


}
