#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "IC.h"

int main(void)
{
	OLED_Init();
	IC_Init();
	while(1)
	{
		OLED_ShowString(1,1,"Time:");
		OLED_ShowNum(1,6,(TIM_GetCapture2(TIM1)+1)-1097,5);
		OLED_ShowString(2,1,"Time:");
		OLED_ShowNum(2,6,(TIM_GetCapture4(TIM1)+1)-(TIM_GetCapture3(TIM1)+1),5);
		OLED_ShowString(3,1,"Time:");
		OLED_ShowNum(3,6,(TIM_GetCapture3(TIM1)+1),5);
		OLED_ShowString(4,1,"Time:");
		OLED_ShowNum(4,6,(TIM_GetCapture4(TIM1)+1),5);
	}


}
