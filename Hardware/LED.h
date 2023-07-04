#ifndef _LED_H
#define _LED_H
void LED_Init(void);
#define LED(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_0, (BitAction)(x))

#endif
