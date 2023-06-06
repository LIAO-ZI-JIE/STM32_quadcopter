#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendNum(uint32_t Number,uint8_t Length);
void Serial_SendString(char *String);
void Serial_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
void Test_Send_User1(int16_t acc_x_,int16_t acc_y_,int16_t acc_z_,int16_t gyro_x_,int16_t gyro_y_,int16_t gyro_z_,int16_t roll_,int16_t pitch_,int16_t yaw_);
void Test_Send_User(uint16_t data1, uint16_t data2, uint16_t data3,uint16_t data4,uint16_t data5,uint16_t data6,uint16_t data7,uint16_t data8,uint16_t data9,uint16_t data10);
void Test_Send_User2(uint16_t throt,uint16_t yaw_2,uint16_t roll_2,uint16_t pitch_2,uint16_t aux_1,uint16_t aux_2,uint16_t aux_3,uint16_t aux_4,uint16_t aux_5,uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4,uint16_t votage);
void Test_Send_User3(int16_t acc_x_3,int16_t acc_y_3,int16_t acc_z_3,int16_t gyro_x_3,int16_t gyro_y_3,int16_t gyro_z_3);
void Test_Send_User4(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d,uint16_t yaw_p,uint16_t yaw_i,uint16_t yaw_d);
#endif
