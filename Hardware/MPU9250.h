#ifndef __MPU9250_H
#define __MPU9250_H

#define RadtoDeg    57.324841f	

void MPU9250_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU9250_ReadReg(uint8_t RegAddress);

void MPU9250_Init(void);
uint8_t MPU9250_GetID(void);
uint8_t AK8963_GetID(void);
void MPU9250_GetData(IMU_Struct *IMU_Structure);
void MPU9250_GetData_continuous(IMU_Struct *IMU_Structure);
void i2c_Mag_write(u8 reg,u8 value);
void AK8963_WriteReg(uint8_t RegAddress, uint8_t Data);
void READ_MPU9250_MAG(IMU_Struct *IMU_Structure);
uint8_t AK8963_ReadReg(uint8_t RegAddress);
void MPU9250_Calibrate(void);
extern uint8_t MPU9250_Acc_Gryo_Calibrate_flag;
extern uint8_t MPU9250_Mag_Calibrate_flag;

#endif
