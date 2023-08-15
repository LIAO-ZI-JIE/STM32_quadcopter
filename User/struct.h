#ifndef _STRUCTURE_H
#define _STRUCTURE_H
typedef struct
{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t Temp;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
}IMU_Struct;

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}int16_Struct;


typedef struct
{
	float Roll;
	float Pitch;
	float Yaw;
}Attitude_Struct;

typedef struct PID
{
  float P;         //参数
  float I;
  float D;
  float Error;     //比例项
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;   
  uint8_t Ilimit_flag;    //积分分离	
}PID_TYPE; 

typedef struct
{
	float Ox;
	float Oy;
	float Oz;
	float Rx;
	float Ry;
	float Rz;
}Calibrate_Struct;

typedef struct
{
	float X;
	float Y;
	float Z;
}XYZ_Struct;

typedef struct
{	
	int16_t Acc_Offset_X;
	int16_t Acc_Offset_Y;
	int16_t Acc_Offset_Z;
	int16_t Gyro_Offset_X;
	int16_t Gyro_Offset_Y;
	int16_t Gyro_Offset_Z;

}Offset_Struct;

typedef struct
{
	XYZ_Struct Acc;
	XYZ_Struct Gyro;
	XYZ_Struct Mag;
}Result_Struct;

typedef struct
{	
	int16_t THROTTLE;
	int16_t YAW;
	int16_t ROLL;
	int16_t PITCH;
}Remote_Control_Struct;

typedef struct
{	
	uint16_t Motor1;
	uint16_t Motor2;
	uint16_t Motor3;
	uint16_t Motor4;
}Motor_Struct;
typedef struct
{
   	float kp,ki,kd;//三个系数
    float error,lastError;//误差、上次误差
    float integral,maxIntegral;//积分、积分限幅
    float P_Out,I_Out,D_Out,output,maxOutput;//输出、输出限幅
}PID_Struct;
//串级PID的结构体，包含两个单级PID
typedef struct
{
    PID_Struct inner;//内环
    PID_Struct outer;//外环
    float output;//串级输出，等于inner.output
}CascadePID_Struct;


extern IMU_Struct IMU_Structure;
extern Attitude_Struct Attitude_Structure;
extern Remote_Control_Struct Remote_Control_Structure;
extern Motor_Struct Motor_Structure;
extern Result_Struct Result_Structure;
extern PID_Struct PID_Structure;
extern Offset_Struct Offset_Structure;
extern CascadePID_Struct PID_Roll_Structure;
#endif


