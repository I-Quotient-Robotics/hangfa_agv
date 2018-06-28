#ifndef _ROBOTSTRUCT_H_
#define _ROBOTSTRUCT_H_

typedef unsigned char byte;


typedef struct
{
	int motor_current;
	int motor_encoder;
}MotorMsg;

typedef struct
{
	byte input_1;
	byte input_2;
	byte output_1;
	byte output_2;
}RobotIO;

typedef struct
{
	float sonar_1;
	float sonar_2;
	float sonar_3;
	float sonar_4;
}RobotSonar;

typedef struct
{
	float yaw;
	float patch;
	float roll;
}RobotIMU;

typedef struct
{
	float x;
	float y;
	float th;
	float vx;
	float vy;
	float vth;
}RobotOdom;

typedef struct
{
	byte MsgType;//1byte
	byte RobotState;//1byte
	MotorMsg Left_motor;//8byte
	MotorMsg Right_motor;//8byte
	RobotOdom Odom;//20byte
	RobotSonar Sonar;//16byte
	RobotIMU IMU;//12byte
	RobotIO  IO;//4byte
	//70byte
}RobotSubMsg;

typedef struct
{
	byte MsgType;//1byte
	byte RobotState;//1byte
	float Left_Speed;//4byte
	float Right_Speed;//4byte
}RobotPubMsg;

#endif // !_ROBOTSTRUCT_H_
