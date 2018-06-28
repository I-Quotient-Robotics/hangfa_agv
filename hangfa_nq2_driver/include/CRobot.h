#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <sys/timeb.h>
#include <time.h>
#include "CRobotSerial.h"
#include "CUart.h"
#include "RobotStruct.h"
#include "function.h"
#include "thread.h"


enum RobotState {STOP, RUN, TURN_LEFT, TURN_RIGHT, ERRO};
enum IOState {LOW, HIGH};
enum IMU {PITCH, ROLL, YAW};
enum MotorNum {LEFT_MOTOR, RIGHT_MOTOR};
enum SonarNum {FRONT_LEFT_SONAR, FRONT_RIGHT_SONAR, REAR_LEFT_SONAR, REAR_RIGHT_SONAR};
enum Odom {X, Y, TH, VX, VY, VTH};


using namespace std;

class CRobot : public Thread
{
  public:
    CRobot(char *port);
    ~CRobot();

    bool setRobotSpeed(const float v, const float r);
    bool setRobotState(const RobotState state);
    bool setMsgType(const byte type);
    bool setIOState(const int IO_num, const IOState state);
    
    int readMotorEncoder(const MotorNum motor);
    float readMotorCurrent(const MotorNum motor);
    float readSonar(const SonarNum sonar);
    float readIMU(const IMU imu);
    float readOdom(const Odom odom);
    IOState readIOState(const int IO_num);

    void setIgnorSonar();
    void setCheckSonar();
    void end();
    void sendframe();

  protected:
    void run();
    void run_2();
    bool checkSonar();
    

    volatile float m_flastLSpeed;
    volatile float m_flastRspeed;
    volatile byte m_bMsgType;
    volatile byte m_bRobotState;
    volatile RobotSubMsg SubMsg;

  private:
    byte PubMsg[10];

    const int m_iSendRate;

    clock_t last_time, now_time;

    std::mutex mtx_1;
    std::mutex mtx_2;

    bool m_bSonar_; 
    bool m_bExit_;
    bool m_bExit_2;
    
    CUart *m_pUart;
    CRobotSerial *m_pSerial;
};