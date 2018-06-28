#include "CRobot.h"


CRobot::CRobot(char *port) : m_iSendRate(100)
{
    m_pUart = new CUart(port, 115200);
    m_pSerial = new CRobotSerial(0x01);

    last_time = 0;
    now_time = 0;

    m_bSonar_ = true;

    m_bExit_ = true;
    m_bExit_2 = true;
}
CRobot::~CRobot()
{
    m_pSerial->~CRobotSerial();
    m_pUart->~CUart();
}

void CRobot::run()
{
    int ii = 0;
    int j = 0;
    int _len = 0;

    byte buf[40] = {
        0,
    };
    while (m_bExit_)
    {
       
        _len = m_pUart->m_read(buf, 39);

        for (int i = 0; i < _len; i++)
        {
            if (m_pSerial->m_ReadFrame(buf[i]))
            {
                mtx_1.lock();

                SubMsg.MsgType = m_pSerial->RcvData[0];
                SubMsg.RobotState = m_pSerial->RcvData[1];

                SubMsg.Odom.x = bytesToFloat(m_pSerial->RcvData + 2);
                SubMsg.Odom.y = bytesToFloat(m_pSerial->RcvData + 6);
                SubMsg.Odom.th = bytesToFloat(m_pSerial->RcvData + 10);
                SubMsg.Odom.vx = bytesToFloat(m_pSerial->RcvData + 14);
                SubMsg.Odom.vy = bytesToFloat(m_pSerial->RcvData + 18);
                SubMsg.Odom.vth = bytesToFloat(m_pSerial->RcvData + 22);

                mtx_1.unlock();

                ii++;
            }
        }

        //select(0, NULL,NULL, NULL, &delay);
        usleep(10);
    }
    cout << ii << '\t' << j << endl;
}
bool CRobot::setRobotSpeed(const float v, const float r)
{
    if (mtx_2.try_lock())
    {
        m_flastLSpeed = v;
        m_flastRspeed = r;
        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}

bool CRobot::setRobotState(const RobotState state)
{
    if (mtx_2.try_lock())
    {
        m_bRobotState = byte(state);
        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}
bool CRobot::setMsgType(const byte type)
{
    if (mtx_2.try_lock())
    {
        m_bMsgType = byte(type);
        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}
bool CRobot::setIOState(const int IO_num, const IOState state)
{
    if (mtx_2.try_lock())
    {
        // TODO:

        mtx_2.unlock();
        return true;
    }
    else
    {
        return false;
    }
}

int CRobot::readMotorEncoder(const MotorNum motor)
{
    int re = 0;

    switch (motor)
    {
    case LEFT_MOTOR:
        mtx_1.lock();
        re = SubMsg.Left_motor.motor_encoder;
        mtx_1.unlock();
        break;
    case RIGHT_MOTOR:
        mtx_1.lock();
        re = SubMsg.Right_motor.motor_encoder;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
float CRobot::readMotorCurrent(const MotorNum motor)
{
    float re = 0.0;

    switch (motor)
    {
    case LEFT_MOTOR:
        mtx_1.lock();
        re = SubMsg.Left_motor.motor_current / 100.0;
        mtx_1.unlock();
        break;
    case RIGHT_MOTOR:
        mtx_1.lock();
        re = SubMsg.Right_motor.motor_current / 100.0;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
float CRobot::readSonar(const SonarNum sonar)
{
    float re = 0.0;
    switch (sonar)
    {
    case FRONT_LEFT_SONAR:
        mtx_1.lock();
        re = SubMsg.Sonar.sonar_1;
        mtx_1.unlock();
        break;
    case FRONT_RIGHT_SONAR:
        mtx_1.lock();
        re = SubMsg.Sonar.sonar_2;
        mtx_1.unlock();
        break;
    case REAR_LEFT_SONAR:
        mtx_1.lock();
        re = SubMsg.Sonar.sonar_3;
        mtx_1.unlock();
        break;
    case REAR_RIGHT_SONAR:
        mtx_1.lock();
        re = SubMsg.Sonar.sonar_4;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
float CRobot::readIMU(const IMU imu)
{
    float re = 0.0;
    switch (imu)
    {
    case PITCH:
        mtx_1.lock();
        re = SubMsg.IMU.patch;
        mtx_1.unlock();
        break;
    case ROLL:
        mtx_1.lock();
        re = SubMsg.IMU.roll;
        mtx_1.unlock();
        break;
    case YAW:
        mtx_1.lock();
        re = SubMsg.IMU.yaw;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
float CRobot::readOdom(const Odom odom)
{
    float re = 0.0;
    switch (odom)
    {
    case X:
        mtx_1.lock();
        re = SubMsg.Odom.x;
        mtx_1.unlock();
        break;
    case Y:
        mtx_1.lock();
        re = SubMsg.Odom.y;
        mtx_1.unlock();
        break;
    case TH:
        mtx_1.lock();
        re = SubMsg.Odom.th;
        mtx_1.unlock();
        break;
    case VX:
        mtx_1.lock();
        re = SubMsg.Odom.vx;
        mtx_1.unlock();
        break;
    case VY:
        mtx_1.lock();
        re = SubMsg.Odom.vy;
        mtx_1.unlock();
        break;
    case VTH:
        mtx_1.lock();
        re = SubMsg.Odom.vth;
        mtx_1.unlock();
        break;
    default:
        break;
    }
    return re;
}
IOState CRobot::readIOState(const int IO_num)
{
    // TODO:
    IOState re = LOW;
    switch (IO_num)
    {
    case 0:
        break;
    case 1:
        break;
    default:
        break;
    }
    return re;
}

void CRobot::setIgnorSonar()
{
    mtx_2.lock();
    m_bSonar_ = false;
    mtx_2.unlock();
}
void CRobot::setCheckSonar()
{
    mtx_2.lock();
    m_bSonar_ = true;
    mtx_2.unlock();
}
void CRobot::end()
{
    mtx_1.lock();
    m_bExit_ = false;
    mtx_1.unlock();

    mtx_2.lock();
    m_bExit_2 = false;
    mtx_2.unlock();
}

void CRobot::sendframe()
{
    mtx_2.lock();
    PubMsg[0] = m_bMsgType;
    PubMsg[1] = m_bRobotState;
    floatToByte(m_flastLSpeed, PubMsg + 2);
    floatToByte(m_flastRspeed, PubMsg + 6);
    mtx_2.unlock();
    m_pSerial->m_BuildFrame(PubMsg, 10);
    m_pUart->m_send(m_pSerial->SenData, m_pSerial->SenData_L);
    //last_time = now_time;
}

void CRobot::run_2()
{
    int ii = 0;
    int j = 0;
    while (m_bExit_2)
    {
        //now_time = clock();
        //if ((now_time - last_time) > (CLOCKS_PER_SEC / m_iSendRate))
        //{
            mtx_2.lock();
            PubMsg[0] = m_bMsgType;
            PubMsg[1] = m_bRobotState;
            floatToByte(m_flastLSpeed, PubMsg + 2);
            floatToByte(m_flastRspeed, PubMsg + 6);
            mtx_2.unlock();
            m_pSerial->m_BuildFrame(PubMsg, 10);
            m_pUart->m_send(m_pSerial->SenData, m_pSerial->SenData_L);
            last_time = now_time;
            j++;
       // }
        usleep(1000*100);
    }
    
    cout << ii << '\t' << j << endl;
}