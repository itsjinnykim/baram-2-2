#include "motor_control.h"

PIDSpeedResult PID_SpeedControl(float position_error,float speed,float speed_ref,
    float integral,float dt,const PIDSpeedConfig *config)
{
    return PID_SpeedControlImpl(position_error,speed,speed_ref,integral,dt,config);
}

PIDHoldResult PID_HoldControl(float error,float speed,float dt,float integral)
{
    return PID_HoldControlImpl(error,speed,dt,integral);
}
