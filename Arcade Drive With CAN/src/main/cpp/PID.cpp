// Included headers
#include "PID.h"

template<class type>
int sign(type n)
{
    return n < 0 ? -1 : 1;
}

PID::PID()
{
    kp = 0.0;
    ki = 0.0;
    kd = 0.0;
    sat = 0.0;
    integral = 0.0;
    pastError = 0.0;
    timer.Start();
    ffSlope = 0.0;
    ffOffset = 0.0;
    max = 0.0;
    target = 0.0;
}

PID::PID(double _kp, double _ki, double _kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
    sat = DBL_MAX;
    integral = 0.0;
    pastError = 0.0;
    timer.Start();
    ffSlope = 0.0;
    ffOffset = 0.0;
    max = DBL_MAX;
    target = 0.0;
}

PID::PID(const PID& copy)
{
    kp = copy.kp;
    ki = copy.ki;
    kd = copy.kd;
    sat = copy.sat;
    integral = copy.integral;
    pastError = copy.pastError;
    timer.Start();
    ffSlope = copy.ffSlope;
    ffOffset = copy.ffOffset;
    max = copy.max;
    target = copy.target;
}

void PID::setSatLimit(double _sat)
{
    sat = _sat;

}

void PID::setFeedForward(double _ffSlope, double _ffOffset)
{
    ffSlope = _ffSlope;
    ffOffset = _ffOffset;
}

void PID::setMax(double _max)
{
    max = _max;
}

void PID::setTarget(double _target)
{
    target = _target;
}

double PID::getValue(double current)
{
    double error = target - current;
    int loopTime = timer.Get().value(); //pros::millis();
    timer.Reset();

    double pValue = kp * error;

    double iValue = 0.0;
    if (std::abs(ki) > 0.0001)
    {
        integral += error * loopTime;
        if (std::abs(integral) > std::abs(sat / ki))
            integral = std::abs(sat / ki) * sign<double>(integral);
        iValue = ki * integral;
    }

    double dValue = 0.0;
    if (loopTime != 0)
        dValue = kd * (error - pastError) / loopTime;

    double ffValue = (ffSlope * target) + (ffOffset * sign<double>(target));

    pastError = error;

    double result = pValue + iValue + dValue + ffValue;

    return std::min(std::abs(result), max) * sign<double>(result);
}

void PID::reset()
{
    integral = 0.0;
    pastError = 0.0;
    timer.Reset();
}

PID& PID::operator=(const PID& rhs)
{
    kp = rhs.kp;
    ki = rhs.ki;
    kd = rhs.kd;
    sat = rhs.sat;
    integral = rhs.integral;
    pastError = rhs.pastError;
    timer = rhs.timer;
    ffSlope = rhs.ffSlope;
    ffOffset = rhs.ffOffset;
    max = rhs.max;
    target = rhs.target;

    return *this;
}