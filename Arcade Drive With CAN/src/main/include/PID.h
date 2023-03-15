// Inclusion guard
#ifndef PID_H
#define PID_H

#include <cmath>
#include <cfloat>
#include <frc/Timer.h>

/**
 * This class manages a PID controller
 * @author Nathan Sandvig
 */
class PID
{
private:
    /**
     * Control constants
     */
    double kp;
    double ki;
    double kd;

    /**
     * Saturation limit for integral
     */
    double sat;

    /**
     * Accumulated error
     */
    double integral;

    /**
     * The error value in the past loop
     */
    double pastError;

    /**
     * The system time in the past loop
     */
    frc::Timer timer;

    /**
     * Feed forward control values
     */
    double ffSlope;
    double ffOffset;

    /**
     * The maximum output from this PID controller
     */
    double max;

    /**
     * The target for the PID controller
     */
    double target;


public:
    /**
     * Default constructor
     */
    PID();

    /**
     * Parameterized constructor
     * @param _kp The proportional control constant
     * @param _ki The integral control constant
     * @param _kd The derivative control constant
     */
    PID(double _kp, double _ki, double _kd);

    /**
     * Copy constructor
     * @param copy The PID controller being copied
     */
    PID(const PID& copy);

    /**
     * Sets the saturation limit for the integral
     * @param _sat The new saturation limit
     */
    void setSatLimit(double _sat);

    /**
     * Sets the feed forward control values
     * @param _ffSlope The slope for the feed forward control
     * @param _ffOffset The offset for the feed forward control
     */
    void setFeedForward(double _ffSlope, double _ffOffset);

    /**
     * Sets the maximum output value for the pid controller
     * @param _max The new maximum value
     */
    void setMax(double _max);

    /**
     * Sets the target of the controller
     * @param _target The new target
     */
    void setTarget(double _target);

    /**
     * Gets the control value
     * @param current The current system value
     * @return The calculated control value
     */
    double getValue(double current);

    /**
     * Resets the inertial and derivative values
     */
    void reset();

    /**
     * Assignment operator overload for PID
     * @param rhs The PID on the right hand side of the operator
     * @return This PID controller with the assigned values
     */
    PID& operator=(const PID& rhs);
};

#endif