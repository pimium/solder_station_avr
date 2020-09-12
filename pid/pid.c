//
// Created by pimi on 01.12.18.
//
#include "pid.h"
#include <math.h>

void set_soll_pid(volatile struct SPid *pid, double new_soll)
{
    pid->soll = new_soll;
}

void set_pid(volatile struct SPid *pid, double p_gain, double i_gain,
             double d_gain)
{
    pid->pGain = p_gain;
    pid->iGain = i_gain;
    pid->dGain = d_gain;
}

double update_pid(volatile struct SPid *pid, double position)
{
    double pTerm = 0;
    double iTerm = 0;
    double dTerm = 0;
    double error = 0;
    error = pid->soll - position;
    // calculate the proportional term
    pTerm = pid->pGain * error;
    // calculate the integral state with appropriate limiting
    if (fabs(error) < (pid->iBand)) // only accumulate error while in band
        pid->iState = pid->iState + error;
    else
        pid->iState = 0;

    if (pid->iState > pid->iMax) // limit accumulator to bounds iMax,iMin
        pid->iState = pid->iMax;
    else if (pid->iState < (pid->iMin))
        pid->iState = pid->iMin;

    iTerm = pid->iGain * pid->iState; // calculate the integral term
    dTerm = pid->dGain * (error - pid->dState);
    pid->dState = error;

    return pTerm + iTerm - dTerm;
}