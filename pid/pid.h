//
// Created by pimi on 01.12.18.
//

#ifndef SMPS_PID_H
#define SMPS_PID_H

struct SPid
{
    double soll;
    double dState; // Last position input
    double iState; // Integrator state
    // Maximum and minimum allowable integrator state
    double iMax;
    double iMin;
    double iBand; // integral operating band
    double iGain; // integral gain
    double pGain; // proportional gain
    double dGain; // derivative gain
};
double update_pid(volatile struct SPid *pid, double position);
void set_soll_pid(volatile struct SPid *pid, double new_soll);
void set_pid(volatile struct SPid *pid, double p_gain, double i_gain,
             double d_gain);
#endif // SMPS_PID_H
