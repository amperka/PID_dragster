/*
 * This file is a part of Dragster car set library.
 *
 * Defines: simple PID-regulator for Dragster 
 * © Amperka LLC (https://amperka.com, dev@amperka.com)
 * 
 * Author: Yury Botov <by@amperka.com>
 * License: GPLv3, all text here must be included in any redistribution.
 */

#ifndef __PID_DRAGSTER_H__
#define __PID_DRAGSTER_H__

class PID {

public:
    PID(double* result, double proporcional, double integral, double differencial);

    void SetTunings(double proporcional, double integral, double differencial);
    void SetOutputLimits(double min, double max);

    void compute(double deviation);

private:
    double clamp(double source, double min, double max);

    double kp; // Proportional parameter
    double ki; // Integral parameter
    double kd; // Differencial parameter

    double previouse; // previouse step deviation value
    double accumulated; // accumulated deviation

    double* output;

    double limitMin, limitMax; // clamp limits
};

#endif //__PID_DRAGSTER_H__
