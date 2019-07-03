/*
 * This file is a part of Dragster car set library.
 *
 * Implement: simple PID-regulator for Dragster 
 * © Amperka LLC (https://amperka.com, dev@amperka.com)
 * 
 * Author: Yury Botov <by@amperka.ru>
 * License: GPLv3, all text here must be included in any redistribution.
 */

#include "PID_dragster.h"

PID::PID(double* result, double proporcional, double integral, double differencial) {
    kp = proporcional;
    ki = integral;
    kd = differencial * 10;
    previouse = 0.0;
    accumulated = 0.0;
    output = result;
    limitMin = -1.1;
    limitMax = 1.1;
}

void PID::setTunings(double proporcional, double integral, double differencial) {
    kp = proporcional;
    ki = integral;
    kd = differencial;
}

void PID::setOutputLimits(double min, double max) {
    limitMin = min;
    limitMax = max;
}

void PID::compute(double deviation) {
    double p = kp * deviation; // proporcional reaction
    double i = ki * accumulated; // one-step integral reaction
    double d = kd * (deviation - previouse); // one-step differencial reaction
    previouse = deviation;
    accumulated = (7 * accumulated + deviation) / 8; // integration
    *output = -clamp(p + i + d, limitMin, limitMax); // reregulation supression
}

double PID::clamp(double source, double min, double max) {
    return (source < min) ? min : ((source > max) ? max : source);
}
