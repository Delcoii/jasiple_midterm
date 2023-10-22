#include "TurtleCar/RemoteSigMapping.h"


double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double constrain(double x, double out_min, double out_max) {
    if (x < out_min) {
        return out_min;
    }
    else if (x > out_max) {
        return out_max;
    }
    else {
        return x;
    }
}



void RemoteSigMapping(double& steer_in, double& accel_in) {

    // cut min & max, set neutral value
    steer_in = constrain(steer_in, STEER_SIG_MIN, STEER_SIG_MAX);
    if ((steer_in > STEER_SIG_N_MIN) && (steer_in < STEER_SIG_N_MAX)) {
        steer_in = STEER_SIG_N;
    }
    
    accel_in = constrain(accel_in, ACCEL_SIG_MIN, ACCEL_SIG_MAX);
    if ((accel_in > ACCEL_SIG_N_MIN) && (accel_in < ACCEL_SIG_N_MAX)) {
        accel_in = ACCEL_SIG_N;
    }

}