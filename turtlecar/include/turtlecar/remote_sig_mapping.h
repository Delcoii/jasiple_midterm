/*
 * 리모콘 수신기의 신호를 가공함
 * 
 *  1. 최대, 최소값으로 cut
 *  2. 중립 범위(SIG_N_MIN ~ SIG_N_MAX)에 해당하는 값을 중립(SIG_N)값으로 고정
 *  
 *  input   : 리모콘 펄스 너비(us)
 *  output  : 1000 ~ 1380, 1450, 1520 ~ 1900
 */

#ifndef __REMOTE_SIG_MAPPING_H__
#define __REMOTE_SIG_MAPPING_H__

// for cutting min & max of remote controller signal
#define STEER_SIG_MIN       1000.
#define STEER_SIG_N_MIN     1380.
#define STEER_SIG_N         1450.
#define STEER_SIG_N_MAX     1520.
#define STEER_SIG_MAX       1900.

#define ACCEL_SIG_MIN       1000.
#define ACCEL_SIG_N_MIN     1380.
#define ACCEL_SIG_N         1450.
#define ACCEL_SIG_N_MAX     1520.
#define ACCEL_SIG_MAX       1900.


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


#endif // __REMOTE_SIG_MAPPING_H__