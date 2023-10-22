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


double map(double x, double in_min, double in_max, double out_min, double out_max);
double constrain(double x, double out_min, double out_max);
void RemoteSigMapping(double& steer_in, double& accel_in);


#endif // __REMOTE_SIG_MAPPING_H__