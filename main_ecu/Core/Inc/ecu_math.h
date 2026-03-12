#ifndef ECU_MATH_H_
#define ECU_MATH_H_

#include "ecu_config.h"


// t = 0～256（固定小数点）で線形補間
int lerp_int(int a, int b, int t);
int findIndex(int valuea, const int *axis, int sizea);
float getValue_f(int rpm, int tps, float mapp[RPM_SIZE][TPS_SIZE]);
int getValue_i(int rpm, int tps, int mapp[RPM_SIZE][TPS_SIZE]);
float GetTMP(int TMPinput);

#endif


