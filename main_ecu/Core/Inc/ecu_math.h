#ifndef ECU_MATH_H_
#define ECU_MATH_H_

#define RPM_SIZE 8//マップサイズ変更はここで
#define TPS_SIZE 6

inline int lerp_int(int a, int b, int t);
int findIndex(int valuea, const int *axis, int sizea);
int getValue_f(int rpm, int tps, double mapp[RPM_SIZE][TPS_SIZE]);
int getValue_i(int rpm, int tps, int mapp[RPM_SIZE][TPS_SIZE]);
float GetTMP(int TMPinput);

#endif


