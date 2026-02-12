#ifndef ECU_MATH_H_
#define ECU_MATH_H_

#define RPM_SIZE 8//マップサイズ変更はここで
#define TPS_SIZE 6

extern int rpm_axis[RPM_SIZE];
extern int tps_axis[TPS_SIZE];
extern int map_ign[RPM_SIZE][TPS_SIZE];
extern double map_fuel[RPM_SIZE][TPS_SIZE];



// t = 0～256（固定小数点）で線形補間
int lerp_int(int a, int b, int t);
int findIndex(int valuea, const int *axis, int sizea);
int getValue_f(int rpm, int tps, double mapp[RPM_SIZE][TPS_SIZE]);
int getValue_i(int rpm, int tps, int mapp[RPM_SIZE][TPS_SIZE]);
float GetTMP(int TMPinput);

#endif


