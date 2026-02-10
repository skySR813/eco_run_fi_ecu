#include "ecu_math.h"
#include <stdio.h>
#include <math.h>

int rpm_axis[RPM_SIZE] = {1200, 2500, 3500, 4500, 5500, 6500, 7500, 8500};
int tps_axis[TPS_SIZE] = {0, 10, 25, 40, 60, 100};

double map_fuel[RPM_SIZE][TPS_SIZE] = {{16.5,16.0,15.5,14.7,14.2,13.8},
		                               {16.8,16.2,15.0,14.7,14.0,13.6},
									   {17.0,16.3,15.0,14.5,13.8,13.2},
									   {17.0,16.2,14.8,14.2,13.5,13.0},
									   {16.8,16.0,14.5,14.0,13.2,12.8},
									   {16.5,15.8,14.2,13.8,13.0,12.6},
									   {16.2,15.5,14.0,13.6,12.8,12.5},
									   {16.0,15.2,13.8,13.5,12.7,12.4}}; //空燃比表示(デフォルト)

int map_ign[RPM_SIZE][TPS_SIZE] = {
 {8,10,12,14,16,16},
 {12,16,20,22,24,24},
 {14,20,24,26,28,28},
 {16,22,26,28,30,30},
 {16,24,28,30,32,32},
 {14,22,26,28,30,30},
 {12,20,24,26,28,28},
 {10,18,22,24,26,26}
};//進角角度表示(デフォルト)上死点前


//温度管理用（センサーの定数など)
const float BETA = 3435.0;     // サーミスタのB定数 103AT-2-34119
const float R25 = 10000.0;     // 25℃でのサーミスタの抵抗値 (10kΩ)
const float T0 = 298.15;       // 25℃ = 298.15K
const float VREF = 3.3;        // 参照電圧
const float R_FIXED = 10000.0; // 分圧抵抗 (10kΩ)

// t = 0～256（固定小数点）で線形補間
inline int lerp_int(int a, int b, int t)
{
    return a + (((b - a) * t) >> 8);   // >>8 は ÷256
}

int findIndex(int valuea, const int *axis, int sizea)
{
    for (int i = 0; i < sizea - 1; i++) {
        if (valuea >= axis[i] && valuea <= axis[i + 1]) {
            return i;
        }
    }
    return sizea - 2;
}


int getValue_f(int rpm, int tps, double mapp[RPM_SIZE][TPS_SIZE])
{
    // 区間インデックス
    int i = findIndex(rpm, rpm_axis, RPM_SIZE);
    int j = findIndex(tps,  tps_axis, TPS_SIZE);

    // 補間係数を 0～256 に変換（固定小数点）
    int t_rpm = ((rpm - rpm_axis[i]) << 8) / (rpm_axis[i+1] - rpm_axis[i]);
    int t_tps = ((tps - tps_axis[j]) << 8) / (tps_axis[j+1] - tps_axis[j]);

    // まずRPM方向補間
    int v1 = lerp_int(mapp[i][j],     mapp[i+1][j],     t_rpm);
    int v2 = lerp_int(mapp[i][j+1],   mapp[i+1][j+1],   t_rpm);

    // 次にTPS方向補間
    int result = lerp_int(v1, v2, t_tps);

    return result;   // int のまま返す
}

int getValue_i(int rpm, int tps, int mapp[RPM_SIZE][TPS_SIZE])
{
    // 区間インデックス
    int i = findIndex(rpm, rpm_axis, RPM_SIZE);
    int j = findIndex(tps,  tps_axis, TPS_SIZE);

    // 補間係数を 0～256 に変換（固定小数点）
    int t_rpm = ((rpm - rpm_axis[i]) << 8) / (rpm_axis[i+1] - rpm_axis[i]);
    int t_tps = ((tps - tps_axis[j]) << 8) / (tps_axis[j+1] - tps_axis[j]);

    // まずRPM方向補間
    int v1 = lerp_int(mapp[i][j],     mapp[i+1][j],     t_rpm);
    int v2 = lerp_int(mapp[i][j+1],   mapp[i+1][j+1],   t_rpm);

    // 次にTPS方向補間
    int result = lerp_int(v1, v2, t_tps);

    return result;   // int のまま返す
}

//温度計算
float GetTMP(int TMPinput){
  if (TMPinput <= 5) TMPinput = 5;
  if (TMPinput >= 4090) TMPinput = 4090;
  float voltage = (TMPinput / 4095.0) * VREF;//ADC値を電圧に変換
  float resistance = (R_FIXED * voltage) / (VREF - voltage); // サーミスタの抵抗値計算

  // Steinhart-Hart 方程式を用いて温度 (K) を算出
    float temperatureK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(resistance / R25));

    // 絶対温度 (K) から摂氏 (°C) に変換
    return temperatureK - 273.15;
  }
