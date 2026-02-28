#include "ecu_math.h"
#include <stdio.h>
#include <math.h>




//温度管理用（センサーの定数など)
const float BETA = 3435.0;     // サーミスタのB定数 103AT-2-34119
const float R25 = 10000.0;     // 25℃でのサーミスタの抵抗値 (10kΩ)
const float T0 = 298.15;       // 25℃ = 298.15K
const float VREF = 3.3;        // 参照電圧
const float R_FIXED = 10000.0; // 分圧抵抗 (10kΩ)



int lerp_int(int a, int b, int t)
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



float lerp_f(float a, float b, float t)
{
    return a + (b - a) * t;
}


float getValue_f(int rpm, int tps, float mapp[RPM_SIZE][TPS_SIZE])
{
    // 区間インデックス取得
    int i = findIndex(rpm, rpm_axis, RPM_SIZE);
    int j = findIndex(tps,  tps_axis, TPS_SIZE);

    // 補間係数（0.0～1.0）
    float t_rpm = (float)(rpm - rpm_axis[i]) /
                  (float)(rpm_axis[i+1] - rpm_axis[i]);

    float t_tps = (float)(tps - tps_axis[j]) /
                  (float)(tps_axis[j+1] - tps_axis[j]);

    // 安全クランプ
    if (t_rpm < 0.0f) t_rpm = 0.0f;
    if (t_rpm > 1.0f) t_rpm = 1.0f;
    if (t_tps < 0.0f) t_tps = 0.0f;
    if (t_tps > 1.0f) t_tps = 1.0f;

    // RPM方向補間
    float v1 = lerp_f(mapp[i][j],   mapp[i+1][j],   t_rpm);
    float v2 = lerp_f(mapp[i][j+1], mapp[i+1][j+1], t_rpm);

    // TPS方向補間（バイリニア）
    float result = lerp_f(v1, v2, t_tps);

    return result;
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
