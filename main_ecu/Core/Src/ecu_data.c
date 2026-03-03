#include "ecu_data.h"
//#include "ecu_config.h"

/* ===== 実体定義 ===== */

const mapdata default_map = {
    .rpm_axis = {1200, 2500, 3500, 4500, 5500, 6500, 7500, 8500},
    .tps_axis = {0, 10, 25, 40, 60, 100},
    .map_fuel = {
        {16.5,16.0,15.5,14.7,14.2,13.8},
        {16.8,16.2,15.0,14.7,14.0,13.6},
        {17.0,16.3,15.0,14.5,13.8,13.2},
        {17.0,16.2,14.8,14.2,13.5,13.0},
        {16.8,16.0,14.5,14.0,13.2,12.8},
        {16.5,15.8,14.2,13.8,13.0,12.6},
        {16.2,15.5,14.0,13.6,12.8,12.5},
        {16.0,15.2,13.8,13.5,12.7,12.4}
    },
    .map_ign = {
        {8,10,12,14,16,16},
        {12,16,20,22,24,24},
        {14,20,24,26,28,28},
        {16,22,26,28,30,30},
        {16,24,28,30,32,32},
        {14,22,26,28,30,30},
        {12,20,24,26,28,28},
        {10,18,22,24,26,26}
    }
};

mapdata current_map;

/* 各変数 */
volatile int THper = 0;
volatile int rpm_A = 0;
volatile int fdeg = 0;
volatile float AFR_target = 0;
volatile float T_inj_ms = 0;
volatile float T_inj_us = 0;

float inj_inv_ms = 1.0f;
float AFR_base = 14.7f;
float T_base = 6.5f;

volatile float tmp = 0;
float timesecmin = 0.0f;

volatile uint32_t crank_last_us = 0;
volatile uint32_t crank_period_us = 15000;
volatile int32_t delay_us = 0;
int dwell_us = 0;
volatile int crank_flag = 0;
volatile int32_t next_delay_us = 2000;

uint8_t raw_map[MAP_SIZE];
