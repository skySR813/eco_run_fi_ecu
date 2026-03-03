#ifndef ECU_DATA_H_
#define ECU_DATA_H_

#include <stdint.h>
#include "ecu_config.h"

typedef struct{
    uint16_t rpm_axis_ee[RPM_SIZE];
    uint16_t tps_axis_ee[TPS_SIZE];
    float map_fuel_raw_ee[RPM_SIZE][TPS_SIZE];
    float fuel_map_ee[RPM_SIZE][TPS_SIZE];
    uint16_t map_ign_ee[RPM_SIZE][TPS_SIZE];

    int rpm_axis[RPM_SIZE];
    int tps_axis[TPS_SIZE];
    float map_fuel[RPM_SIZE][TPS_SIZE];
    int map_ign[RPM_SIZE][TPS_SIZE];
} mapdata;

/* ===== extern宣言だけ書く ===== */
extern const mapdata default_map;
extern mapdata current_map;

extern volatile int THper;
extern volatile int rpm_A;
extern volatile int fdeg;
extern volatile float AFR_target;
extern volatile float T_inj_ms;
extern volatile float T_inj_us;

extern float inj_inv_ms;
extern float AFR_base;
extern float T_base;

extern volatile float tmp;
extern float timesecmin;

extern volatile uint32_t crank_last_us;
extern volatile uint32_t crank_period_us;
extern volatile int32_t delay_us;
extern int dwell_us;
extern volatile int crank_flag;
extern volatile int32_t next_delay_us;

extern uint8_t raw_map[MAP_SIZE];

#endif
