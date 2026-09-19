#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#define RPM_SIZE 8
#define TPS_SIZE 6


#define BASE_ANGLE 40//センサー基準位置

/* Binary map layout shared with the PC tool. */
#define MAP_DATA_SIZE (2U * (RPM_SIZE + TPS_SIZE + 2U * RPM_SIZE * TPS_SIZE))
#define MAP_CRC_SIZE  2U
#define MAP_SIZE      (MAP_DATA_SIZE + MAP_CRC_SIZE)

/* TIM2 runs at 1 MHz.  Reject implausible crank edges before scheduling spark. */
#define CRANK_PERIOD_MIN_US  2000U
#define CRANK_PERIOD_MAX_US  500000U
#define CRANK_TIMEOUT_MS     600U

#endif
