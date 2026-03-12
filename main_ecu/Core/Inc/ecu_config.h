#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#define RPM_SIZE 8
#define TPS_SIZE 6


#define BASE_ANGLE 40//センサー基準位置

#define MAP_SIZE 2*(RPM_SIZE + TPS_SIZE + 2*RPM_SIZE*TPS_SIZE)

#endif
