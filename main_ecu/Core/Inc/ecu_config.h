#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#define RPM_SIZE 8
#define TPS_SIZE 6


#define MAP_SIZE ((RPM_SIZE*2) + (TPS_SIZE*2) + (RPM_SIZE*TPS_SIZE*2) +(RPM_SIZE*TPS_SIZE*2) + 2 )

#endif
