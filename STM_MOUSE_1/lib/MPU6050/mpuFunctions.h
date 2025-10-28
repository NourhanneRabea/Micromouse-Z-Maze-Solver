#ifndef MPU_FUNCTIONS_H
#define MPU_FUNCTIONS_H

#include "../utils.h"

E_Direction_t CurrentDirection();
void Calculate_Accelration_Velocity_Displacement_Yaw(void);
void initializeMPU(void);
void calibrate_sensor(void) ;


#endif // MPU_FUNCTIONS_H
