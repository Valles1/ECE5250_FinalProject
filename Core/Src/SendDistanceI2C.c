/*
 * SendDistanceI2C.c
 *
 *  Created on: Nov 21, 2025
 *      Author: andrewvalles
 */

#include "main.h"
#include "SendDistanceI2C.h"

/*
void SendDistance_I2C(uint16_t distance_cm)
{
    uint8_t buf[2];
    buf[0] = (distance_cm >> 8) & 0xFF;
    buf[1] = distance_cm & 0xFF;

    HAL_I2C_Master_Transmit(&hi2c2, 0x42 << 1, buf, 2, HAL_MAX_DELAY);
}
*/
