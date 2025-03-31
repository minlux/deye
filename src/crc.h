#ifndef CRC_H
#define CRC_H

/* -- Includes ------------------------------------------------------------ */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* -- Defines ------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------- */

/* -- Function Prototypes ------------------------------------------------- */
// uint16_t crc_calc_crc16(uint16_t seed, uint8_t const * data, uint32_t length);
uint16_t crc_calc_crc16_reflected(uint16_t seed, uint8_t const * data, uint32_t length);

/* -- Implementation ------------------------------------------------------ */

#ifdef __cplusplus
}
#endif

#endif

