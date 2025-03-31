/* -- Includes ------------------------------------------------------------ */
#include "crc.h"

/* -- Defines ------------------------------------------------------------- */

/* -- Types --------------------------------------------------------------- */

/* -- Global Variables ---------------------------------------------------- */

/* -- Module Global Variables --------------------------------------------- */

/* -- Module Global Function Prototypes ----------------------------------- */
//CRC16 MODBUS
//Polynomial: 0x8005
//4-bit lookup table
static const uint16_t crc16_lut[16] =
{
   0x0000u, 0x8005u, 0x800Fu, 0x000Au, 0x801Bu, 0x001Eu, 0x0014u, 0x8011u,
   0x8033u, 0x0036u, 0x003Cu, 0x8039u, 0x0028u, 0x802Du, 0x8027u, 0x0022u
};
//CRC16 MODBUS
//Polynomial: 0x8005
//4-bit lookup table, reflected
static const uint16_t crc16_reflected_lut[16] =
{
   0x0000u, 0xCC01u, 0xD801u, 0x1400u, 0xF001u, 0x3C00u, 0x2800u, 0xE401u,
   0xA001u, 0x6C00u, 0x7800u, 0xB401u, 0x5000u, 0x9C01u, 0x8801u, 0x4400u
};


/* -- Implementation ------------------------------------------------------ */
#define GET_NIBBLE(word, start) (((word) >> (start)) & 0xF)


uint16_t crc_calc_crc16(uint16_t seed, uint8_t const * data, uint32_t length)
{
   uint32_t i, crc = seed;
   for (i = 0; i < length; ++i)
   {
      uint32_t dataByte = data[i];
      uint32_t crcNibble;
      uint32_t dataNibble;
      uint32_t idx;

      //1st nibble
      //calculate index into lookup table
      crcNibble = GET_NIBBLE(crc, 12); //xor the upper 4 bits of the crc ...
      dataNibble = GET_NIBBLE(dataByte, 4); //... with the upper 4 bits for the data-byte.
      idx = crcNibble ^ dataNibble;; //This produces the index into the lookup table
      //update crc according to the respective value from lookup table
      crc = (crc << 4) ^ crc16_lut[idx]; //shift out the 4 upper bits of the crc, and xor the remainder with the value from the lookup table

      //2nd nibble
      crcNibble = GET_NIBBLE(crc, 12); //xor the upper 4 bits of the crc ...
      dataNibble = GET_NIBBLE(dataByte, 0); //... with the lower 4 bits for the data-byte.
      idx = crcNibble ^ dataNibble;; //This produces the index into the lookup table
      //update crc according to the respective value from lookup table
      crc = (crc << 4) ^ crc16_lut[idx]; //shift out the 4 upper bits of the crc, and xor the remainder with the value from the lookup table
   }
   return (uint16_t)crc;
}


uint16_t crc_calc_crc16_reflected(uint16_t seed, uint8_t const * data, uint32_t length)
{
   uint32_t crc = seed;
   for (uint32_t i = 0; i < length; ++i)
   {
      uint32_t dataByte = data[i];
      uint32_t crcNibble;
      uint32_t dataNibble;
      uint32_t idx;

      //1st nibble
      //calculate index into lookup table
      crcNibble = GET_NIBBLE(crc, 0); //xor the lower 4 bits of the crc ...
      dataNibble = GET_NIBBLE(dataByte, 0); //... with the lower 4 bits for the data-byte.
      idx = crcNibble ^ dataNibble;; //This produces the index into the lookup table
      //update crc according to the respective value from lookup table
      crc = (crc >> 4) ^ crc16_reflected_lut[idx]; //shift out the 4 upper bits of the crc, and xor the remainder with the value from the lookup table

      //2nd nibble
      crcNibble = GET_NIBBLE(crc, 0); //xor the lower 4 bits of the crc ...
      dataNibble = GET_NIBBLE(dataByte, 4); //... with the upper 4 bits for the data-byte.
      idx = crcNibble ^ dataNibble;; //This produces the index into the lookup table
      //update crc according to the respective value from lookup table
      crc = (crc >> 4) ^ crc16_reflected_lut[idx]; //shift out the 4 upper bits of the crc, and xor the remainder with the value from the lookup table
   }
   return (uint16_t)crc;
}
