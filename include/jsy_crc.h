#ifndef __JSY_CRC_CHECK
#define __JSY_CRC_CHECK

#include <stdbit.h>

bool jsy_check_crc(const uint8_t *buf, uint16_t len);
void jsy_set_crc(uint8_t *buf, uint16_t len);
uint16_t jsy_crc_16(const uint8_t *data, uint16_t len);

#endif // __JSY_CRC_CHECK