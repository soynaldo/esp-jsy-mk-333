/**
 * @file jsy_crc.h
 * @brief CRC utility functions for checking and setting CRC values.
 */

#ifndef __JSY_CRC_CHECK
#define __JSY_CRC_CHECK

#include <stdbit.h>


/**
 * @brief Verifies if the CRC16 of the given buffer matches the expected value.
 *
 * This function calculates the CRC16 of the buffer excluding the last two bytes
 * and compares it to the CRC value stored in the last two bytes of the buffer.
 *
 * @param buf Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @return true if the CRC matches, false otherwise.
 */
bool jsy_check_crc(const uint8_t *buf, uint16_t len);

/**
 * @brief Appends a CRC16 value to the end of the given buffer.
 *
 * This function calculates the CRC16 of the buffer excluding the last two bytes,
 * and then stores the resulting CRC in the last two bytes of the buffer.
 *
 * @param buf Pointer to the data buffer.
 * @param len Length of the data buffer.
 */
void jsy_set_crc(uint8_t *buf, uint16_t len);

/**
 * @brief Calculates the CRC16 of a given data buffer using a lookup table.
 *
 * The CRC16 is calculated using a precomputed table for performance optimization.
 * The initial CRC value is set to 0xFFFF.
 *
 * @param data Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @return The calculated CRC16 value.
 */
uint16_t jsy_crc_16(const uint8_t *data, uint16_t len);

#endif // __JSY_CRC_CHECK