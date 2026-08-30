/**
 * @file flight_log_config.h
 * @brief Flash region for flight log. App linker FLASH must end below FLIGHT_LOG_FLASH_BASE.
 *
 * H723 bank 1: sectors 2–7 @ 0x08040000, 768 KB (~24575 records @ 32 B).
 * Readback in CubeIDE: start 0x08040000, size 768 KB.
 * Log region must be 0xFF or valid ABIN; boot auto-erases if the header slot is not writable.
 */
#ifndef FLIGHT_LOG_CONFIG_H
#define FLIGHT_LOG_CONFIG_H

#include <stdint.h>

#define FLIGHT_LOG_FLASH_BASE   ((uint32_t)0x08040000U)
#define FLIGHT_LOG_FLASH_SIZE   ((uint32_t)(768U * 1024U))
#define FLIGHT_LOG_FLASH_END    (FLIGHT_LOG_FLASH_BASE + FLIGHT_LOG_FLASH_SIZE)

/* H723: sectors 2–7, 128 KB each. */
#define FLIGHT_LOG_FLASH_BANK          FLASH_BANK_1
#define FLIGHT_LOG_FLASH_SECTOR_FIRST  FLASH_SECTOR_2
#define FLIGHT_LOG_FLASH_SECTOR_COUNT  6U

#endif /* FLIGHT_LOG_CONFIG_H */
