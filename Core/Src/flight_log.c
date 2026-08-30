/**
 * @file flight_log.c
 * @brief Binary flight log in internal flash (+ RAM staging).
 */
#include "flight_log.h"
#include "flight_log_config.h"

#include "main.h"

#include <string.h>

#define FLASH_WORD_SIZE 32U
#define FLIGHT_LOG_DATA_BASE  (FLIGHT_LOG_FLASH_BASE + FLIGHT_LOG_FLASH_HEADER_SIZE)
#define FLIGHT_LOG_MAX_RECORDS \
    ((FLIGHT_LOG_FLASH_SIZE - FLIGHT_LOG_FLASH_HEADER_SIZE) / FLIGHT_LOG_RECORD_SIZE)

static uint32_t s_write_addr;
static uint32_t s_record_count;
static uint8_t s_full;
static uint8_t s_staging;

static flight_log_record_t s_ram[FLIGHT_LOG_RAM_CAPACITY];

#if FLIGHT_LOG_DEBUG_LEDS
static void dbg_led_boot_fail(void)
{
    BSP_LED_On(LED_RED);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_YELLOW);
}

static void dbg_led_append_fail(void)
{
    BSP_LED_On(LED_RED);
}
#else
static void dbg_led_boot_fail(void) {}
static void dbg_led_append_fail(void) {}
#endif

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
static uint8_t dcache_is_enabled(void)
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}
#else
static uint8_t dcache_is_enabled(void)
{
    return 0U;
}
#endif

static void invalidate_flash_cache(uint32_t addr, uint32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (!dcache_is_enabled()) {
        return;
    }
    uint32_t aligned = addr & ~31U;
    uint32_t end = addr + len;
    int32_t size = (int32_t)(((end - aligned) + 31U) & ~31U);

    SCB_InvalidateDCache_by_Addr((uint32_t *)(uintptr_t)aligned, size);
#else
    (void)addr;
    (void)len;
#endif
}

static void flash_read(uint32_t addr, void *dst, uint32_t len)
{
    invalidate_flash_cache(addr, len);
    memcpy(dst, (const void *)(uintptr_t)addr, len);
}

static void reset_state(uint8_t staging)
{
    s_write_addr = FLIGHT_LOG_DATA_BASE;
    s_record_count = 0U;
    s_full = 0U;
    s_staging = staging;
}

static void fill_record(
    flight_log_record_t *rec,
    const stub_telemetry_frame_t *frame,
    const flight_state_t *st,
    const control_output_t *cmd)
{
    rec->time_ms = frame->timeMs;
    rec->fc_state = frame->state;
    rec->active = cmd->active;
    rec->servo_angle_cdeg = (uint16_t)(cmd->servo_angle_deg * 100.0f + 0.5f);
    rec->position_z = st->position_z;
    rec->velocity_z = st->velocity_z;
    rec->velocity_lateral = st->velocity_lateral;
    rec->predicted_apogee = cmd->predicted_apogee;
    rec->time_to_apogee = cmd->time_to_apogee;
    rec->control_u = cmd->control_u;
}

static int program_flash_word(uint32_t addr, const uint8_t *src)
{
    uint8_t aligned[FLASH_WORD_SIZE] __attribute__((aligned(32)));
    HAL_StatusTypeDef st;

    memcpy(aligned, src, FLASH_WORD_SIZE);

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (dcache_is_enabled()) {
        SCB_CleanDCache_by_Addr((uint32_t *)aligned, (int32_t)FLASH_WORD_SIZE);
    }
#endif

    __disable_irq();
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)(uintptr_t)aligned);
    __enable_irq();

    if (st != HAL_OK) {
        return -1;
    }
    invalidate_flash_cache(addr, FLASH_WORD_SIZE);
    return 0;
}

static int program_record(uint32_t addr, const flight_log_record_t *rec)
{
    uint8_t aligned[FLASH_WORD_SIZE] __attribute__((aligned(32)));

    memset(aligned, 0xFF, sizeof(aligned));
    memcpy(aligned, rec, sizeof(*rec));
    return program_flash_word(addr, aligned);
}

static int erase_log_region(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef st;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLIGHT_LOG_FLASH_BANK;
    erase.Sector = FLIGHT_LOG_FLASH_SECTOR_FIRST;
    erase.NbSectors = FLIGHT_LOG_FLASH_SECTOR_COUNT;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    __disable_irq();
    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    __enable_irq();
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 0 : -1;
}

static int flash_word_is_erased(uint32_t addr)
{
    uint8_t buf[FLASH_WORD_SIZE];

    flash_read(addr, buf, FLASH_WORD_SIZE);
    for (uint32_t i = 0U; i < FLASH_WORD_SIZE; i++) {
        if (buf[i] != 0xFFU) {
            return 0;
        }
    }
    return 1;
}

static int verify_header_magic(void)
{
    uint32_t magic;

    flash_read(FLIGHT_LOG_FLASH_BASE, &magic, sizeof(magic));
    return (magic == FLIGHT_LOG_BIN_MAGIC) ? 0 : -1;
}

static int write_file_header(uint32_t record_count)
{
    flight_log_file_header_t hdr = {
        .magic = FLIGHT_LOG_BIN_MAGIC,
        .version = FLIGHT_LOG_BIN_VERSION,
        .record_size = FLIGHT_LOG_RECORD_SIZE,
        .record_count = record_count,
    };
    uint8_t aligned[FLASH_WORD_SIZE] __attribute__((aligned(32)));

    memset(aligned, 0xFF, sizeof(aligned));
    memcpy(aligned, &hdr, sizeof(hdr));

    HAL_FLASH_Unlock();
    if (program_flash_word(FLIGHT_LOG_FLASH_BASE, aligned) != 0) {
        HAL_FLASH_Lock();
        return -1;
    }
    HAL_FLASH_Lock();
    return verify_header_magic();
}

static int record_slot_erased(uint32_t addr)
{
    uint8_t buf[FLIGHT_LOG_RECORD_SIZE];

    flash_read(addr, buf, FLIGHT_LOG_RECORD_SIZE);
    for (uint32_t i = 0U; i < FLIGHT_LOG_RECORD_SIZE; i++) {
        if (buf[i] != 0xFFU) {
            return 0;
        }
    }
    return 1;
}

int flight_log_boot(void)
{
    flight_log_file_header_t hdr;

    flash_read(FLIGHT_LOG_FLASH_BASE, &hdr, sizeof(hdr));
    if (hdr.magic == FLIGHT_LOG_BIN_MAGIC && hdr.record_size == FLIGHT_LOG_RECORD_SIZE) {
        uint32_t addr = FLIGHT_LOG_DATA_BASE;
        uint32_t count = 0U;

        while (count < FLIGHT_LOG_MAX_RECORDS &&
               (addr + FLIGHT_LOG_RECORD_SIZE) <= FLIGHT_LOG_FLASH_END &&
               !record_slot_erased(addr)) {
            count++;
            addr += FLIGHT_LOG_RECORD_SIZE;
        }
        s_write_addr = addr;
        s_record_count = count;
        s_full = (count >= FLIGHT_LOG_MAX_RECORDS) ? 1U : 0U;
        s_staging = 0U;
        return 0;
    }

    reset_state(0U);
    if (!flash_word_is_erased(FLIGHT_LOG_FLASH_BASE)) {
        if (erase_log_region() != 0) {
            dbg_led_boot_fail();
            return -1;
        }
    }
    if (write_file_header(0U) != 0) {
        dbg_led_boot_fail();
        return -1;
    }
    return 0;
}

int flight_log_begin_staging(void)
{
    reset_state(1U);
    return 0;
}

int flight_log_append(
    const stub_telemetry_frame_t *frame,
    const flight_state_t *st,
    const control_output_t *cmd)
{
    flight_log_record_t rec;

    if (s_full) {
        return -1;
    }

    if (s_staging) {
        if (s_record_count >= FLIGHT_LOG_RAM_CAPACITY) {
            s_full = 1U;
            return -1;
        }
        fill_record(&s_ram[s_record_count], frame, st, cmd);
        s_record_count++;
        return 0;
    }

    if (s_record_count >= FLIGHT_LOG_MAX_RECORDS) {
        s_full = 1U;
        return -1;
    }

    fill_record(&rec, frame, st, cmd);
    HAL_FLASH_Unlock();
    if (program_record(s_write_addr, &rec) != 0) {
        HAL_FLASH_Lock();
        dbg_led_append_fail();
        return -1;
    }
    HAL_FLASH_Lock();

    s_write_addr += FLIGHT_LOG_RECORD_SIZE;
    s_record_count++;
    return 0;
}

int flight_log_flush(void)
{
    uint32_t i;
    uint32_t addr;

    if (!s_staging) {
        return 0;
    }

    if (erase_log_region() != 0) {
        return -1;
    }
    if (write_file_header(s_record_count) != 0) {
        return -1;
    }

    addr = FLIGHT_LOG_DATA_BASE;
    HAL_FLASH_Unlock();
    for (i = 0U; i < s_record_count; i++) {
        if (program_record(addr, &s_ram[i]) != 0) {
            HAL_FLASH_Lock();
            return -1;
        }
        addr += FLIGHT_LOG_RECORD_SIZE;
    }
    HAL_FLASH_Lock();

    s_write_addr = addr;
    s_staging = 0U;
    return 0;
}

uint8_t flight_log_is_full(void)
{
    return s_full;
}

uint32_t flight_log_record_count(void)
{
    return s_record_count;
}
