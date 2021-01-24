/*
 * driver for spi ram connected to ospi controller
 * tested on stm32h7a3 with 64mbit esp-psram64h.
 */

/* notes:
 * even though the spi ram does not have a DQS pin, set HAL_OSPI_DQS_ENABLE during write, HAL_OSPI_DQS_DISABLE during read.
 * else hardfault during write.
 * see errata 2.7.8 "Memory-mapped write error response when DQS output is disabled"
 *
 * when memory mapping the spi ram,
 * setting MPU_TEX_LEVEL1, MPU_ACCESS_CACHEABLE, MPU_ACCESS_BUFFERABLE results in occasional data corruption during write.
 *
 * dm00598144-stm32h7a3xig-stm32h7b0xb-and-stm32h7b3xi-device-errata-stmicroelectronics.pdf
 */

#include <stdio.h>

#include "py/mphal.h"
#include "py/mpconfig.h"
#include "py/runtime.h"
#include "mpu.h"
#include "pin.h"
#include "pin_static_af.h"
#include "spiram.h"

#include <stm32h7xx_hal_rcc.h>
#include <stm32h7xx_hal_ospi.h>

extern void __fatal_error(const char *msg);
#define mp_raise_RuntimeError(msg) (mp_raise_msg(&mp_type_RuntimeError, MP_ROM_QSTR(msg)))

// SPI commands, from ESP-PSRAM64H and APS6404L-3SQR-SN datasheet
#define SRAM_CMD_READ           0x03
#define SRAM_CMD_FAST_READ      0x0b
#define SRAM_CMD_QUAD_READ      0xeb
#define SRAM_CMD_WRITE          0x02
#define SRAM_CMD_QUAD_WRITE     0x38
#define SRAM_CMD_QUAD_ON        0x35
#define SRAM_CMD_QUAD_OFF       0xf5
#define SRAM_CMD_RST_EN         0x66
#define SRAM_CMD_RST            0x99
#define SRAM_CMD_BURST_LEN      0xc0
#define SRAM_CMD_READ_ID        0x9f

#ifdef MICROPY_HW_SPIRAM_SIZE_BITS_LOG2

#define OSPI_MAP_ADDR OCTOSPI1_BASE
#define MICROPY_HW_SPIRAM_SIZE (0x800000)

#if defined(MICROPY_HW_SPIRAM_STARTUP_TEST)

// memtest
// spiram_test() tests memory before uart or usb is initialized.
// spiram_dmesg() prints memory test results later, when uart or usb console is initialized.

enum spiram_err_enum {SPIRAM_ERR_OK, SPIRAM_ERR_MEMTEST_PASS, SPIRAM_ERR_MEMTEST8, SPIRAM_ERR_MEMTEST16, SPIRAM_ERR_MEMTEST32, SPIRAM_ERR_OSPI_INIT, SPIRAM_ERR_OSPI_WRITE_CONFIG,SPIRAM_ERR_OSPI_READ_CONFIG,SPIRAM_ERR_OSPI_MMAP,SPIRAM_ERR_READID_CMD,SPIRAM_ERR_READID_DTA, SPIRAM_ERR_QSPI_RST_EN,SPIRAM_ERR_QSPI_RST,SPIRAM_ERR_SPI_RSTEN,SPIRAM_ERR_SPI_RST,SPIRAM_ERR_QUAD_ON, SPIRAM_ERR_CLEAR};
static enum spiram_err_enum spiram_err = SPIRAM_ERR_OK;
static uint8_t spiram_id[8] = {0};
static const uint8_t spiram_pattern8 = 0xA5;
static const uint16_t spiram_pattern16 = 0x5A5A;
static const uint32_t spiram_pattern32 = 0xA5A5A5A5;
static uint32_t spiram_bad_addr = -1;
static uint8_t spiram_bad_pattern8 = -1;
static uint16_t spiram_bad_pattern16 = -1;
static uint32_t spiram_bad_pattern32 = -1;

static inline void spiram_error(enum spiram_err_enum errno) {
    if (spiram_err == SPIRAM_ERR_OK) {
        spiram_err = errno;
    }
}
#endif

OSPI_HandleTypeDef hospi1;

// -----------------------------------------------------------------------------
// Configure MPU. Two options: use HAL, or use micropython primitives.
// If only memory mapping is spiram, there is no difference.
// default: use micropython.

#if MICROPY_HW_SPIRAM_USE_HAL
static inline void ospi_mpu_disable_all(void) {
    HAL_MPU_Disable();
}

static inline void ospi_mpu_enable_mapped(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x90000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

#else

static inline void ospi_mpu_disable_all(void) {
    // Configure MPU to disable access to entire OSPI region, to prevent CPU
    // speculative execution from accessing this region and modifying QSPI registers.
    uint32_t irq_state = mpu_config_start();
    mpu_config_region(MPU_REGION_QSPI1, OSPI_MAP_ADDR, MPU_CONFIG_DISABLE(0x00, MPU_REGION_SIZE_256MB));
    mpu_config_end(irq_state);
}

static inline void ospi_mpu_enable_mapped(void) {
    // Configure MPU to allow access to the valid part of external SPI RAM only.
    // At the moment this is hard-coded to 8 Mbyte of OSPI address space.

    uint32_t irq_state = mpu_config_start();
    mpu_config_region(MPU_REGION_QSPI1, OSPI_MAP_ADDR, MPU_CONFIG_DISABLE(0x00, MPU_REGION_SIZE_256MB));
    mpu_config_region(MPU_REGION_QSPI2, OSPI_MAP_ADDR, MPU_CONFIG_SDRAM(MPU_REGION_SIZE_8MB));
    mpu_config_end(irq_state);
}
#endif

// -----------------------------------------------------------------------------

void ospi_init(void) {
    /* As described in STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ospi.c */

    /* octospi clock enable and reset */
    __HAL_RCC_OSPI1_CLK_ENABLE();
    __HAL_RCC_OSPI1_FORCE_RESET();
    __HAL_RCC_OSPI1_RELEASE_RESET();

    /* octospi pins clock enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* octospi pins configure */
    // Note: checked for PB6, PB2, PD11, PD12, PE2, PD13:
    // OSPI bank 1 on STM32H7A3 is same AF as QSPI on STM32H732, so decided to keep using stm32h743_af.csv and STATIC_AF_QUADSPI_*.

    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_CS, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_BK1_NCS);
    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_SCK, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_CLK);
    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_IO0, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_BK1_IO0);
    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_IO1, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_BK1_IO1);
    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_IO2, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_BK1_IO2);
    mp_hal_pin_config_alt_static_speed(MICROPY_HW_SPIRAM_IO3, MP_HAL_PIN_MODE_ALT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_SPEED_VERY_HIGH, STATIC_AF_QUADSPI_BK1_IO3);

    /* ospi clear */
    hospi1.Instance = OCTOSPI1;
    HAL_OSPI_DeInit(&hospi1);

    /* ospi configure */
    hospi1.Init.FifoThreshold = 1;
    hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY; // sdr qspi
    hospi1.Init.DeviceSize = 23;          // 8 Mbyte, 2**23 MBits
    hospi1.Init.ChipSelectHighTime = 1;
    hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    hospi1.Init.ClockPrescaler = 0x02; // set clock frequency
    hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
    hospi1.Init.ChipSelectBoundary = 10; // 1 kbyte page size
    hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
    hospi1.Init.MaxTran = 0;
    hospi1.Init.Refresh = 0;

    if (HAL_OSPI_Init(&hospi1) != HAL_OK) {
        spiram_error(SPIRAM_ERR_OSPI_INIT);
    }
}

void ospi_mmap() {

    OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};
    OSPI_RegularCmdTypeDef sCommand = {0};

    ospi_mpu_disable_all();

    /* set command to write to spi ram */

    sCommand.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_4_LINES;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE; /* stmh7a3 errata: Memory-mapped write error response when DQS output is disabled */
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_QUAD_WRITE;
    sCommand.Address = 0;
    sCommand.NbData = 0;
    sCommand.DummyCycles = 0;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_OSPI_WRITE_CONFIG);
    }

    /* set command to read from spi ram */

    sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
    sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
    sCommand.Instruction = SRAM_CMD_QUAD_READ;
    sCommand.DummyCycles = 6;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_OSPI_READ_CONFIG);
    }

    /* set up memory mapping */

    /* release nCS after access, else no refresh */
    sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
    sMemMappedCfg.TimeOutPeriod = 1;

    if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK) {
        spiram_error(SPIRAM_ERR_OSPI_MMAP);
    }

    /* set up mpu access */
    ospi_mpu_enable_mapped();
}

// -----------------------------------------------------------------------------

/* spiram read id */

/* read id does not work in qspi mode, only in spi mode. Needs clock <= 84MHz
   sample output "spiram eid 0d 5d 52 a2 64 31 91 31" */

static void spiram_read_id() {

    OSPI_RegularCmdTypeDef sCommand = {0};
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_READ_ID;
    sCommand.Address = 0;
    sCommand.NbData = sizeof(spiram_id);
    sCommand.DummyCycles = 0;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_READID_CMD);
    }

    if (HAL_OSPI_Receive(&hospi1, (uint8_t *)spiram_id, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_READID_DTA);
    }

    return;
}

// -----------------------------------------------------------------------------

/* reset spi ram and switch to qspi mode */

void spiram_quad_on() {
    /* don't know which mode spi ram is in. Might be spi if cold start, qspi if reset/reboot.
     * send spiram reset twice; once in qspi and once in spi mode */

    OSPI_RegularCmdTypeDef sCommand = {0};
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_NONE;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_RST_EN;
    sCommand.Address = 0;
    sCommand.NbData = 0;
    sCommand.DummyCycles = 0;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_QSPI_RST_EN);
    }

    sCommand.Instruction = SRAM_CMD_RST;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_QSPI_RST);
    }

    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = SRAM_CMD_RST_EN;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_SPI_RSTEN);
    }

    sCommand.Instruction = SRAM_CMD_RST;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_SPI_RST);
    }

    /* now in spi mode, can run read_id */

    #if defined(MICROPY_HW_SPIRAM_STARTUP_TEST)
    /* read id */
    spiram_read_id();
    #endif

    /* set qspi mode */
    sCommand.Instruction = SRAM_CMD_QUAD_ON;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        spiram_error(SPIRAM_ERR_QUAD_ON);
    }
}


/* Initialize spi ram to zero. Use after spi ram in qspi mode and before memory mapping. */

static void spiram_clear() {
    // const uint32_t src[256] = {0};
    const uint32_t src[8] = {0xDEADBEEF, 0xDEADBEEF,0xDEADBEEF,0xDEADBEEF,0xDEADBEEF,0xDEADBEEF,0xDEADBEEF,0xDEADBEEF};

    OSPI_RegularCmdTypeDef sCommand = {0};
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_4_LINES;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_QUAD_WRITE;
    sCommand.Address = 0;
    sCommand.NbData = sizeof(src);
    sCommand.DummyCycles = 0;

    for (uint32_t addr = 0; addr < MICROPY_HW_SPIRAM_SIZE; addr += sizeof(src)) {
        sCommand.Address = addr;

        if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            spiram_error(SPIRAM_ERR_CLEAR);
        }

        if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)src, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            spiram_error(SPIRAM_ERR_CLEAR);
        }
    }

}

// -----------------------------------------------------------------------------
// spiram read and write commands. Use in qspi mode, when not memory-mapped.

// like qspi_read_qcmd_qaddr_qdata(NULL, SRAM_CMD_QUAD_READ, addr, len, (void *)dest);

void spiram_read(uint32_t addr, size_t len, uint8_t *dest) {

    OSPI_RegularCmdTypeDef sCommand = {0};
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_4_LINES;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_QUAD_READ;
    sCommand.Address = addr;
    sCommand.NbData = len;
    sCommand.DummyCycles = 6;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        mp_raise_RuntimeError("HAL_OSPI_Command");
    }

    if (HAL_OSPI_Receive(&hospi1, (uint8_t *)dest, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        mp_raise_RuntimeError("HAL_OSPI_Receive");
    }
}

// like qspi_write_qcmd_qaddr_qdata(NULL, SRAM_CMD_QUAD_WRITE, addr, len, (void *)src);

void spiram_write(uint32_t addr, size_t len, const uint8_t *src) {

    OSPI_RegularCmdTypeDef sCommand = {0};
    sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
    sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES;
    sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    sCommand.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = HAL_OSPI_DATA_4_LINES;
    sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.DQSMode = HAL_OSPI_DQS_ENABLE; // See errata
    sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    sCommand.Instruction = SRAM_CMD_QUAD_WRITE;
    sCommand.Address = addr;
    sCommand.NbData = len;
    sCommand.DummyCycles = 0;

    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        mp_raise_RuntimeError("HAL_OSPI_Command");
    }

    if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)src, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        mp_raise_RuntimeError("HAL_OSPI_Transmit");
    }
}


// -----------------------------------------------------------------------------

bool spiram_init(void) {
    ospi_init();
    spiram_quad_on();
    spiram_clear(); // not necessary, but play it safe
    ospi_mmap();
    #if defined(MICROPY_HW_SPIRAM_STARTUP_TEST)
    spiram_test(false);
    #endif
    return true;
}

void *spiram_start(void) {
    return (void *)OSPI_MAP_ADDR;
}

void *spiram_end(void) {
    return (void *)(OSPI_MAP_ADDR + MICROPY_HW_SPIRAM_SIZE);
}

// -----------------------------------------------------------------------------

/* spi ram tests. Write 8, 16, and 32 bit data to ram.
   the patterns used (0xA5) toggle the quad spi lines
   from 1010 to 0101.
 */

static void spiram_memtest8() {
    uint8_t *const mem_base = (uint8_t *)OSPI_MAP_ADDR;
    uint8_t mem_read8;

    /* write pattern to ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE; ++i) {
        mem_base[i] = spiram_pattern8;
    }

    /* assume that, after writing the last address of spi ram,
      the data cache no longer contains the contents of the first address. */

    /* read ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE; ++i) {
        mem_read8 = mem_base[i];
        if (mem_read8 != spiram_pattern8) {
            spiram_error(SPIRAM_ERR_MEMTEST8);
            spiram_bad_addr = OSPI_MAP_ADDR + i;
            spiram_bad_pattern8 = mem_read8;
            return;
        }
    }
}

static void spiram_memtest16() {
    uint16_t *const mem_base = (uint16_t *)OSPI_MAP_ADDR;
    uint16_t mem_read16;

    /* write pattern to ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE / 2; i++) {
        mem_base[i] = spiram_pattern16;
    }

    /* read ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE / 2; i++) {
        mem_read16 = mem_base[i];
        if (mem_read16 != spiram_pattern16) {
            spiram_error(SPIRAM_ERR_MEMTEST16);
            spiram_bad_addr = OSPI_MAP_ADDR + 2 * i;
            spiram_bad_pattern16 = mem_read16;
            return;
        }
    }
}

static void spiram_memtest32() {
    uint32_t *const mem_base = (uint32_t *)OSPI_MAP_ADDR;
    uint32_t mem_read32;

    /* write pattern to ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE / 4; i++) {
        mem_base[i] = spiram_pattern32;
    }

    /* read ram */
    for (uint32_t i = 0; i < MICROPY_HW_SPIRAM_SIZE / 4; i++) {
        mem_read32 = mem_base[i];
        if (mem_read32 != spiram_pattern32) {
            spiram_error(SPIRAM_ERR_MEMTEST32);
            spiram_bad_addr = OSPI_MAP_ADDR + 4 * i;
            spiram_bad_pattern32 = mem_read32;
            return;
        }
    }
}

bool spiram_test(bool fast) {
    spiram_memtest32();
    spiram_memtest16();
    spiram_memtest8();
    spiram_error(SPIRAM_ERR_MEMTEST_PASS);
    return spiram_err == SPIRAM_ERR_MEMTEST_PASS;
}

void spiram_dmesg() {
    mp_printf(MICROPY_ERROR_PRINTER, "spiram eid");
    for (int i = 0; i < sizeof(spiram_id); i++) {
        mp_printf(MICROPY_ERROR_PRINTER, " %02x", spiram_id[i]);
    }
    mp_printf(MICROPY_ERROR_PRINTER, "\n");
    switch (spiram_err) {
        case SPIRAM_ERR_OK:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram ok\n");
            break;
        case  SPIRAM_ERR_MEMTEST_PASS:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram memtest pass\n");
            break;
        case  SPIRAM_ERR_MEMTEST8:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram memtest8 fail, address 0x%08x written 0x%02x read 0x%02x\n", spiram_bad_addr, spiram_pattern8, spiram_bad_pattern8);
            break;
        case  SPIRAM_ERR_MEMTEST16:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram memtest16 fail, address 0x%08x written 0x%04x read 0x%04x\n", spiram_bad_addr, spiram_pattern16, spiram_bad_pattern16);
            break;
        case  SPIRAM_ERR_MEMTEST32:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram memtest32 fail, address 0x%08x written 0x%08x read 0x%08x\n", spiram_bad_addr, spiram_pattern32, spiram_bad_pattern32);
            break;
        case  SPIRAM_ERR_OSPI_INIT:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram ospi init fail\n");
            break;
        case  SPIRAM_ERR_OSPI_WRITE_CONFIG:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram mmap write config fail\n");
            break;
        case SPIRAM_ERR_OSPI_READ_CONFIG:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram mmap read config fail\n");
            break;
        case SPIRAM_ERR_OSPI_MMAP:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram mmap fail\n");
            break;
        case SPIRAM_ERR_READID_CMD:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram readid cmd fail\n");
            break;
        case SPIRAM_ERR_READID_DTA:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram readid dta fail\n");
            break;
        case  SPIRAM_ERR_QSPI_RST_EN:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram qspi rst_en fail\n");
            break;
        case SPIRAM_ERR_QSPI_RST:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram qspi rst fail\n");
            break;
        case SPIRAM_ERR_SPI_RSTEN:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram spi rst_en fail\n");
            break;
        case SPIRAM_ERR_SPI_RST:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram spi rst fail\n");
            break;
        case SPIRAM_ERR_QUAD_ON:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram spi quad on fail\n");
            break;
        case SPIRAM_ERR_CLEAR:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram clear fail\n");
            break;
        default:
            mp_printf(MICROPY_ERROR_PRINTER, "spiram fail, errcode 0x%x\n", spiram_err);
            break;
    }
}

// -----------------------------------------------------------------------------

#endif

// not truncated
