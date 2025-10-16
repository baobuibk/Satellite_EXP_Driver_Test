/*
 * as3004204.h
 *
 *  Created on: Oct 15, 2025
 *      Author: HTSANG
 */

#ifndef AS3004204_H_
#define AS3004204_H_

#ifndef MRAM_H_
#define MRAM_H_

#include "stdbool.h"
#include "stdint.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"

/*=============================
 * MRAM SPI Command Opcodes
 *=============================*/

#define MRAM_CMD_WREN          0x06    /* Write Enable */
#define MRAM_CMD_WRDI          0x04    /* Write Disable */
#define MRAM_CMD_RDID          0x9F    /* Read Device ID */
#define MRAM_CMD_RDSR          0x05    /* Read Status Register */
#define MRAM_CMD_WRSR          0x01    /* Write Status Register */
#define MRAM_CMD_READ          0x03    /* Read Memory Data */
#define MRAM_CMD_FAST_READ     0x0B    /* Fast Read (requires dummy byte) */
#define MRAM_CMD_WRITE         0x02    /* Write Memory Data */
#define MRAM_CMD_SE            0xD8    /* Sector Erase (if supported) */
#define MRAM_CMD_BE            0xC7    /* Bulk Erase / Chip Erase */
#define MRAM_CMD_SLEEP         0xB9    /* Deep Power-Down */
#define MRAM_CMD_WAKE          0xAB    /* Release from Deep Power-Down */
#define MRAM_CMD_RDCR          0x15    /* Read Configuration Register */
#define MRAM_CMD_WRCR          0x11    /* Write Configuration Register */
#define MRAM_CMD_RDNVCR        0xB5    /* Read Non-Volatile Configuration Register */
#define MRAM_CMD_WRNVCR        0xB1    /* Write Non-Volatile Configuration Register */
#define MRAM_CMD_RDVCR         0x85    /* Read Volatile Configuration Register */
#define MRAM_CMD_WRVCR         0x81    /* Write Volatile Configuration Register */
#define MRAM_CMD_RDEVCR        0x65    /* Read Extended Volatile Configuration Register */
#define MRAM_CMD_WREVCR        0x61    /* Write Extended Volatile Configuration Register */
#define MRAM_CMD_RDNVCFG       0xB5    /* Read Non-Volatile Config */
#define MRAM_CMD_WRNVCFG       0xB1    /* Write Non-Volatile Config */
#define MRAM_CMD_RESET_ENABLE  0x66    /* Enable Reset */
#define MRAM_CMD_RESET_MEMORY  0x99    /* Reset Device */
#define MRAM_CMD_RDUID         0x4B    /* Read Unique ID (optional, some variants only) */
#define MRAM_CMD_RDSFDP        0x5A    /* Read SFDP (Serial Flash Discoverable Parameters) */

/* MRAM status register bits */
#define MRAM_SR_WIP_MASK     0x01   /* Write In Progress bit */
#define MRAM_SR_QE_MASK      0x40   /* Quad Enable bit (nếu có) */

/* MRAM device parameters */
#define MRAM_ADDR_BYTES      3      /* 24-bit address */
#define MRAM_DUMMY_CYCLES    8      /* số dummy cycles cho fast read — chỉnh theo datasheet */

/* --------------------------------------------------------------------------
 *  Device struct
 * -------------------------------------------------------------------------- */
typedef struct {
    SPI_TypeDef *hspi;           /* SPI instance (LL) */
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    bool is_quad;
} mram_dev_t;

/* --------------------------------------------------------------------------
 *  API prototypes
 * -------------------------------------------------------------------------- */

/* Core init/reset */
int8_t mram_init(mram_dev_t *dev);
int8_t mram_reset(mram_dev_t *dev);

/* Mode control */
int8_t mram_enter_qpi(mram_dev_t *dev);
int8_t mram_exit_qpi(mram_dev_t *dev);

/* Read / Write */
int8_t mram_read(mram_dev_t *dev, uint32_t addr, uint8_t *buf, uint32_t len);
int8_t mram_write(mram_dev_t *dev, uint32_t addr, const uint8_t *buf, uint32_t len);

/* Status register */
int8_t mram_read_status(mram_dev_t *dev, uint8_t *sr);
int8_t mram_write_status(mram_dev_t *dev, uint8_t sr);

#endif /* AS3004204_H_ */
