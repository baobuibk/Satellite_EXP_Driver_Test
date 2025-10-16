/*
 * as3004204.c
 *
 *  Created on: Oct 15, 2025
 *      Author: HTSANG
 */

#include "as3004204.h"
#include <string.h>

/* CS control */
static inline void mram_cs_low(mram_dev_t *dev) {
    LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin);
}
static inline void mram_cs_high(mram_dev_t *dev) {
    LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);
}

/* SPI byte transmit + receive (8-bit) */
static uint8_t mram_spi_txrx(SPI_TypeDef *hspi, uint8_t data) {
    while (!LL_SPI_IsActiveFlag_TXE(hspi));
    LL_SPI_TransmitData8(hspi, data);
    while (!LL_SPI_IsActiveFlag_RXNE(hspi));
    return LL_SPI_ReceiveData8(hspi);
}

/* Block transfer */
static void mram_spi_transfer(mram_dev_t *dev, uint8_t *buf, uint32_t len) {
    mram_cs_low(dev);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = mram_spi_txrx(dev->hspi, buf[i]);
    }
    mram_cs_high(dev);
}

/* Write enable for status register or config (nếu cần) — nếu datasheet hỗ trợ */
static int8_t mram_write_enable(mram_dev_t *dev) {
    uint8_t cmd = 0x06;  /* Write Enable opcode — bạn kiểm tra datasheet xem có lệnh này không */
    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, cmd);
    mram_cs_high(dev);
    return 0;
}

/* Poll WIP bit until 0 */
static int8_t mram_wait_wip_clear(mram_dev_t *dev, uint32_t timeout_ms) {
    uint8_t sr = 0;
    uint32_t tick0 = 0;
    while (1) {
        if (mram_read_status(dev, &sr) < 0) {
            return -1;
        }
        if ((sr & MRAM_SR_WIP_MASK) == 0) {
            return 0;
        }
        LL_mDelay(1);
        tick0++;
        if (tick0 > timeout_ms) {
            return -2;
        }
    }
}

/* --------------------------------------------------------------------------
 *  init / reset
 * -------------------------------------------------------------------------- */
int8_t mram_init(mram_dev_t *dev) {
    mram_cs_high(dev);
    dev->is_quad = false;

    /* reset device */
    if (mram_reset(dev) < 0) {
        return -1;
    }

    return 0;
}

int8_t mram_reset(mram_dev_t *dev) {
    uint8_t buf[1];
    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, MRAM_CMD_RSTEN);
    mram_cs_high(dev);

    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, MRAM_CMD_RESET);
    mram_cs_high(dev);

    /* Chờ WIP = 0 */
    if (mram_wait_wip_clear(dev, 10) < 0) {
        return -2;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 *  Mode control: QPI / Exit QPI
 * -------------------------------------------------------------------------- */
int8_t mram_enter_qpi(mram_dev_t *dev) {
    /* Gửi lệnh Enter QPI */
    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, MRAM_CMD_ENTER_QPI);
    mram_cs_high(dev);

    dev->is_quad = true;
    return 0;
}

int8_t mram_exit_qpi(mram_dev_t *dev) {
    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, MRAM_CMD_EXIT_QPI);
    mram_cs_high(dev);

    dev->is_quad = false;
    return 0;
}

/* --------------------------------------------------------------------------
 *  Read / Write
 * -------------------------------------------------------------------------- */
int8_t mram_read(mram_dev_t *dev, uint32_t addr, uint8_t *buf, uint32_t len) {
    if (len == 0) return 0;

    uint8_t header[1 + MRAM_ADDR_BYTES + 1]; /* opcode + addr + maybe dummy */
    uint32_t hdr_len = 0;

    /* Use fast read (với dummy) nếu tốc độ cao */
    header[0] = MRAM_CMD_FAST_READ;
    for (int i = 0; i < MRAM_ADDR_BYTES; i++) {
        /* MSB first */
        header[1 + i] = (uint8_t)(addr >> ((MRAM_ADDR_BYTES - 1 - i) * 8));
    }
    hdr_len = 1 + MRAM_ADDR_BYTES;

    /* thêm dummy byte */
    header[hdr_len++] = 0x00;

    /* Gửi header rồi nhận data */
    mram_cs_low(dev);
    for (uint32_t i = 0; i < hdr_len; i++) {
        mram_spi_txrx(dev->hspi, header[i]);
    }
    /* Đọc data */
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = mram_spi_txrx(dev->hspi, 0x00);
    }
    mram_cs_high(dev);

    return 0;
}

int8_t mram_write(mram_dev_t *dev, uint32_t addr, const uint8_t *buf, uint32_t len) {
    if (len == 0) return 0;

    /* Trước khi ghi, cần đảm bảo không có WIP */
    if (mram_wait_wip_clear(dev, 10) < 0) {
        return -1;
    }

    /* Nếu có lệnh write enable */
    mram_write_enable(dev);

    uint8_t header[1 + MRAM_ADDR_BYTES];
    header[0] = MRAM_CMD_WRITE;
    for (int i = 0; i < MRAM_ADDR_BYTES; i++) {
        header[1 + i] = (uint8_t)(addr >> ((MRAM_ADDR_BYTES - 1 - i) * 8));
    }

    mram_cs_low(dev);
    /* gửi header */
    for (uint32_t i = 0; i < (1 + MRAM_ADDR_BYTES); i++) {
        mram_spi_txrx(dev->hspi, header[i]);
    }
    /* gửi data */
    for (uint32_t i = 0; i < len; i++) {
        mram_spi_txrx(dev->hspi, buf[i]);
    }
    mram_cs_high(dev);

    /* chờ hoàn tất */
    if (mram_wait_wip_clear(dev, 10) < 0) {
        return -2;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 *  Status register access
 * -------------------------------------------------------------------------- */
int8_t mram_read_status(mram_dev_t *dev, uint8_t *sr) {
    uint8_t buf[2];
    buf[0] = MRAM_CMD_RDSR;
    buf[1] = 0xFF;  /* dummy to clock out status */

    mram_cs_low(dev);
    buf[1] = mram_spi_txrx(dev->hspi, buf[0]);
    buf[1] = mram_spi_txrx(dev->hspi, 0x00);
    mram_cs_high(dev);

    *sr = buf[1];
    return 0;
}

int8_t mram_write_status(mram_dev_t *dev, uint8_t sr) {
    mram_write_enable(dev);

    mram_cs_low(dev);
    mram_spi_txrx(dev->hspi, MRAM_CMD_WRSR);
    mram_spi_txrx(dev->hspi, sr);
    mram_cs_high(dev);

    /* chờ hoàn thành */
    if (mram_wait_wip_clear(dev, 10) < 0) {
        return -1;
    }
    return 0;
}
