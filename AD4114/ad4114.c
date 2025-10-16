/*
 * ad4114.c
 *
 *  Created on: Oct 14, 2025
 *      Author: HTSANG
 */

#include "ad4114.h"
#include <string.h>

/* --------------------------------------------------------------------------
 *  Internal SPI and GPIO helpers
 * -------------------------------------------------------------------------- */
static inline void ad4114_cs_low(ad4114_dev_t *dev)  { LL_GPIO_ResetOutputPin(dev->cs_port, dev->cs_pin); }
static inline void ad4114_cs_high(ad4114_dev_t *dev) { LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin); }

static uint8_t ad4114_spi_txrx(SPI_TypeDef *hspi, uint8_t data)
{
    while (!LL_SPI_IsActiveFlag_TXE(hspi));
    LL_SPI_TransmitData8(hspi, data);
    while (!LL_SPI_IsActiveFlag_RXNE(hspi));
    return LL_SPI_ReceiveData8(hspi);
}

static void ad4114_spi_transfer(ad4114_dev_t *dev, uint8_t *buf, uint16_t len)
{
    ad4114_cs_low(dev);
    for (uint16_t i = 0; i < len; ++i)
        buf[i] = ad4114_spi_txrx(dev->hspi, buf[i]);
    ad4114_cs_high(dev);
}

/* --------------------------------------------------------------------------
 *  CRC Calculation
 * -------------------------------------------------------------------------- */
uint8_t ad4114_calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? ((crc << 1) ^ AD4114_CRC_POLYNOMIAL) : (crc << 1);
    }
    return crc;
}

/* --------------------------------------------------------------------------
 *  Low-level register read/write
 * -------------------------------------------------------------------------- */
static uint8_t ad4114_comms_byte(AD4114_RorW_CMD RorW_CMD, uint8_t reg_addr)
{
    return (is_read ? AD4114_COMMS_READ : AD4114_COMMS_WRITE) | (reg_addr & AD4114_COMMS_ADDR_MASK);
}

static int8_t ad4114_reg_read(ad4114_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    if (len + 2 > sizeof(buf)) return -1;

    buf[0] = ad4114_comms_byte(READ_CMD, reg);
    memset(&buf[1], 0x00, len + (dev->use_crc ? 1 : 0));

    ad4114_spi_transfer(dev, buf, 1 + len + (dev->use_crc ? 1 : 0));

    if (dev->use_crc) {
        uint8_t crc_calc = ad4114_calc_crc8(buf, 1 + len);
        if (crc_calc != buf[1 + len])
            return -2; /* CRC mismatch */
    }
    memcpy(data, &buf[1], len);
    return 0;
}

static int8_t ad4114_reg_write(ad4114_dev_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    if (len + 2 > sizeof(buf)) return -1;

    buf[0] = ad4114_comms_byte(WRITE_CMD, reg);
    memcpy(&buf[1], data, len);
    if (dev->use_crc)
        buf[1 + len] = ad4114_calc_crc8(buf, 1 + len);

    ad4114_spi_transfer(dev, buf, 1 + len + (dev->use_crc ? 1 : 0));
    return 0;
}

/* --------------------------------------------------------------------------
 *  Core initialization and reset
 * -------------------------------------------------------------------------- */
int8_t ad4114_init(ad4114_dev_t *dev)
{
    ad4114_cs_high(dev);
    dev->use_crc = 0;
    dev->last_status = 0;

    uint8_t id[2];
    if (ad4114_reg_read(dev, AD4114_REG_ID, id, 2) != 0)
        return -1;

    ad4114_enable_continuous_read(dev);

    return 0;
}

int8_t ad4114_reset(ad4114_dev_t *dev)
{
    ad4114_cs_low(dev);
    for (uint8_t i = 0; i < 8; ++i)
        ad4114_spi_txrx(dev->hspi, 0xFF);
    ad4114_cs_high(dev);
    return 0;
}

/* --------------------------------------------------------------------------
 *  Operating mode control
 * -------------------------------------------------------------------------- */

/**
 * @brief  Đặt ADC vào chế độ Continuous Conversion
 *         (ADC chuyển đổi liên tục và lưu kết quả vào thanh ghi DATA)
 */
void ad4114_set_continuous_mode(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_CONTINUOUS >> 8),
        (uint8_t)(AD4114_ADCMODE_CONTINUOUS & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

/**
 * @brief  Kích hoạt Continuous Read Mode
 *         (ADC tự động gửi dữ liệu qua SPI mỗi khi có mẫu mới)
 */
void ad4114_enable_continuous_read(ad4114_dev_t *dev)
{
    uint8_t ifmode[2];
    ad4114_reg_read(dev, AD4114_REG_IFMODE, ifmode, 2);
    uint16_t val = ((uint16_t)ifmode[0] << 8) | ifmode[1];
    val |= AD4114_IFMODE_CONT_READ;
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    ad4114_reg_write(dev, AD4114_REG_IFMODE, buf, 2);
}

/**
 * @brief  Đặt ADC vào chế độ Single Conversion
 *         (Chỉ thực hiện 1 lần chuyển đổi rồi tự động dừng)
 */
void ad4114_set_single_mode(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_SINGLE >> 8),
        (uint8_t)(AD4114_ADCMODE_SINGLE & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

/**
 * @brief  Đặt ADC vào Standby Mode
 *         (Dừng chuyển đổi, giữ mạch bias hoạt động để khởi động nhanh)
 */
void ad4114_set_standby_mode(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_STANDBY >> 8),
        (uint8_t)(AD4114_ADCMODE_STANDBY & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

/**
 * @brief  Đặt ADC vào Power-Down Mode
 *         (Tắt hoàn toàn ADC, tiết kiệm điện năng tối đa)
 */
void ad4114_set_powerdown_mode(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_POWER_DOWN >> 8),
        (uint8_t)(AD4114_ADCMODE_POWER_DOWN & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

/* --------------------------------------------------------------------------
 *  Calibration modes
 * -------------------------------------------------------------------------- */
void ad4114_start_internal_zero_cal(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_ZERO_SCALE_CAL >> 8),
        (uint8_t)(AD4114_ADCMODE_ZERO_SCALE_CAL & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

void ad4114_start_internal_full_cal(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_SELF_CAL >> 8),
        (uint8_t)(AD4114_ADCMODE_SELF_CAL & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

void ad4114_start_system_zero_cal(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_SYS_CAL_ZERO >> 8),
        (uint8_t)(AD4114_ADCMODE_SYS_CAL_ZERO & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

void ad4114_start_system_full_cal(ad4114_dev_t *dev)
{
    uint8_t buf[2] = {
        (uint8_t)(AD4114_ADCMODE_SYS_CAL_FULL >> 8),
        (uint8_t)(AD4114_ADCMODE_SYS_CAL_FULL & 0xFF)
    };
    ad4114_reg_write(dev, AD4114_REG_ADCMODE, buf, 2);
}

/* --------------------------------------------------------------------------
 *  Data and status read
 * -------------------------------------------------------------------------- */
uint32_t ad4114_read_data(ad4114_dev_t *dev)
{
    uint8_t buf[3];
    ad4114_reg_read(dev, AD4114_REG_DATA, buf, 3);
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

int8_t ad4114_read_status(ad4114_dev_t *dev, uint8_t *status)
{
    uint8_t buf[1];
    int8_t ret = ad4114_reg_read(dev, AD4114_REG_STATUS, buf, 1);
    if (ret) return ret;
    *status = buf[0];
    dev->last_status = buf[0];
    return 0;
}
