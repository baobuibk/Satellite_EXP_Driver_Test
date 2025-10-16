v/*
 * ad4114.h
 *
 *  Created on: Oct 14, 2025
 *      Author: HTSANG
 */

#ifndef AD4114_H_
#define AD4114_H_

#include "stdbool.h"
#include "stdint.h"

/* --------------------------------------------------------------------------
 *  Common definitions
 * -------------------------------------------------------------------------- */
enum AD4114_RorW_CMD { WRITE_CMD = 0, READ_CMD = 1 };

#define AD4114_CRC_POLYNOMIAL        0x07
#define AD4114_COMMS_READ            0x40
#define AD4114_COMMS_WRITE           0x00
#define AD4114_COMMS_ADDR_MASK       0x3F

/* --------------------------------------------------------------------------
 *  ADC Mode Register bitfields
 * -------------------------------------------------------------------------- */
#define AD4114_ADCMODE_CONTINUOUS       (0x00 << 13)
#define AD4114_ADCMODE_SINGLE           (0x01 << 13)
#define AD4114_ADCMODE_STANDBY          (0x02 << 13)
#define AD4114_ADCMODE_POWER_DOWN       (0x03 << 13)
#define AD4114_ADCMODE_ZERO_SCALE_CAL   (0x04 << 13)
#define AD4114_ADCMODE_SELF_CAL         (0x05 << 13)
#define AD4114_ADCMODE_SYS_CAL_ZERO     (0x06 << 13)
#define AD4114_ADCMODE_SYS_CAL_FULL     (0x07 << 13)

/* IFMODE bits */
#define AD4114_IFMODE_CONT_READ         (1 << 6)

/* Register addresses */
#define AD4114_REG_COMMS             0x00
#define AD4114_REG_STATUS            0x00 /* COMMS + STATUS share opcode: datasheet uses Addr 0x00 as COMMS and Status read */
#define AD4114_REG_ADCMODE           0x01
#define AD4114_REG_IFMODE            0x02
#define AD4114_REG_REGCHECK          0x03
#define AD4114_REG_DATA              0x04
#define AD4114_REG_GPIOCON           0x06
#define AD4114_REG_ID                0x07

#define AD4114_REG_CH0               0x10 /* channel registers 0x10..0x1F */
#define AD4114_REG_CH1               0x11
#define AD4114_REG_CH2               0x12
#define AD4114_REG_CH3               0x13
#define AD4114_REG_CH4               0x14
#define AD4114_REG_CH5               0x15
#define AD4114_REG_CH6               0x16
#define AD4114_REG_CH7               0x17
#define AD4114_REG_CH8               0x18
#define AD4114_REG_CH9               0x19
#define AD4114_REG_CH10              0x1A
#define AD4114_REG_CH11              0x1B
#define AD4114_REG_CH12              0x1C
#define AD4114_REG_CH13              0x1D
#define AD4114_REG_CH14              0x1E
#define AD4114_REG_CH15              0x1F

#define AD4114_REG_SETUP0            0x20 /* setup registers 0x20..0x27 */
#define AD4114_REG_SETUP1            0x21
#define AD4114_REG_SETUP2            0x22
#define AD4114_REG_SETUP3            0x23
#define AD4114_REG_SETUP4            0x24
#define AD4114_REG_SETUP5            0x25
#define AD4114_REG_SETUP6            0x26
#define AD4114_REG_SETUP7            0x27

#define AD4114_REG_FILTER0           0x28 /* filter registers 0x28..0x2F */
#define AD4114_REG_FILTER1           0x29
#define AD4114_REG_FILTER2           0x2A
#define AD4114_REG_FILTER3           0x2B
#define AD4114_REG_FILTER4           0x2C
#define AD4114_REG_FILTER5           0x2D
#define AD4114_REG_FILTER6           0x2E
#define AD4114_REG_FILTER7           0x2F

#define AD4114_REG_OFFSET0           0x30 /* offset registers 0x30..0x37 */
#define AD4114_REG_OFFSET1           0x31
#define AD4114_REG_OFFSET2           0x32
#define AD4114_REG_OFFSET3           0x33
#define AD4114_REG_OFFSET4           0x34
#define AD4114_REG_OFFSET5           0x35
#define AD4114_REG_OFFSET6           0x36
#define AD4114_REG_OFFSET7           0x37

#define AD4114_REG_GAIN0             0x38 /* gain registers 0x38..0x3F */
#define AD4114_REG_GAIN1             0x39
#define AD4114_REG_GAIN2             0x3A
#define AD4114_REG_GAIN3             0x3B
#define AD4114_REG_GAIN4             0x3C
#define AD4114_REG_GAIN5             0x3D
#define AD4114_REG_GAIN6             0x3E
#define AD4114_REG_GAIN7             0x3F

/* --------------------------------------------------------------------------
 *  Device structure
 * -------------------------------------------------------------------------- */
typedef struct {
    SPI_TypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *rdy_port;
    uint16_t rdy_pin;
    uint8_t use_crc;
    uint32_t last_status;
} ad4114_dev_t;

/* --------------------------------------------------------------------------
 *  API prototypes
 * -------------------------------------------------------------------------- */

/* Core functions */
int8_t ad4114_init(ad4114_dev_t *dev);
int8_t ad4114_reset(ad4114_dev_t *dev);
int8_t ad4114_enable_crc(ad4114_dev_t *dev, bool enable);

/* Conversion control */
void ad4114_set_continuous_mode(ad4114_dev_t *dev);
void ad4114_enable_continuous_read(ad4114_dev_t *dev);
void ad4114_set_single_mode(ad4114_dev_t *dev);
void ad4114_set_standby_mode(ad4114_dev_t *dev);
void ad4114_set_powerdown_mode(ad4114_dev_t *dev);

/* Calibration */
void ad4114_start_internal_zero_cal(ad4114_dev_t *dev);
void ad4114_start_internal_full_cal(ad4114_dev_t *dev);
void ad4114_start_system_zero_cal(ad4114_dev_t *dev);
void ad4114_start_system_full_cal(ad4114_dev_t *dev);

/* Data readout */
uint32_t ad4114_read_data(ad4114_dev_t *dev);
int8_t ad4114_read_status(ad4114_dev_t *dev, uint8_t *status);

/* Utilities */
uint8_t ad4114_calc_crc8(const uint8_t *data, uint16_t len);

#endif /* AD4114_H_ */
