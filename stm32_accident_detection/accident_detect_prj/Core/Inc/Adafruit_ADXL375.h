/**************************************************************************/
/*!
    @file     Adafruit_ADXL375.h
    @author   Bryan Siepert and K. Townsend (Adafruit Industries)

    BSD license (see license.txt)

    This is a library for the Adafruit ADXL375 breakout board
    ----> https://www.adafruit.com

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    v1.0  - First release
*/
/**************************************************************************/
#ifndef _ADAFRUIT_SENSOR_ADXL375_H
#define _ADAFRUIT_SENSOR_ADXL375_H

#include "spi.h"
#include "common_def.h"

#define ADXL375_SPI					(&hspi4)

#define ADXL375_CS_PIN_PORT		GPIOE
#define ADXL375_CS_PIN_NUMBER		GPIO_PIN_4

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define ADXL3XX_REG_DEVID (0x00)        /**< Device ID */
#define ADXL3XX_REG_THRESH_TAP (0x1D)   /**< Tap threshold */
#define ADXL3XX_REG_OFSX (0x1E)         /**< X-axis offset */
#define ADXL3XX_REG_OFSY (0x1F)         /**< Y-axis offset */
#define ADXL3XX_REG_OFSZ (0x20)         /**< Z-axis offset */
#define ADXL3XX_REG_DUR (0x21)          /**< Tap duration */
#define ADXL3XX_REG_LATENT (0x22)       /**< Tap latency */
#define ADXL3XX_REG_WINDOW (0x23)       /**< Tap window */
#define ADXL3XX_REG_THRESH_ACT (0x24)   /**< Activity threshold */
#define ADXL3XX_REG_THRESH_INACT (0x25) /**< Inactivity threshold */
#define ADXL3XX_REG_TIME_INACT (0x26)   /**< Inactivity time */
#define ADXL3XX_REG_ACT_INACT_CTL (0x27) /**< Axis enable control for activity and inactivity detection */
#define ADXL3XX_REG_THRESH_FF (0x28) /**< Free-fall threshold */
#define ADXL3XX_REG_TIME_FF (0x29)   /**< Free-fall time */
#define ADXL3XX_REG_TAP_AXES (0x2A)  /**< Axis control for single/double tap */
#define ADXL3XX_REG_ACT_TAP_STATUS (0x2B) /**< Source for single/double tap */
#define ADXL3XX_REG_BW_RATE (0x2C)     /**< Data rate and power mode control */
#define ADXL3XX_REG_POWER_CTL (0x2D)   /**< Power-saving features control */
#define ADXL3XX_REG_INT_ENABLE (0x2E)  /**< Interrupt enable control */
#define ADXL3XX_REG_INT_MAP (0x2F)     /**< Interrupt mapping control */
#define ADXL3XX_REG_INT_SOURCE (0x30)  /**< Source of interrupts */
#define ADXL3XX_REG_DATA_FORMAT (0x31) /**< Data format control */
#define ADXL3XX_REG_DATAX0 (0x32)      /**< X-axis data 0 */
#define ADXL3XX_REG_DATAX1 (0x33)      /**< X-axis data 1 */
#define ADXL3XX_REG_DATAY0 (0x34)      /**< Y-axis data 0 */
#define ADXL3XX_REG_DATAY1 (0x35)      /**< Y-axis data 1 */
#define ADXL3XX_REG_DATAZ0 (0x36)      /**< Z-axis data 0 */
#define ADXL3XX_REG_DATAZ1 (0x37)      /**< Z-axis data 1 */
#define ADXL3XX_REG_FIFO_CTL (0x38)    /**< FIFO control */
#define ADXL3XX_REG_FIFO_STATUS (0x39) /**< FIFO status */
/*=========================================================================*/

#define ADXL375_MG2G_MULTIPLIER (0.049) /**< 49mg per lsb */
/*=========================================================================*/
typedef enum {
  ADXL3XX_DATARATE_3200_HZ = 0b1111, /**< 3200Hz Bandwidth */
  ADXL3XX_DATARATE_1600_HZ = 0b1110, /**< 1600Hz Bandwidth */
  ADXL3XX_DATARATE_800_HZ = 0b1101,  /**<  800Hz Bandwidth */
  ADXL3XX_DATARATE_400_HZ = 0b1100,  /**<  400Hz Bandwidth */
  ADXL3XX_DATARATE_200_HZ = 0b1011,  /**<  200Hz Bandwidth */
  ADXL3XX_DATARATE_100_HZ = 0b1010,  /**<  100Hz Bandwidth */
  ADXL3XX_DATARATE_50_HZ = 0b1001,   /**<   50Hz Bandwidth */
  ADXL3XX_DATARATE_25_HZ = 0b1000,   /**<   25Hz Bandwidth */
  ADXL3XX_DATARATE_12_5_HZ = 0b0111, /**< 12.5Hz Bandwidth */
  ADXL3XX_DATARATE_6_25HZ = 0b0110,  /**< 6.25Hz Bandwidth */
  ADXL3XX_DATARATE_3_13_HZ = 0b0101, /**< 3.13Hz Bandwidth */
  ADXL3XX_DATARATE_1_56_HZ = 0b0100, /**< 1.56Hz Bandwidth */
  ADXL3XX_DATARATE_0_78_HZ = 0b0011, /**< 0.78Hz Bandwidth */
  ADXL3XX_DATARATE_0_39_HZ = 0b0010, /**< 0.39Hz Bandwidth */
  ADXL3XX_DATARATE_0_20_HZ = 0b0001, /**< 0.20Hz Bandwidth */
  ADXL3XX_DATARATE_0_10_HZ = 0b0000  /**< 0.10Hz Bandwidth (default value) */
} adxl3xx_dataRate_t;

/** Used with register 0x31 (ADXL3XX_REG_DATA_FORMAT) to set g range */



/**
 * Driver for the Adafruit ADXL375 breakout.
 */


uint8_t ADXL375_begin();

uint8_t getDeviceID(void);

static uint8_t* read_multiple_adxl375_reg( uint8_t reg, uint8_t len);
static void write_multiple_adxl375_reg(uint8_t reg, uint8_t* val, uint8_t len);
static uint8_t read_single_adxl375_reg(uint8_t reg);
static void write_single_adxl375_reg( uint8_t reg, uint8_t val);
void ADXL375_GETXYZ(IMUDATA* data);


#endif
