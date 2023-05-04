/**************************************************************************/
/*!
    @file     Adafruit_ADXL375.cpp
    @author   Bryan Siepert and K.Townsend (Adafruit Industries)

    BSD License (see license.txt)

    The ADXL375 is a digital accelerometer with 13-bit resolution, capable
    of measuring +/-200g.  This driver communicates using I2C or SPI

    This is a library for the Adafruit ADXL375 breakout
    ----> https://www.adafruit.com/

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    v1.0 - First release
*/
/**************************************************************************/

#include "Adafruit_ADXL375.h"
#include "spi.h"


uint8_t ADXL375_begin() {
  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL375 detected ... return false */

    return 0;
  }

  // Default tap detection level (2G, 31.25ms duration, single tap only)
  // If only the single tap function is in use, the single tap interrupt
  // is triggered when the acceleration goes below the threshold, as
  // long as DUR has not been exceeded.
  //write_single_adxl375_reg(ADXL3XX_REG_INT_ENABLE, 0);  // Disable interrupts to start
  //write_single_adxl375_reg(ADXL3XX_REG_THRESH_TAP, 20); // 62.5 mg/LSB (so 0xFF = 16 g)
  //write_single_adxl375_reg(ADXL3XX_REG_DUR, 50);        // Max tap duration, 625 µs/LSB
  //write_single_adxl375_reg(ADXL3XX_REG_LATENT,0); // Tap latency, 1.25 ms/LSB, 0=no double tap
  //write_single_adxl375_reg(ADXL3XX_REG_WINDOW, 0); // Waiting period,  1.25 ms/LSB, 0=no double tap
  //write_single_adxl375_reg(ADXL3XX_REG_TAP_AXES, 0x7); // Enable the XYZ axis for tap

  // Enable measurements
  write_single_adxl375_reg(ADXL3XX_REG_POWER_CTL, 0x08);

  // Force full range (fixes issue with DATA_FORMAT register's reset value)
  // Per datasheet, needs to be D4=0, D3=D1=D0=1
  write_single_adxl375_reg(ADXL3XX_REG_DATA_FORMAT, 0b00001111);
  write_single_adxl375_reg(ADXL3XX_REG_BW_RATE, ADXL3XX_DATARATE_100_HZ);


  return read_single_adxl375_reg(ADXL3XX_REG_BW_RATE);
}

uint8_t getDeviceID(void) {
  // Check device ID register
  return read_single_adxl375_reg(ADXL3XX_REG_DEVID);
}




void ADXL375_GETXYZ(IMUDATA* data) {
  uint8_t buffer[6] = {0, 0, 0,0,0,0};
  uint8_t *temp = read_multiple_adxl375_reg(ADXL3XX_REG_DATAX0,6);
  data->accel200g_x = (int16_t) (temp[1] << 8 | temp[0]);
  data->accel200g_y = (int16_t) (temp[3] << 8 | temp[2]);
  data->accel200g_z = (int16_t) (temp[5] << 8 | temp[4]);
}


static uint8_t read_single_adxl375_reg(uint8_t reg)
{
	uint8_t read_reg = 0x80 | reg;
	uint8_t reg_val;

	cs_low();
	HAL_SPI_Transmit(ADXL375_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ADXL375_SPI, &reg_val, 1, 1000);
	cs_high();

	return reg_val;
}

static void write_single_adxl375_reg( uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = 0x00 | reg;
	write_reg[1] = val;

	cs_low();
	HAL_SPI_Transmit(ADXL375_SPI, write_reg, 2, 1000);
	cs_high();
}

static uint8_t* read_multiple_adxl375_reg( uint8_t reg, uint8_t len)
{
	uint8_t read_reg = 0x80 | reg;
	read_reg = read_reg | 0x40;
	static uint8_t reg_val[6];

	cs_low();
	HAL_SPI_Transmit(ADXL375_SPI, &read_reg, 1,1000);
	HAL_SPI_Receive(ADXL375_SPI, reg_val, len,1000);
	cs_high();

	return reg_val;
}

static void write_multiple_adxl375_reg(uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = 0x00 | reg;

	cs_low();
	HAL_SPI_Transmit(ADXL375_SPI, &write_reg, 1, 1000);
	HAL_SPI_Transmit(ADXL375_SPI, val, len, 1000);
	cs_high();
}

void cs_low(){
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}
void  cs_high(){
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}
