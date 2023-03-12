/*
 * port.h
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */

#ifndef INCLUDE_PORT_H_
#define INCLUDE_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#ifdef PORT_PI4MSD5V9540B
#include "PI4MSD5V9540B.h"
#define PORT_CHANNEL_ANY PI4MSD5V9540B_CHANNEL_ANY
#define PORT_CHANNEL_NONE PI4MSD5V9540B_CHANNEL_NONE
#define PORT_CHANNEL_0 PI4MSD5V9540B_CHANNEL_0
#define PORT_CHANNEL_1 PI4MSD5V9540B_CHANNEL_1
#define PORT_MUX_TYPEDEF PI4MSD5V9540B_inst_t
#define PORT_MUX_DEFAULT_ADDRESS PI4MSD5V9540B_DEFAULT_ADDRESS
#define PORT_MUX_OK PI4MSD5V9540B_OK
#define PORT_MUX_ERROR PI4MSD5V9540B_ERROR
#endif

#ifdef PORT_PCA9548A
#include "PCA9548A.h"
#define PORT_CHANNEL_ANY PCA9548A_CHANNEL_ANY
#define PORT_CHANNEL_NONE PCA9548A_CHANNEL_NONE
#define PORT_CHANNEL_0 PCA9548A_CHANNEL_0
#define PORT_CHANNEL_1 PCA9548A_CHANNEL_1
#define PORT_CHANNEL_2 PCA9548A_CHANNEL_2
#define PORT_CHANNEL_3 PCA9548A_CHANNEL_3
#define PORT_CHANNEL_4 PCA9548A_CHANNEL_4
#define PORT_CHANNEL_5 PCA9548A_CHANNEL_5
#define PORT_CHANNEL_6 PCA9548A_CHANNEL_6
#define PORT_CHANNEL_7 PCA9548A_CHANNEL_7
#define PORT_MUX_TYPEDEF PCA9548A_inst_t
#define PORT_MUX_DEFAULT_ADDRESS PCA9548A_DEFAULT_ADDRESS
#define PORT_MUX_OK PCA9548A_OK
#define PORT_MUX_ERROR PCA9548A_ERROR
#endif

#define PORT_MUX_NONE 0

#define PORT_ERROR PICO_ERROR_GENERIC
#define PORT_OK PICO_OK

#define PORT_TIME_OUT_BASE (50)
#define PORT_TIME_OUT_BYTE (8)

#define PORT_I2C_MAX_NUM_OF_MUXES 1

#define PORT_SPI_MAX_NUM_OF_DEVICES 5

typedef struct {
	PORT_MUX_TYPEDEF mux_inst;
	i2c_inst_t *i2c;
} port_i2c_t;

typedef struct {
	spi_inst_t *spi;
	uint num_of_devices;
	uint cs_pins[PORT_SPI_MAX_NUM_OF_DEVICES];
} port_spi_t;

typedef struct {
    port_spi_t *port;
    uint dev_num;
} port_spi_dev_t;

int port_i2c_init(port_i2c_t *inst, i2c_inst_t *i2c, uint8_t address);
int port_i2c_device_select(port_i2c_t *inst, int8_t channel);
int port_i2c_device_check(port_i2c_t *inst, int8_t channel, uint8_t address);
int port_i2c_write(port_i2c_t *inst, int8_t channel, uint8_t address, uint8_t *data, uint32_t size);
int port_i2c_read(port_i2c_t *inst, int8_t channel, uint8_t address, uint8_t *data, uint32_t size);

int port_spi_init(port_spi_t *inst, spi_inst_t *spi);
int port_spi_add_device(port_spi_t *inst, uint cs_pin);
int port_spi_write_mem(port_spi_t *inst, uint device, uint8_t address, uint8_t *data, uint32_t size);
int port_spi_read_mem(port_spi_t *inst, uint device, uint8_t address, uint8_t *data, uint32_t size);

void port_delay(uint32_t ms);
void port_delay_ms(uint32_t ms);
void port_delay_us(uint64_t us);

static inline int mux_init(PORT_MUX_TYPEDEF *inst, i2c_inst_t *i2c, uint8_t address)
{
	#ifdef PORT_PI4MSD5V9540B
	return PI4MSD5V9540B_init(inst, i2c, address);
	#endif

	#ifdef PORT_PCA9548A
	return PCA9548A_init(inst, i2c, address);
	#endif
}

static inline int mux_set_channel(PORT_MUX_TYPEDEF *inst, int8_t channel)
{
	#ifdef PORT_PI4MSD5V9540B
	return PI4MSD5V9540B_set_channel(inst, channel);
	#endif

	#ifdef PORT_PCA9548A
	return PCA9548A_set_channel(inst, channel);
	#endif
}

static inline int mux_get_channel(PORT_MUX_TYPEDEF *inst)
{
	#ifdef PORT_PI4MSD5V9540B
	return PI4MSD5V9540B_get_channel(inst);
	#endif

	#ifdef PORT_PCA9548A
	return PCA9548A_get_channel(inst);
	#endif
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_PORT_H_ */
