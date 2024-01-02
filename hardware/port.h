/*
 * 2023-10-06, Minduagas Mikalauskas.
 */

#ifndef _PORT_H
#define _PORT_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "i2c_mux.h"

#define PORT_ERROR PICO_ERROR_GENERIC
#define PORT_OK PICO_OK

// #define PORT_TIME_OUT_BASE (50)
// #define PORT_TIME_OUT_BYTE (8)

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	PORT_SPI = 1,
	// PORT_I2C = 2
	PORT_I2C_MUX = 3
} port_type_t;

typedef struct
{
	void *hw_intf;
	uint hw_intf_index;
	uint hw_address;
	port_type_t hw_type;
} port_device_t;

static inline int port_i2c_mux_device_check(i2c_mux_inst_t *mux, uint8_t dev_channel, uint8_t dev_address)
{
	uint8_t rxdata;
	int status = PORT_OK;

	status |= i2c_mux_set_channel(mux, dev_channel);

	status |= i2c_read_blocking(mux->i2c, dev_address, &rxdata, 1, false);

	return status;
}

static inline int port_i2c_mux_write(i2c_mux_inst_t *mux, uint8_t dev_channel, uint8_t dev_address, const uint8_t *data, uint32_t size)
{
	int status = PORT_OK;

	status |= i2c_mux_set_channel(mux, dev_channel);

	status |= i2c_write_blocking(mux->i2c, dev_address, data, size, false);

	return status;
}

static inline int port_i2c_mux_read(i2c_mux_inst_t *mux, uint8_t dev_channel, uint8_t dev_address, uint8_t *data, uint32_t size)
{
	int status = PORT_OK;

	status |= i2c_mux_set_channel(mux, dev_channel);

	status |= i2c_read_blocking(mux->i2c, dev_address, data, size, false);

	return status;
}

static inline int port_spi_write_mem(spi_inst_t *spi, uint cs, uint8_t address, const uint8_t *data, uint32_t size)
{
	gpio_put(cs, 0);
	spi_write_blocking(spi, &address, 1);
	spi_write_blocking(spi, data, size);
	gpio_put(cs, 1);

	return PORT_OK;
}

static inline int port_spi_read_mem(spi_inst_t *spi, uint cs, uint8_t address, uint8_t *data, uint32_t size)
{
	gpio_put(cs, 0);
	spi_write_blocking(spi, &address, 1);
	spi_read_blocking(spi, 0x00, data, size);
	gpio_put(cs, 1);

	return PORT_OK;
}

static inline int port_write_mem(port_device_t *device, uint8_t address, const uint8_t *data, uint32_t size)
{
	switch (device->hw_type)
	{
	case PORT_SPI:
		return port_spi_write_mem((spi_inst_t *)device->hw_intf, device->hw_intf_index, address, data, size);
	}

	return PORT_ERROR;
}

static inline int port_read_mem(port_device_t *device, uint8_t address, uint8_t *data, uint32_t size)
{
	switch (device->hw_type)
	{
	case PORT_SPI:
		return port_spi_read_mem((spi_inst_t *)device->hw_intf, device->hw_intf_index, address, data, size);
	}

	return PORT_ERROR;
}

static inline int port_write(port_device_t *device, const uint8_t *data, uint32_t size)
{
	switch (device->hw_type)
	{
	case PORT_I2C_MUX:
		return port_i2c_mux_write((i2c_mux_inst_t *)device->hw_intf, device->hw_intf_index, device->hw_address, data, size);
	}

	return PORT_ERROR;
}

static inline int port_read(port_device_t *device, uint8_t *data, uint32_t size)
{
	switch (device->hw_type)
	{
	case PORT_I2C_MUX:
		return port_i2c_mux_read((i2c_mux_inst_t *)device->hw_intf, device->hw_intf_index, device->hw_address, data, size);
	}

	return PORT_ERROR;
}

static inline void port_delay(uint32_t ms)
{
	sleep_ms(ms);
}

static inline void port_delay_ms(uint32_t ms)
{
	sleep_ms(ms);
}

static inline void port_delay_us(uint64_t us)
{
	sleep_us(us);
}

#ifdef __cplusplus
}
#endif

#endif
