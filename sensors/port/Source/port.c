/*
 * port.c
 *
 *  Created on: Oct 15, 2022
 *      Author: minda
 */
#include "port.h"
#include "stdio.h"
#include "pico/stdlib.h"
int port_i2c_init(port_i2c_t *inst, i2c_inst_t *i2c, uint8_t address)
{
	inst->i2c = i2c;
	
	if (address != PORT_MUX_NONE)
	{
		if(mux_init(&inst->mux_inst, i2c, address) == PORT_MUX_ERROR)
			return PORT_ERROR;
	}
	else
	{
		inst->mux_inst.address = PORT_MUX_NONE;
		inst->mux_inst.i2c = 0;
	}
	
	return PORT_OK;
}

int port_i2c_device_select(port_i2c_t *inst, int8_t channel)
{
	return mux_set_channel(&inst->mux_inst, channel);
}

int port_i2c_device_check(port_i2c_t *inst, int8_t channel, uint8_t address)
{
	uint8_t rxdata;
	int status = PORT_OK;
	if (channel != PORT_CHANNEL_ANY && inst->mux_inst.address != PORT_MUX_NONE)
		status |= port_i2c_device_select(inst, channel);
	
	status |= i2c_read_blocking(inst->i2c, address, &rxdata, 1, false);

	return status;
}

int port_i2c_write(port_i2c_t *inst, int8_t channel, uint8_t address, uint8_t *data, uint32_t size)
{
	int status = PORT_OK;
	if (channel != PORT_CHANNEL_ANY && inst->mux_inst.address != PORT_MUX_NONE)
		status |= port_i2c_device_select(inst, channel);
	
	status |= i2c_write_blocking(inst->i2c, address, data, size, false);

	return status;
}

int port_i2c_read(port_i2c_t *inst, int8_t channel, uint8_t address, uint8_t *data, uint32_t size)
{
	int status = PORT_OK;
	if (channel != PORT_CHANNEL_ANY && inst->mux_inst.address != PORT_MUX_NONE)
		status |= port_i2c_device_select(inst, channel);
	
	status |= i2c_read_blocking(inst->i2c, address, data, size, false);

	return status;
}

int port_spi_init(port_spi_t *inst, spi_inst_t *spi)
{
	inst->spi = spi;

	return PORT_OK;
}

int port_spi_add_device(port_spi_t *inst, uint cs_pin)
{	
	if (inst->num_of_devices >= PORT_SPI_MAX_NUM_OF_DEVICES) return PORT_ERROR;

	gpio_init(cs_pin);
	gpio_set_dir(cs_pin, GPIO_OUT);
	gpio_put(cs_pin, 1);

	int dev_num = inst->num_of_devices;
	inst->cs_pins[inst->num_of_devices++] = cs_pin;

	return dev_num;
}

int port_spi_write_mem(port_spi_t *inst, uint device, uint8_t address, uint8_t *data, uint32_t size)
{
	int status = PORT_OK;
    uint cs_pin = inst->cs_pins[device];
    gpio_put(cs_pin, 0);
	spi_write_blocking(inst->spi, &address, 1);
    spi_write_blocking(inst->spi, data, size);
    gpio_put(cs_pin, 1);

    return status;
}

int port_spi_read_mem(port_spi_t *inst, uint device, uint8_t address, uint8_t *data, uint32_t size)
{
    int status = PORT_OK;
    uint cs_pin = inst->cs_pins[device];
    gpio_put(cs_pin, 0);
    spi_write_blocking(inst->spi, &address, 1);
    spi_read_blocking(inst->spi, 0x00, data, size);
    gpio_put(cs_pin, 1);

    return status;
}

void port_delay(uint32_t ms)
{
	sleep_ms(ms);
}

void port_delay_ms(uint32_t ms)
{
	sleep_ms(ms);
}

void port_delay_us(uint64_t us)
{
	sleep_us(us);
}
