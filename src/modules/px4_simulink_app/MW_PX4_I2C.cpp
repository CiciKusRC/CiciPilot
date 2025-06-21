/* Copyright 2020 The MathWorks, Inc. */

#include "MW_PX4_I2C.h"
#include<stdio.h>


MW_I2C_DEVICE *
MW_I2C_DEVICE_interface(int bus)
{
	return new MW_I2C_DEVICE(bus);
}

MW_I2C_DEVICE::MW_I2C_DEVICE(int bus) :
	I2C("MW_I2C_DEVICE", nullptr, bus, 0x01, 400000)
{
	_device_id.devid_s.devtype =  0x80;
}

int
MW_I2C_DEVICE::init()
{
	/* this will call probe() */
	return I2C::init();
}

int
MW_I2C_DEVICE::setBusSpeed(int bus, uint32_t frequency)
{
    return I2C::set_bus_clock(bus,frequency);
}

int
MW_I2C_DEVICE::write(uint16_t address, uint8_t *data, unsigned count)
{
    set_device_address(address);
	return transfer(data, count, nullptr, 0);
}

int
MW_I2C_DEVICE::read(uint16_t address, uint8_t *data, unsigned count)
{
    set_device_address(address);
	return transfer(nullptr, 0, data, count);

}

