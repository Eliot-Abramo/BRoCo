/*
 * I2CDevice.cpp
 *
 *  Created on: Jul 14, 2021
 *      Author: arion
 */

#include "I2CDevice.h"

I2CDevice::I2CDevice(I2CDeviceType type) : type(type) {

}

I2CDevice::~I2CDevice() {

}

I2CDeviceType I2CDevice::getType() {
	return type;
}

I2CMaster::I2CMaster(std::initializer_list<uint8_t> slaves) : I2CDevice(MASTER) {
	this->num_slaves = 0;

	for(uint8_t slave : slaves) {
		this->slaves[num_slaves++] = slave;
	}
}

uint8_t* I2CMaster::getSlaves() {
	return this->slaves;
}

uint8_t I2CMaster::getNumSlaves() {
	return this->num_slaves;
}

I2CSlave::I2CSlave() : I2CDevice(SLAVE) {

}
