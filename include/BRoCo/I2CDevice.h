/*
 * I2CDevice.h
 *
 *  Created on: Jul 14, 2021
 *      Author: arion
 */

#ifndef LIBRARIES_ROCO_SRC_I2CDEVICE_H_
#define LIBRARIES_ROCO_SRC_I2CDEVICE_H_

#include <initializer_list>
#include <cstdint>


enum I2CDeviceType {
	MASTER, SLAVE
};

class I2CDevice {
public:
	I2CDeviceType getType();

protected:
	I2CDevice(I2CDeviceType type);
	virtual ~I2CDevice();

private:
	I2CDeviceType type;
};

class I2CMaster : public I2CDevice {
public:
	I2CMaster(std::initializer_list<uint8_t> slaves);
	uint8_t* getSlaves();
	uint8_t getNumSlaves();

private:
	uint8_t slaves[256];
	uint8_t num_slaves;
};

class I2CSlave : public I2CDevice {
public:
	I2CSlave();
};


#endif /* LIBRARIES_ROCO_SRC_I2CDEVICE_H_ */
