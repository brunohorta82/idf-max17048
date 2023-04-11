#pragma once
#include "freertos/FreeRTOS.h"
#include "I2C.hpp"
#define VCELL_REGISTER 0x02
#define SOC_REGISTER 0x04
#define MODE_REGISTER 0x06
#define VERSION_REGISTER 0x08
#define CONFIG_REGISTER 0x0C
#define COMMAND_REGISTER 0xFE

using namespace std;
using namespace Components;
class MAX17048
{

public:
	MAX17048(uint8_t address, shared_ptr<I2CMaster> i2cMasterBus)
	{
		this->address = address;
		this->i2cMasterBus = i2cMasterBus;
	};
	float getVCell();
	float getSoC();
	int getVersion();
	uint8_t getCompensateValue();
	uint8_t getAlertThreshold();
	void setAlertThreshold(uint8_t threshold);
	bool inAlert();
	void clearAlert();
	uint8_t getAddress()
	{
		return this->address;
	}
	void reset();
	void quickStart();
	shared_ptr<I2CMaster> getBus()
	{
		return this->i2cMasterBus;
	}

private:
	uint8_t address;
	shared_ptr<I2CMaster> i2cMasterBus;
	void readConfigRegister(uint8_t &MSB, uint8_t &LSB);
	void readRegister(uint8_t startAddress, uint8_t &MSB, uint8_t &LSB);
	void writeRegister(uint8_t address, uint8_t MSB, uint8_t LSB);
};
