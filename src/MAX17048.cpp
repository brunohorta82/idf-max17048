#include "MAX17048.hpp"
#include "freertos/FreeRTOS.h"
#include <esp_log.h>
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	const long run = in_max - in_min;
	if (run == 0)
	{
		return -1;
	}
	const long rise = out_max - out_min;
	const long delta = x - in_min;
	return (delta * rise) / run + out_min;
}
float MAX17048::getVCell()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readRegister(VCELL_REGISTER, MSB, LSB);
	int value = (MSB << 4) | (LSB >> 4);
	return map(value, 0x000, 0xFFF, 0, 50000) / 10000.0;
	// return value * 0.00125;
}

float MAX17048::getSoC()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readRegister(SOC_REGISTER, MSB, LSB);
	float decimal = LSB / 256.0;
	return MSB + decimal;
}

int MAX17048::getVersion()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readRegister(VERSION_REGISTER, MSB, LSB);
	return (MSB << 8) | LSB;
}

uint8_t MAX17048::getCompensateValue()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readConfigRegister(MSB, LSB);
	return MSB;
}

uint8_t MAX17048::getAlertThreshold()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readConfigRegister(MSB, LSB);
	return 32 - (LSB & 0x1F);
}

void MAX17048::setAlertThreshold(uint8_t threshold)
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readConfigRegister(MSB, LSB);
	if (threshold > 32)
		threshold = 32;
	threshold = 32 - threshold;

	writeRegister(CONFIG_REGISTER, MSB, (LSB & 0xE0) | threshold);
}

bool MAX17048::inAlert()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readConfigRegister(MSB, LSB);
	return LSB & 0x20;
}

void MAX17048::clearAlert()
{

	uint8_t MSB = 0;
	uint8_t LSB = 0;

	readConfigRegister(MSB, LSB);
}

void MAX17048::reset()
{

	writeRegister(COMMAND_REGISTER, 0x00, 0x54);
}

void MAX17048::quickStart()
{

	writeRegister(MODE_REGISTER, 0x40, 0x00);
}

void MAX17048::readConfigRegister(uint8_t &MSB, uint8_t &LSB)
{

	readRegister(CONFIG_REGISTER, MSB, LSB);
}

void MAX17048::readRegister(uint8_t startAddress, uint8_t &MSB, uint8_t &LSB)
{
	try
	{
		getBus()->syncWrite(I2CAddress(getAddress()), {startAddress});
		vector<uint8_t> data = getBus()->syncRead(I2CAddress(getAddress()), 2);
		MSB = data.data()[0];
		LSB = data.data()[1];
	}
	catch (const I2CException &e)
	{
		ESP_LOGE("MAX17048", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
	}
}

void MAX17048::writeRegister(uint8_t address, uint8_t MSB, uint8_t LSB)
{
	try
	{
		vector<uint8_t> data;
		data.push_back(address);
		data.push_back(MSB);
		data.push_back(LSB);
		getBus()->syncWrite(I2CAddress(getAddress()), data);
	}
	catch (const I2CException &e)
	{
		ESP_LOGI("MAX17048", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
	}
}