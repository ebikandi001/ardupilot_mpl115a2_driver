/*
 * AP_Baro_MPL115A2.cpp
 *
 *  Created on: 25/06/2014
 *      Author: ebikandi001
 */
#include "AP_Baro_MPL115A2.h"

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////
bool AP_Baro_MPL115A2::init()
{
	//TODO
}

void AP_Baro_MPL115A2::accumulate(void)
{
	//TODO
}

float AP_Baro_MPL115A2::get_pressure() {
    return getPressure();
}

float AP_Baro_MPL115A2::get_temperature() {
    return getTemperature();
}

// Private Methods //////////////////////////////////////////////
//Based on the Driver AdaFruit_MPL115A2
void AP_Baro_MPL115A2::readCoefficients()
{
	//TODO
}


void 	AP_Baro_MPL115A2::begin(void)
{
	 hal.i2c->begin();
	  // Read factory coefficient values (this only needs to be done once)
	 readCoefficients();
}

float 	AP_Baro_MPL115A2::getPressure(void)
{
	float     pressureComp,centigrade;
	getPT(&pressureComp, &centigrade);
	return pressureComp;
}

float 	AP_Baro_MPL115A2::getTemperature(void)
{
	float     pressureComp, centigrade;
	getPT(&pressureComp, &centigrade);
	return centigrade;}


void AP_Baro_MPL115A2::getPT(float *P, float *T)
{
	//FIXME
	uint16_t 	pressure, temp;
	float     pressureComp;
	uint8_t buf[4];

	// Get raw pressure and temperature settings
	hal.i2c->writeRegister(MPL115A2_ADDRESS,MPL115A2_REGISTER_STARTCONVERSION, 0x00);
//	Wire.beginTransmission(MPL115A2_ADDRESS);
//	i2cwrite((uint8_t)MPL115A2_REGISTER_STARTCONVERSION);
//	i2cwrite((uint8_t)0x00);
//	Wire.endTransmission();

	// Wait a bit for the conversion to complete (3ms max)
	sleep(5);
//	delay(5);

//FIXME MPL115A2_REGISTER_PRESSURE_MSB = 0x00 ???
	hal.i2c->writeRegister(MPL115A2_ADDRESS,0x00, MPL115A2_REGISTER_PRESSURE_MSB);
//	Wire.beginTransmission(MPL115A2_ADDRESS);
//	i2cwrite((uint8_t)MPL115A2_REGISTER_PRESSURE_MSB);  // Register
//	Wire.endTransmission();

	hal.i2c->read(MPL115A2_ADDRESS, 4, buf);
//	Wire.requestFrom(MPL115A2_ADDRESS, 4);

	//TODO checkout the buf[] indexes
	pressure = (( (uint16_t) buf[0] << 8) | buf[1]) >> 6;
	temp = (( (uint16_t) buf[2] << 8) | buf[3]) >> 6;
//	pressure = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;
//	temp = (( (uint16_t) i2cread() << 8) | i2cread()) >> 6;

	// See datasheet p.6 for evaluation sequence
	pressureComp = _mpl115a2_a0 + (_mpl115a2_b1 + _mpl115a2_c12 * temp ) * pressure + _mpl115a2_b2 * temp;
	// Return pressure and temperature as floating point values
	*P = ((65.0F / 1023.0F) * pressureComp) + 50.0F;        // kPa
	*T = ((float) temp - 498.0F) / -5.35F +25.0F;
}
