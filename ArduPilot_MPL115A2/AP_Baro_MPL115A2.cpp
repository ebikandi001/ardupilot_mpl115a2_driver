/*
 * AP_Baro_MPL115A2.cpp
 *
 *  Created on: 25/06/2014
 *      Author: ebikandi001
 */
#include "AP_Baro_MPL115A2.h"

extern const AP_HAL::HAL& hal;

//bool healthy; --> AP_Baro.h

// Public Methods //////////////////////////////////////////
bool AP_Baro_MPL115A2::init() {
	//uint8_t buff[22];

	// get pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus sempahore
	if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
		return false;

	readCoefficients();

	healthy = true;
	i2c_sem->give();
	return true;
}

//void AP_Baro_MPL115A2::accumulate(void)
//{
//
//}

float AP_Baro_MPL115A2::get_pressure() {
	return getPressure();
}

float AP_Baro_MPL115A2::get_temperature() {
	return getTemperature();
}

// Private Methods //////////////////////////////////////////////
//Based on the Driver AdaFruit_MPL115A2
void AP_Baro_MPL115A2::readCoefficients() {
	//FIXME repasar
	if (!healthy)
		return;

	int16_t a0coeff;
	int16_t b1coeff;
	int16_t b2coeff;
	int16_t c12coeff;
	uint8_t buf[8];
	uint8_t res;

	res = hal.i2c->readRegisters(MPL115A2_ADDRESS,
			MPL115A2_REGISTER_A0_COEFF_MSB, 8, buf);
	if (res != 0) {
		hal.i2c->setHighSpeed(false);
		healthy = false;
		return;
	}

	a0coeff = (((uint16_t) buf[0] << 8) | buf[1]);
	b1coeff = (((uint16_t) buf[2] << 8) | buf[3]);
	b2coeff = (((uint16_t) buf[4] << 8) | buf[5]);
	c12coeff = (((uint16_t) (buf[6] << 8) | buf[7])) >> 2;

	_mpl115a2_a0 = (float) a0coeff / 8;
	_mpl115a2_b1 = (float) b1coeff / 8192;
	_mpl115a2_b2 = (float) b2coeff / 16384;
	_mpl115a2_c12 = (float) c12coeff;
	_mpl115a2_c12 /= 4194304.0;

	/*
	 Serial.print("a0 = "); Serial.println(_mpl115a2_a0);
	 Serial.print("b1 = "); Serial.println(_mpl115a2_b1);
	 Serial.print("b2 = "); Serial.println(_mpl115a2_b2);
	 Serial.print("c12 = "); Serial.println(_mpl115a2_c12);
	 */
}

//void 	AP_Baro_MPL115A2::begin(void)
//{
//	 hal.i2c->begin();
//	  // Read factory coefficient values (this only needs to be done once)
//	 readCoefficients();
//}

float AP_Baro_MPL115A2::getPressure(void) {
	float pressureComp, centigrade;
	getPT(&pressureComp, &centigrade);
	return pressureComp;
}

float AP_Baro_MPL115A2::getTemperature(void) {
	float pressureComp, centigrade;
	getPT(&pressureComp, &centigrade);
	return centigrade;
}

void AP_Baro_MPL115A2::getPT(float *P, float *T) {
	if (!healthy)
		return;

	uint16_t pressure, temp;
	float pressureComp;
	uint8_t buf[4];
	uint8_t res;

	// Get raw pressure and temperature settings
	//FIXME MPL115A2_REGISTER_PRESSURE_MSB = 0x00 ???
	res = hal.i2c->writeRegister(MPL115A2_ADDRESS,
			MPL115A2_REGISTER_STARTCONVERSION, 0x00);
	if (res != 0) {
		healthy = false;
	}

	// Wait a bit for the conversion to complete (3ms max)
	sleep(5);	//delay(5);

	//TODO test the read to the register 0x00 (PL115A2_REGISTER_PRESSURE_MSB)
	res = hal.i2c->read(MPL115A2_ADDRESS, 4, buf);

	if (res != 0) {
		hal.i2c->setHighSpeed(false);
		healthy = false;
		return;
	}
	//TODO checkout the buf[] indexes and the pressure/Temp registers
	pressure = (((uint16_t) buf[0] << 8) | buf[1]) >> 6;
	temp = (((uint16_t) buf[2] << 8) | buf[3]) >> 6;

	// See datasheet p.6 for evaluation sequence
	pressureComp = _mpl115a2_a0
			+ (_mpl115a2_b1 + _mpl115a2_c12 * temp) * pressure
			+ _mpl115a2_b2 * temp;
	// Return pressure and temperature as floating point values
	*P = ((65.0F / 1023.0F) * pressureComp) + 50.0F;        // kPa
	*T = ((float) temp - 498.0F) / -5.35F + 25.0F;
}
