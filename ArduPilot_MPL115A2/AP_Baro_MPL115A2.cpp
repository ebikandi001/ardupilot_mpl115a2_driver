/*
 * AP_Baro_MPL115A2.cpp

 *
 *  Created on: 25/06/2014
 *      Author: ebikandi001
 *
 *       AP_Baro_MPL115A2.cpp - Arduino Library for MPL115A2 sensor
 *       Sensor is conected to I2C port
 *
 *       public methods:
 *       				init(): Setups the HW (reads coefficients values, etc.)
 *       				read(): Reads sensor data and calculates Temperature and Pressure
 *       				accumulate(void): Accumulates the data in order to give an average value of read values.
 *       				get_pressure(): returns the pressure.
 *       				get_temperature(): returns the temperature.
*       private methods:
*       				getPT(float *P, float *T): Calculates pressure and temperature (both at once and saves a little time)
*       				readCoefficients(void): Gets the factory-set coefficients for this particular sensor
*
*
*/


#include "AP_Baro_MPL115A2.h"


extern const AP_HAL::HAL& hal;

bool AP_Baro_MPL115A2::init() {

	// get pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus sempahore
	if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
		return false;

	_temp_sum = 0;
	_press_sum = 0;
	_count = 0;


	readCoefficients();

	healthy = true;

	i2c_sem->give();

	return true;
}


uint8_t AP_Baro_MPL115A2::read(){


	if (_count == 0) {
		accumulate();
	}

    _last_update = hal.scheduler->millis();

    Press = _press_sum / _count;
	Temp =  0.1f * _temp_sum / _count;

	 _pressure_samples = _count;
	_count = 0;
	_temp_sum = 0;
	_press_sum = 0;

	return 1;
}

void AP_Baro_MPL115A2::accumulate(void){
	// get pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus sempahore
	if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
		return;

	//Read the coefficients to ensure the newest data
	readCoefficients();

	//Get the pressure and temperature
	getPT(&Press, &Temp);

	i2c_sem->give();

}

float AP_Baro_MPL115A2::get_pressure() {
	return Press;
}

float AP_Baro_MPL115A2::get_temperature() {
	return Temp;
}



void AP_Baro_MPL115A2::readCoefficients() {

	int16_t a0coeff;
	int16_t b1coeff;
	int16_t b2coeff;
	int16_t c12coeff;
	uint8_t buf[8];
	uint8_t res;

	if (!healthy && hal.scheduler->millis() < _retry_time) {
			return;
		}

	res = hal.i2c->readRegisters(MPL115A2_ADDRESS,
			MPL115A2_REGISTER_A0_COEFF_MSB, 8, buf);

	if (res != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
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


void AP_Baro_MPL115A2::getPT(float *P, float *T) {

	uint16_t pressure, temp;
	float pressureComp;
	uint8_t buf[4];
	uint8_t res;

	// Get raw pressure and temperature settings
	res = hal.i2c->writeRegister(MPL115A2_ADDRESS,
			MPL115A2_REGISTER_STARTCONVERSION, 0x00);

	if (res != 0) {
		healthy = false;
	}

	// Wait a bit for the conversion to complete (3ms max)
	hal.scheduler->delay(3);

	res = hal.i2c->read(MPL115A2_ADDRESS, 4, buf);

	if (res != 0) {
		hal.i2c->setHighSpeed(false);
		healthy = false;
		return;
	}

	pressure = (((uint16_t) buf[0] << 8) | buf[1]) >> 6;
	temp = (((uint16_t) buf[2] << 8) | buf[3]) >> 6;

	// See datasheet p.6 for evaluation sequence
	pressureComp = _mpl115a2_a0
			+ (_mpl115a2_b1 + _mpl115a2_c12 * temp) * pressure
			+ _mpl115a2_b2 * temp;
	// Return pressure and temperature as floating point values
	*P = ((65.0F / 1023.0F) * pressureComp) + 50.0F;        // kPa
	*T = ((float) temp - 498.0F) / -5.35F + 25.0F;

	_temp_sum += *T;
	_press_sum += *P;

	_count++;
	    if (_count == 254) {
	        _temp_sum *= 0.5;
	        _press_sum *= 0.5;
	        _count /= 2;
	    }


}
