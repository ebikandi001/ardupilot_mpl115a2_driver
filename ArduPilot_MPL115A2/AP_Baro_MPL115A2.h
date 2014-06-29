#ifndef __AP_BARO_MPL115A2_H__
#define __AP_BARO_MPL115A2_H__

#include "stdint.h"
#include "unistd.h"
#include "AP_Baro.h"
#include "AP_HAL.h"
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library


/*=========================================================================
 I2C ADDRESS/BITS
 -----------------------------------------------------------------------*/
#define MPL115A2_ADDRESS                       (0x60)    // 1100000
/*=========================================================================*/

/*=========================================================================
 REGISTERS
 -----------------------------------------------------------------------*/
#define MPL115A2_REGISTER_PRESSURE_MSB         (0x00)
#define MPL115A2_REGISTER_PRESSURE_LSB         (0x01)
#define MPL115A2_REGISTER_TEMP_MSB             (0x02)
#define MPL115A2_REGISTER_TEMP_LSB             (0x03)
#define MPL115A2_REGISTER_A0_COEFF_MSB         (0x04)
#define MPL115A2_REGISTER_A0_COEFF_LSB         (0x05)
#define MPL115A2_REGISTER_B1_COEFF_MSB         (0x06)
#define MPL115A2_REGISTER_B1_COEFF_LSB         (0x07)
#define MPL115A2_REGISTER_B2_COEFF_MSB         (0x08)
#define MPL115A2_REGISTER_B2_COEFF_LSB         (0x09)
#define MPL115A2_REGISTER_C12_COEFF_MSB        (0x0A)
#define MPL115A2_REGISTER_C12_COEFF_LSB        (0x0B)
#define MPL115A2_REGISTER_STARTCONVERSION      (0x12)
/*=========================================================================*/

class AP_Baro_MPL115A2: public AP_Baro {
public:
	AP_Baro_MPL115A2() {
		_mpl115a2_a0 = 0.0F;
		_mpl115a2_b1 = 0.0F;
		_mpl115a2_b2 = 0.0F;
		_mpl115a2_c12 = 0.0F;
		_pressure_samples = 1;
	}
	;       // Constructor

	/* AP_Baro public interface: */
	bool init();
	uint8_t read();
    void	accumulate(void);
	float get_pressure();
	float get_temperature();

private:

	//Semaphore has to be taken before calling this 2 functions
	void getPT(float *P, float *T);
	void readCoefficients(void);

	float _mpl115a2_a0;
	float _mpl115a2_b1;
	float _mpl115a2_b2;
	float _mpl115a2_c12;

	float           Temp;
	float           Press;
	float		    _temp_sum;
	float			_press_sum;
	uint8_t			_count;

	uint32_t        _retry_time;

};

#endif // __AP_BARO_MPL115A2_H__
