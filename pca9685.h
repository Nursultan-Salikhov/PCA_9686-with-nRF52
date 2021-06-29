#ifndef PCA9685_H__
#define PCA9685_H__
#pragma once
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include <math.h>
#include <assert.h>
#include <string.h>

#define PCA9865_I2C_DEFAULT_DEVICE_ADDRESS 0x40
#define PCA9685_ADDRESS 0x40

#define PCA9685_SCL_PIN		27
#define PCA9685_SDA_PIN		26
#define INIT_DELAY	1000

#define PCA9685_SET_BIT_MASK(BYTE, MASK)      ((BYTE) |= (uint8_t)(MASK))
#define PCA9685_CLEAR_BIT_MASK(BYTE, MASK)    ((BYTE) &= (uint8_t)(~(uint8_t)(MASK)))
#define PCA9685_READ_BIT_MASK(BYTE, MASK)     ((BYTE) & (uint8_t)(MASK))




/**
 * Registers addresses.
 */
typedef enum
{
	PCA9685_REGISTER_MODE1 = 0x00,
	PCA9685_REGISTER_MODE2 = 0x01,
	PCA9685_REGISTER_LED0_ON_L = 0x06,
	PCA9685_REGISTER_ALL_LED_ON_L = 0xfa,
	PCA9685_REGISTER_ALL_LED_ON_H = 0xfb,
	PCA9685_REGISTER_ALL_LED_OFF_L = 0xfc,
	PCA9685_REGISTER_ALL_LED_OFF_H = 0xfd,
	PCA9685_REGISTER_PRESCALER = 0xfe
} pca9685_register_t;

/**
 * Bit masks for the mode 1 register.
 */
typedef enum
{
	PCA9685_REGISTER_MODE1_SLEEP = (1u << 4u),
	PCA9685_REGISTER_MODE1_RESTART = (1u << 7u)
} pca9685_register_mode1_t;
      
typedef struct {

	/**
	 * The handle to the TWI bus for the device.
	 */
	const nrf_drv_twi_t  twi_handle;

	/**
	 * The I2C device address.
	 * @see{PCA9865_I2C_DEFAULT_DEVICE_ADDRESS}
	 */
	uint16_t device_address;

	/**
	 * Set to true to drive inverted.
	 */
	_Bool inverted;

} pca9685_handle_t;

extern const nrf_drv_twi_t m_twi;

extern pca9685_handle_t m_pcah;
      
void pca9685_init(pca9685_handle_t *handle); // init function

void pca9685_set_pwm_frequency( float frequency); // install PWM frequency

void pca9685_set_channel_pwm_times(pca9685_handle_t *m_handle,uint8_t channel, uint16_t on_time, uint16_t off_time);// set channel


#endif // - PCA9685_H__ - //