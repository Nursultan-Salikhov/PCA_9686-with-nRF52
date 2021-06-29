#include "pca9685.h"


 /**
 * @brief Transmit one byte.
 */     
void pca9685_write_u8 (uint8_t reg,uint8_t val)
{
  uint8_t buff[2] = {reg,val};
    nrf_drv_twi_tx(&m_pcah.twi_handle,m_pcah.device_address,buff, 2, false);
}

void pca9685_write_data (pca9685_handle_t *m_handle, uint8_t reg, uint8_t *data, size_t length)
{
  uint8_t buff[length+1];
  buff[0] = reg;
  memcpy(&buff[1], data, length);
  nrf_drv_twi_tx(&m_handle->twi_handle,m_handle->device_address,buff, sizeof(buff), false);
}
  
void pca9685_read_u8( uint8_t address, uint8_t *dest)
{
nrf_drv_twi_tx(&m_pcah.twi_handle,m_pcah.device_address,address, 1, false);
nrf_delay_us(1000);
	nrf_drv_twi_rx(&m_pcah.twi_handle,m_pcah.device_address, dest,1);
}
  
  
  
  
  
void pca9685_is_sleeping( bool *sleeping)
{
	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
        pca9685_read_u8( PCA9685_REGISTER_MODE1, &mode1_reg);

	// Check if the sleeping bit is set.
	*sleeping = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);
}
  
void pca9685_sleep(void)
{
	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
	 pca9685_read_u8( PCA9685_REGISTER_MODE1, &mode1_reg);

	// Don't write the restart bit back and set the sleep bit.
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);
	 pca9685_write_u8( PCA9685_REGISTER_MODE1, mode1_reg);

}
 
 
 
void pca9685_wakeup(void)
{
	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
        pca9685_read_u8( PCA9685_REGISTER_MODE1, &mode1_reg);

	bool restart_required = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);

	// Clear the restart bit for now and clear the sleep bit.
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);
	 pca9685_write_u8( PCA9685_REGISTER_MODE1, mode1_reg);

	if (restart_required) {

		// Oscillator requires at least 500us to stabilise, so wait 1ms.
		nrf_delay_us(10000);

		PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
		 pca9685_write_u8( PCA9685_REGISTER_MODE1, mode1_reg);
	}
}
      
      
  
void pca9685_set_pwm_frequency( float frequency)
{


	

	// Calculate the prescaler value (see datasheet page 25)
	uint8_t prescaler = (uint8_t)roundf(25000000.0f / (4096 * frequency)) - 1;

	bool already_sleeping;
	pca9685_is_sleeping( &already_sleeping);

	// The prescaler can only be changed in sleep mode.
	if (!already_sleeping) {
	  pca9685_sleep();
	}

	// Write the new prescaler value.
	 pca9685_write_u8(PCA9685_REGISTER_PRESCALER, prescaler);

	// If the device wasn't sleeping, return from sleep mode.
	if (!already_sleeping) {
	 pca9685_wakeup();
	}

	
}
 
 

 
      
void pca9685_init (pca9685_handle_t *handle)
{
	uint8_t mode1_reg_default_value = 0b00110000u;
	uint8_t mode2_reg_default_value = 0b00000100u;
	
	//if (handle->inverted) {
	//	mode2_reg_default_value |= 0b00010000u;
	//}



	 pca9685_write_u8(PCA9685_REGISTER_MODE1, mode1_reg_default_value);
	  nrf_delay_us(INIT_DELAY);
	pca9685_write_u8( PCA9685_REGISTER_MODE2, mode2_reg_default_value);
	 nrf_delay_us(INIT_DELAY);

    // Turn all channels off to begin with.
    uint8_t data[4] = { 0x00, 0x00, 0x00, 0x10 };
     pca9685_write_data(handle, PCA9685_REGISTER_ALL_LED_ON_L, data, 4);
     nrf_delay_us(INIT_DELAY);

	 pca9685_set_pwm_frequency(1000);
	  nrf_delay_us(INIT_DELAY);
	 pca9685_wakeup();
	  nrf_delay_us(INIT_DELAY);
}
 
 
 void pca9685_set_channel_pwm_times(pca9685_handle_t *m_handle, uint8_t channel, uint16_t on_time, uint16_t off_time)
{
	uint8_t data[4] = { on_time, on_time >> 8u, off_time, off_time >> 8u };
	pca9685_write_data(m_handle, PCA9685_REGISTER_LED0_ON_L + channel * 4, data, 4);
} 
