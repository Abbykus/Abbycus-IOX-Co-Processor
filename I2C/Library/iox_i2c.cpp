/********************************************************************
 * @file iox_i2c.cpp
 * @author J.Hoeppner @ ABBYKUS
 * @brief 
 * @version 1.0
 * @date 2022-09-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "iox_i2c.h"

/********************************************************************
 * @brief Construct a new IOX_I2C class object
 * 
 */
IOX_I2C::IOX_I2C(void)
{
   _wire = NULL;
}


/********************************************************************
 * @brief Initialize the master I2C controller.
 * 
 * @param sda - I2C data pin
 * @param sck - I2C clock pin
 * @param bus_freq - I2C std speed = 100000U, fast mode = 400000U, fast 
 *       mode plus = 800000U. Default = 100000U (std speed).
 * @param wire - ptr to I2C controller. Default is 'Wire' 
 * @return true if successful
 */
bool IOX_I2C::init(uint8_t sda, uint8_t sck, uint32_t bus_freq, TwoWire *wire)
{
   _wire = wire;
   return _wire->begin(sda, sck, bus_freq);
}


/********************************************************************
 * @brief Who Am I checks the I2C bus connection. The slave device is 
 *       expected to return it's own address (0x5D or 0x5E).
 * @return ERR_NONE if connection is valid.
 */
uint8_t IOX_I2C::whoAmI(uint8_t iox_adrs)   // connection test
{
   uint8_t ret = i2c_write(iox_adrs, WHO_AM_I, NULL, 0, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 2);
      if(ret == 2)                     // read two bytes from the iox device
      {
         if(_rxbufr[0] != get_iox_adrs(iox_adrs))   // validate connection 
            ret = ERR_GP_FAILURE;
         else 
            ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Error Check returns a STATUS byte and an ERROR word reflecting
 *       the status from the last operation.
 * @param sys_status - pointer to a SYS_STATUS structure - see iox_i2c.h
 * 
 * @return ERR_NONE if successful. Otherwise see error codes in iox_i2c.h 
 */
uint8_t IOX_I2C::read_status(uint8_t iox_adrs, SYS_STATUS *sys_status, uint8_t field_mask)  // connection test
{
   int16_t _field;
   uint8_t ret;

   _txbufr[0] = field_mask;
   ret = i2c_write(iox_adrs, READ_STATUS, (uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 4);
      if(ret == 4)                              // read 4 bytes from the iox device
      {
         sys_status->status = _rxbufr[0];       // status in high byte, error code in low byte
         _field = _rxbufr[1];
         _field = (_field << 8) | _rxbufr[2];
         sys_status->field = _field;
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Configure one or more gpio pins with the same mode info.
 * 
 * @param iox_adrs - IOX device 0 or 1.
 * @param gpio_map - 20 bit map, a '1' flags the GPIO to be configured.
 * @param io_mode - Encoded mode, type, pullup, speed, and exti enable.
 * 
 * @return ERR_NONE if successful.
 */
uint8_t IOX_I2C::config_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint8_t io_mode)
{
   uint8_t ret;

   _txbufr[0] = (gpio_map >> 16);    // msb of gpio_map word
   _txbufr[1] = (gpio_map >> 8);    // msb of gpio_map word
   _txbufr[2] = (gpio_map & 0xFF);  // lsb of gpio_map word
   _txbufr[3] = io_mode;

   ret = i2c_write(iox_adrs, CONFIG_GPIOS, (uint8_t *)&_txbufr, 4, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}



/********************************************************************
 * @brief Return the IO mode for the specified GPIO.
 * 
 * @param gpio_num - Logical GPIO 0 - 19
 * @return GPIO mode encoded as follows:
 * bit: 7 - 6  	5     4     3     2     1    0
 *	     unused   mode  |---attr1 ---|    |attr2|
 *
 *	mode	attr1	meaning
 *	0		000	low power mode
 *	0		001	Input, floating
 *	0		010	Input, pullup
 *	0		100	Input, pulldown
 *	1		000	Output, Push-Pull, no pullup / pulldown
 *	1		001	Output, Push-Pull, pullup
 *	1		010	Output, Push-Pull, pulldown
 *	1		100	Output, Open-Drain, no pullup / pulldown
 *	1		101	Output, Open-Drain, pullup
 *	1		110	Output, Open-Drain, pulldown
 *
 *	attr2 (if mode == Output)
 *	00		Low power - 2 MHz
 *	01		Medium power - 10 MHz
 *	1x		High power - 50 MHz
 * 
 * attr2 (if mode == Input)
 *	00		EXTI disabled
 *	01		ETXI enabled, rising edge trigger
 *	10		ETXI enabled, falling edge trigger
 * 11    ETXI enabled, rising & falling edge trigger
 */
uint8_t IOX_I2C::get_gpio_config(uint8_t iox_adrs, uint8_t gpio_num)
{
   uint8_t ret = 0;
 
   _txbufr[0] = gpio_num;        // param to send to slave

   ret = i2c_write(iox_adrs, GET_GPIO_CONFIG, (uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 2);
      if(ret == 2)               // read 2 bytes from the iox device
      {
         ret = _rxbufr[0];       // get encoded GPIO mode
      }
      else 
         ret = 0xFF;
   }
   else 
      ret = 0xFF;
   return ret;
}


/********************************************************************
 * @brief Write one or more gpio's with the same state.
 * 
 * @param start - GPIO number to start range. Valid is 0 - 19.
 * @param end - GPIO number to end range. Valid is 'start' - 19.
 * @param state - a 1 or 0 to write to the gpio.
 * @return 0 or error code if bad parameter or bus error.
 */
uint8_t IOX_I2C::write_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint32_t state_map)
{
   uint16_t wrgpio = 0;
   uint8_t ret = ERR_NONE;

   _txbufr[0] = gpio_map >> 16;     // msb first
   _txbufr[1] = gpio_map >> 8;   
   _txbufr[2] = gpio_map & 0xFF;
   _txbufr[3] = state_map >> 16;     // msb first
   _txbufr[4] = state_map >> 8;   
   _txbufr[5] = state_map & 0xFF;
   
   ret = i2c_write(iox_adrs, WRITE_GPIOS, (uint8_t *)&_txbufr, 6, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Toggle the output state of a range of GPIO's.
 * 
 * @param iox_adrs - IOX device address 0 or 1
 * @param gpio_map - 20 bits, a '1' toggles an output
 * @return ERR_NONE if successful
 */
uint8_t IOX_I2C::toggle_gpios(uint8_t iox_adrs, uint32_t _gpio_map)
{
   uint8_t ret;

   _txbufr[0] = _gpio_map >> 16;      // msb first
   _txbufr[1] = _gpio_map >> 8;
   _txbufr[2] = _gpio_map & 0xFF;

   ret = i2c_write(iox_adrs, TOGGLE_GPIOS, (uint8_t *)&_txbufr, 3, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read the state of the specified GPIO
 * 
 * @param iox_adrs - IOX device number 0 or 1
 * @param gpio_num - 0-19
 * @param ret_value is a pointer to a memory location where the return value
 *       is written.
 * @return uint8_t - ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_I2C::read_gpio(uint8_t iox_adrs, uint8_t gpio_num, uint8_t *ret_value)   // connection test
{
   uint8_t ret = 0;

   _txbufr[0] = gpio_num;

   ret = i2c_write(iox_adrs, READ_GPIO,(uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 2);
      if(ret == 1)                        // read one byte from the slave
      {
         *ret_value = _rxbufr[0];         // get GPIO state
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read all inputs and return state as a 20 bit field.
 * 
 * @param gpio_map - pointer to a 32 bit variable where the 20 
 *       bit map will be written.
 * @return uint8_t - ERR_NONE if succesful, otherwise an error code.
 */
uint8_t IOX_I2C::read_gpio_all(uint8_t iox_adrs, uint32_t *gpio_map)
{
   uint32_t _gpio_map = 0;
   uint8_t ret;

   if(gpio_map == NULL)
      return ERR_PARAM;

   ret = i2c_write(iox_adrs, READ_GPIO_ALL, NULL, 0, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 4);
      if(ret == 4)                             // read three bytes from the slave
      {
         _gpio_map = _rxbufr[0];              // 20 bit field MSB first
         _gpio_map = (_gpio_map << 8) | _rxbufr[1];
         _gpio_map = (_gpio_map << 8) | _rxbufr[2];
         *gpio_map = _gpio_map;
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Configure a GPIO to be used as an interrupt alert output pin.
 * 
 * @param event_type - type of event to trigger output. Values can be:
 *          EVENT_EXTI, EVENT_ADC, EVENT_CAPTURE, or EVENT_ENCODER.
 * @param event_io - logical gpio output. Valid values are GPIO16 - GPIO19.
 * @note - Any event_io values that are not valid (i.e. 0) will disable the 
 *       event function.
 */
uint8_t IOX_I2C::config_event_output(uint8_t iox_adrs, uint8_t event_type, uint8_t event_io)
{
   uint8_t ret;

   _txbufr[0] = event_type;
   _txbufr[1] = event_io;      // msb first

   ret = i2c_write(iox_adrs, CONFIG_EXTI_EVENT, (uint8_t *)&_txbufr, 2, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}   


/********************************************************************
 * @brief Configure & Start a PWM output.
 * @note: There are 10 output pins that can function as PWM outputs.
 * 
 * @param pwm_num - PWM output. Valid numbers are 0 - 9.
 * @param clk_div - 16 bit value used to set the PWM clock prescale.
 * @param pwm_freq - PWM period frequency = clock speed / pwm_freq
 * @param pwm_duty - PWM duty cycle (on time) is ratio of pwm_freq / pwm_duty
 * @param polarity - 0 = duty cycle is high. 1 = inverse
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_I2C::start_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t clk_div, uint16_t pwm_freq, uint16_t pwm_duty, bool polarity)
{
   uint8_t ret;

   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[1] = (clk_div >> 8);
   _txbufr[2] = (clk_div & 0xFF);   
   _txbufr[3] = (pwm_freq >> 8);
   _txbufr[4] = (pwm_freq & 0xFF); 
   _txbufr[5] = (pwm_duty >> 8);
   _txbufr[6] = (pwm_duty & 0xFF); 
   _txbufr[7] = polarity; 

   ret = i2c_write(iox_adrs, START_PWM, (uint8_t *)&_txbufr, 8, false);    
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret; 
}


/********************************************************************
 * @brief Update a configured PWM output.
 * 
 * @param pwm_num - PWM output to update
 * @param pwm_duty - 16 bit duty cycle value
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_I2C::update_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t pwm_duty)
{
   uint8_t ret;
   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the new PWM duty cycle value **/
   _txbufr[0] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[1] = (pwm_duty >> 8);    
   _txbufr[2] = (pwm_duty & 0xFF);

   ret = i2c_write(iox_adrs, UPDATE_PWM, (uint8_t *)&_txbufr, 3, false);   
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   } 
   return ret;    
}


/********************************************************************
 * @brief Configure one or more ADC channels. This funtion set the ADC
 *       resolution and configures gpio's for analog input.
 * 
 * @param adc_chnls - bit-wise map of channels to configure as ADC input.
 *       Each bit from 0 - 9 represents a logical channel.
 * @param adc_resol - 6, 8, 10, or 12 bit resolution applies to all channels
 * @return true if successful
 */
uint8_t IOX_I2C::config_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint8_t adc_resol)
{
  uint8_t ret;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = (adc_chnls >> 8);         // adc channel map MSB
   _txbufr[1] = (adc_chnls & 0xFF);
   _txbufr[2] = adc_resol;

   ret = i2c_write(iox_adrs, CONFIG_ADC, (uint8_t *)&_txbufr, 3, false);  
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }  
   return ret;       
}

      
/********************************************************************
 * @brief Start an ADC conversion on one or more channels with the 
 *       specified number os samples.
 * 
 * @param adc_chnls - bitwise map of channels to convert
 * @param num_samples - number of samples to average over.
 * @return true if successful.
 */
uint8_t IOX_I2C::start_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint16_t num_samples)
{
   uint8_t ret;
   uint16_t adcval = 0;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = (adc_chnls >> 8);               // adc chnl map
   _txbufr[1] = (adc_chnls & 0xFF);
   _txbufr[2] = (num_samples >> 8);
   _txbufr[3] = (num_samples & 0xFF);

   ret = i2c_write(iox_adrs, START_ADC, (uint8_t *)&_txbufr, 4, false);  
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   } 
   return ret; 
}


/********************************************************************
 * @brief Read result of one ADC channel (0 - 9).
 * 
 * @param adc_num - 0 - 9
 * @param adc_conv - pointer to an ADC_CONVERT structure - see iox_i2c.h
 * @return ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_I2C::read_ADC(uint8_t iox_adrs, uint8_t adc_num, ADC_CONVERT *adc_conv)
{
   uint8_t ret;
   uint16_t adcval = 0;             // error return value

   if(adc_num > 9 || adc_conv == NULL)
      return ERR_PARAM;             // validate adc number range 0 - 9

   /** load the _txbufr with the adc channel number **/
   _txbufr[0] = adc_num;            // adc chnl 0 - 9
   ret = i2c_write(iox_adrs, READ_ADC, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 9);
      if(ret == 9)           // read 9 bytes from the IOX
      {
         adcval = _rxbufr[0];       // get mean value
         adcval = (adcval << 8) | _rxbufr[1];
         adc_conv->mean = adcval;

         adcval = _rxbufr[2];       // get min value
         adcval = (adcval << 8) | _rxbufr[3];
         adc_conv->min = adcval;

         adcval = _rxbufr[4];       // get max value
         adcval = (adcval << 8) | _rxbufr[5];
         adc_conv->max = adcval;     

         adcval = _rxbufr[6];       // get num samples
         adcval = (adcval << 8) | _rxbufr[7];
         adc_conv->samples = adcval;   

         /** last byte read is the status **/
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Helper function to convert first bit-wise ADC channel map 
 *       to its logical number.
 * 
 * @param adc_chnls 
 * @return int8_t - logical adc chnl (0 - 9) or -1 if adc_chnls == 0.
 */
int8_t IOX_I2C::find_first_adc(uint16_t adc_chnls)
{
   uint8_t i;
   uint16_t j = 0x1;
   int8_t ret = 0;

   if(adc_chnls == 0)
      return -1;

   for(i=0; i<NUM_ADC_CHNLS; i++)
   {
      if((adc_chnls & j) > 0)
      {
         break;
      }
      j = j << 1;
      ret++;
   }
   return ret;
}


/********************************************************************
 * @brief Start an Input Capture measurement
 * 
 * @param capt_num - Capture channel number (0-9)
 * @param trig_edge - INPUT_CAPT_RISING, INPUT_CAPT_FALLING, or INPUT_CAPT_BOTH
 * @param cap_type - CAPTURE_TYPE_FREQ or CAPTURE_TYPE_PW
 */
uint8_t IOX_I2C::start_capture(uint8_t iox_adrs, uint8_t capt_num, uint8_t trig_edge, uint8_t capt_type)
{
   uint8_t ret;
   if(capt_num > 9)
      return ERR_PARAM;

   _txbufr[0] = capt_num;            // capture chnl 0 - 9
   _txbufr[1] = trig_edge; 
   _txbufr[2] = capt_type;

   ret = i2c_write(iox_adrs, START_CAPTURE, (uint8_t *)&_txbufr, 3, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read capture measurement.
 * 
 * @param capt_num - capture channel logical number (0-9)
 * @return uint32_t - 32 bit capture measurement
 */
uint32_t IOX_I2C::read_capture(uint8_t iox_adrs, uint8_t capt_num)
{
   uint8_t ret;
   uint32_t captval = 0;

   if(capt_num > 9)
      return 0;

   _txbufr[0] = capt_num;
   ret = i2c_write(iox_adrs, READ_CAPTURE, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      if(i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 5) == 5)      // read 5 bytes from the IOX
      {
         captval = _rxbufr[0];       // capture measurement value 
         captval = (captval << 8) | _rxbufr[1];
         captval = (captval << 8) | _rxbufr[2];
         captval = (captval << 8) | _rxbufr[3];
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return captval;
}


/********************************************************************
 * @brief Configure a pair of inputs for encoder function
 * 
 * @param enc_num - logical encoder number (0 - 7)
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_I2C::config_encoder(uint8_t iox_adrs, uint8_t enc_num)
{
   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = enc_num;
   uint8_t ret = i2c_write(iox_adrs, CONFIG_ENCODER, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Return last encoder values.
 * 
 * @param enc_num - logical encoder number (0 - 7)
 * @param rot_enc - pointer to a ROT_ENC struct
 * @return ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_I2C::read_encoder(uint8_t iox_adrs, uint8_t enc_num, ROT_ENC *rot_enc)
{
   uint8_t ret;

   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = enc_num;

   ret = i2c_write(iox_adrs, READ_ENCODER, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 6);
      if(ret == 6)     // read two bytes from the IOX
      {
         rot_enc->dir = _rxbufr[0];
         rot_enc->count = _rxbufr[1];
         rot_enc->count = (rot_enc->count << 8) | _rxbufr[2];
         rot_enc->speed = _rxbufr[3];
         rot_enc->speed = (rot_enc->speed << 8) | _rxbufr[4];
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Enter sleep mode. Wake on GPIO level change.
 * 
 * @param wake_gpio - gpio number to use as wake up.
 * @return uint8_t - ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_I2C::sleep(uint8_t iox_adrs, uint8_t wake_gpio)
{
   _txbufr[0] = wake_gpio;
   uint8_t ret = i2c_write(iox_adrs, SLEEP, (uint8_t *)&_txbufr, 1, true);     // send wakeup gpio
   // if(ret == ERR_NONE)
   // {
   //    ret = i2c_read(iox_adrs, (uint8_t *)&_rxbufr, 1);
   //    ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   // }
   return ret;
}


/********************************************************************
 * @brief I2C helper functions **************************************
 *******************************************************************/

uint8_t IOX_I2C::get_iox_adrs(uint8_t iox_adrs)    // 0 or 1
{
   uint8_t ret = (iox_adrs == 0) ? 0x5D : 0x5E;
   return ret;
}


/********************************************************************
 * @brief i2c_write - send bytes to slave register.
 * 
 * @param regnum - register number
 * @param databuf - ptr to xmit data buffer
 * @param bytes2send - number of bytes to send (not including the register num)
 * @param end_comm - true if no bytes to be received from slave (write only transfer)
 * @return ERR_NONE if successful, else failure error code
 */
uint8_t IOX_I2C::i2c_write(uint8_t iox_adrs, uint8_t regnum, uint8_t *databuf, uint8_t bytes2send, bool write_only)
{
   uint8_t i;
   uint8_t ret = ERR_NONE;

   if(databuf == NULL && bytes2send > 0)     // validate buffer
   {
      return ERR_PARAM;
   }

   _wire->beginTransmission(get_iox_adrs(iox_adrs));      // transmit to device ABBY-IOX
   _wire->write(regnum);                     // send mandatory register address
   for(i=0; i<bytes2send; i++)
   {
      _wire->write(*databuf++);              // MSB first
   }
   ret = _wire->endTransmission(write_only);
   return  ret;      // end transmit, if write_only true                      
}


/********************************************************************
 * @brief i2c_read - read bytes from slave. This function follows a 
 * call to i2c_write with write_only = false. This creates a Restart
 * condition to switch from slave read to slave write.
 * 
 * @param rcvbuf - ptr to rcv data bufr
 * @param numbytes - number of expected bytes
 * @return number of bytes read from IOX device. Returns 0 on error.
 *       
 */
uint8_t IOX_I2C::i2c_read(uint8_t iox_adrs, uint8_t *rcvbuf, uint8_t numbytes)
{
   uint8_t bytes_read;
   uint8_t i;
   uint8_t ret = ERR_NONE;
   uint32_t timeout;
   if(numbytes > 0)                          // validate read request
   {
      ret = _wire->requestFrom(get_iox_adrs(iox_adrs), numbytes);   
      bytes_read = _wire->available();       // get # bytes sent from slave
      if(bytes_read > 0)
      {
         if(bytes_read > MAX_BUFR_SIZE)
            bytes_read = MAX_BUFR_SIZE -1;

         for(i=0; i<bytes_read; i++)
         {
            *rcvbuf++ = _wire->read();
         }
         if(_wire->endTransmission(true) == 0)  // end of bus transaction
            ret = bytes_read;
         else 
            ret = 0;
      }
      else 
         ret = 0;       // timeout error
   }
   return ret;
}