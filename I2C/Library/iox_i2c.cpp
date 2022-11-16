/********************************************************************
 * @file iox_i2c.cpp
 * @author J.Hoeppner @ Abbycus
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
 * @param slave_adr - address of the ABBY-IOX I2C slave device.
 *       Default address = 0x5D if pin ADR0 is low or floating. 0x5E
 *       otherwise.
 */
IOX_I2C::IOX_I2C(uint8_t slave_adr)
{
   _i2c_adrs = slave_adr;
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
uint8_t IOX_I2C::whoAmI(void)   // connection test
{
   uint8_t ret = i2c_write(WHO_AM_I, NULL, 0, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 1);
      if(ret == ERR_NONE)            // read one byte from the iox device
      {
         if(_rxbufr[0] != _i2c_adrs)    // validate connection 
            ret = ERR_GP_FAILURE;
      }
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
uint8_t IOX_I2C::read_status(SYS_STATUS *sys_status, uint8_t field_mask)  // connection test
{
   int16_t _field;
   uint8_t ret;

   _txbufr[0] = field_mask;
   ret = i2c_write(READ_STATUS, (uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 3);
      if(ret == ERR_NONE)      // read one byte from the slave
      {
         sys_status->status = _rxbufr[0];       // status in high byte, error code in low byte
         _field = _rxbufr[1];
         _field = (_field << 8) | _rxbufr[2];
         sys_status->field = _field;
      }
   }
   return ret;
}


/********************************************************************
 * @brief Configure one or more gpio pins with the same mode info.
 * 
 * @param start - GPIO number to start range. Valid is 0 - 19.
 * @param end - GPIO number to end range. Valid is 'start' - 19.
 * @param io_mode - Encoded mode, type, pullup, speed, and exti enable.
 * @example 
 *    config_gpios(0, 3, IO_OUTPUT_PP | IO_SPEED_MED)
 *    config_gpios(7, 19, IO_INPUT | IO_PULLUP | IO_EXTI_RISING);
 * 
 * @return 0 if successful, error code otherwise.
 */
uint8_t IOX_I2C::config_gpios(uint8_t start, uint8_t end, uint8_t io_mode)
{
   uint8_t err = ERR_NONE;
   uint16_t ioset = 0;

   /** range check start & end values **/
   if(end < start || start > 19 || end > 19)
      return ERR_PARAM;

   ioset = (start << 11);
   ioset |= (end << 6);
   ioset |= io_mode;

   _txbufr[0] = (ioset >> 8);    // msb of ioset word
   _txbufr[1] = (ioset & 0xFF);  // lsb of ioset word

   err = i2c_write(CONFIG_GPIOS, (uint8_t *)&_txbufr, 2, true);
   return err;
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
uint8_t IOX_I2C::get_gpio_config(uint8_t gpio_num)
{
   uint8_t ret = 0;
 
   _txbufr[0] = gpio_num;        // param to send to slave

   ret = i2c_write(GET_GPIO_CONFIG, (uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 1);
      if(ret == ERR_NONE)      // read one byte from the slave
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
uint8_t IOX_I2C::write_gpios(uint8_t start, uint8_t end, uint8_t state)
{
   uint16_t wrgpio = 0;
   uint8_t ret = ERR_NONE;

   if(end < start || start > 19 || end > 19)
      return ERR_PARAM;

   wrgpio = (start << 11);
   wrgpio |= (end << 6);
   wrgpio |= (state & 0x01);     // mask lsb
   _txbufr[0] = wrgpio >> 8;     // msb first
   _txbufr[1] = wrgpio & 0xFF;   // then lsb

   ret = i2c_write(WRITE_GPIOS, (uint8_t *)&_txbufr, 2, true);
   return ret;
}


/********************************************************************
 * @brief Write all gpios from a 20 bit field. GPIO's that are not set as 
 *       outputs are ignored. This function allows all gpio's to be 
 *       set/reset in parallel.
 * 
 * @param gpio_bits - GPIO's 0-19.
 * @return true if successful
 */
uint8_t IOX_I2C::write_gpios_all(uint32_t gpio_bits)
{
   uint8_t err;

   _txbufr[0] = (gpio_bits >> 16);    // xmit bufr MSB first
   _txbufr[1] = (gpio_bits >> 8) & 0x000000FF;
   _txbufr[2] = gpio_bits & 0x000000FF;

   err = i2c_write(WRITE_GPIOS_ALL, (uint8_t *)&_txbufr, 3, true);
   return err;
}


/********************************************************************
 * @brief Toggle the output state of a range of GPIO's.
 * 
 * @param start - starting GPIO number
 * @param end - ending GPIO number
 * @return true if operation is successful
 */
uint8_t IOX_I2C::toggle_gpios(uint8_t start, uint8_t end)
{
   uint16_t gpios;
   uint8_t ret;

   if(end < start || start > 19 || end > 19)
      return ERR_PARAM;

   gpios = (start << 11);
   gpios |= (end << 6);
   _txbufr[0] = gpios >> 8;      // msb first
   _txbufr[1] = gpios & 0xFF;
   ret = i2c_write(TOGGLE_GPIOS, (uint8_t *)&_txbufr, 2, true);

   return ret;
}


/********************************************************************
 * @brief Read the state of the specified GPIO
 * 
 * @param gpio - 0-19
 * @param value is a pointer to a memory location where the return value
 *       is written.
 * @return uint8_t - ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_I2C::read_gpio(uint8_t gpio_num, uint8_t *ret_value)   // connection test
{
   uint8_t ret = 0;

   _txbufr[0] = gpio_num;

   ret = i2c_write(READ_GPIO,(uint8_t *)&_txbufr, 1, false);
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 1);
      if(ret == ERR_NONE)                 // read one byte from the slave
      {
         *ret_value = _rxbufr[0];         // get GPIO state
      }
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
uint8_t IOX_I2C::read_gpio_all(uint32_t *gpio_map)
{
   uint32_t _gpio_map = 0;
   uint8_t ret;

   if(gpio_map == NULL)
      return ERR_PARAM;

   ret = i2c_write(READ_GPIO_ALL, NULL, 0, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 3);
      if(ret == ERR_NONE)                             // read three bytes from the slave
      {
         _gpio_map = _rxbufr[0];              // 20 bit field MSB first
         _gpio_map = (_gpio_map << 8) | _rxbufr[1];
         _gpio_map = (_gpio_map << 8) | _rxbufr[2];
         *gpio_map = _gpio_map;
      }
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
uint8_t IOX_I2C::config_event_output(uint8_t event_type, uint8_t event_io)
{
   uint8_t ret;

   _txbufr[0] = event_type;
   _txbufr[1] = event_io;      // msb first

   ret = i2c_write(CONFIG_EXTI_EVENT, (uint8_t *)&_txbufr, 2, true);
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
uint8_t IOX_I2C::start_PWM(uint8_t pwm_num, uint16_t clk_div, uint16_t pwm_freq, uint16_t pwm_duty, bool polarity)
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

   ret = i2c_write(START_PWM, (uint8_t *)&_txbufr, 8, true);    
   return ret; 
}


/********************************************************************
 * @brief Update a configured PWM output.
 * 
 * @param pwm_num - PWM output to update
 * @param pwm_duty - 16 bit duty cycle value
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_I2C::update_PWM(uint8_t pwm_num, uint16_t pwm_duty)
{
   uint8_t ret;
   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the new PWM duty cycle value **/
   _txbufr[0] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[1] = (pwm_duty >> 8);    
   _txbufr[2] = (pwm_duty & 0xFF);

   ret = i2c_write(UPDATE_PWM, (uint8_t *)&_txbufr, 3, true);    
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
uint8_t IOX_I2C::config_ADC(uint16_t adc_chnls, uint8_t adc_resol)
{
  uint8_t ret;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = (adc_chnls >> 8);         // adc channel map MSB
   _txbufr[1] = (adc_chnls & 0xFF);
   _txbufr[2] = adc_resol;

   ret = i2c_write(CONFIG_ADC, (uint8_t *)&_txbufr, 3, true);    
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
uint8_t IOX_I2C::start_ADC(uint16_t adc_chnls, uint16_t num_samples)
{
   uint8_t ret;
   uint16_t adcval = 0;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = (adc_chnls >> 8);               // adc chnl map
   _txbufr[1] = (adc_chnls & 0xFF);
   _txbufr[2] = (num_samples >> 8);
   _txbufr[3] = (num_samples & 0xFF);

   ret = i2c_write(START_ADC, (uint8_t *)&_txbufr, 4, true);   
   return ret; 
}


/********************************************************************
 * @brief Read result of one ADC channel (0 - 9).
 * 
 * @param adc_num - 0 - 9
 * @param adc_conv - pointer to an ADC_CONVERT structure - see iox_i2c.h
 * @return ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_I2C::read_ADC(uint8_t adc_num, ADC_CONVERT *adc_conv)
{
   uint8_t ret;
   uint16_t adcval = 0;             // error return value

   if(adc_num > 9 || adc_conv == NULL)
      return ERR_PARAM;             // validate adc number range 0 - 9

   /** load the _txbufr with the adc channel number **/
   _txbufr[0] = adc_num;            // adc chnl 0 - 9
   ret = i2c_write(READ_ADC, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 8);
      if(ret == ERR_NONE)           // read 10 bytes from the IOX
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

      }
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
uint8_t IOX_I2C::start_capture(uint8_t capt_num, uint8_t trig_edge, uint8_t capt_type)
{
   uint8_t ret;
   if(capt_num > 9)
      return ERR_PARAM;

   _txbufr[0] = capt_num;            // capture chnl 0 - 9
   _txbufr[1] = trig_edge; 
   _txbufr[2] = capt_type;

   ret = i2c_write(START_CAPTURE, (uint8_t *)&_txbufr, 3, true); 
   return ret;
}


/********************************************************************
 * @brief Read capture measurement.
 * 
 * @param capt_num - capture channel logical number (0-9)
 * @return uint32_t - 32 bit capture measurement
 */
uint32_t IOX_I2C::read_capture(uint8_t capt_num)
{
   uint8_t ret;
   uint32_t captval = 0;

   if(capt_num > 9)
      return 0;

   _txbufr[0] = capt_num;
   ret = i2c_write(READ_CAPTURE, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      if(i2c_read((uint8_t *)&_rxbufr, 4) == ERR_NONE)      // read 4 bytes from the IOX
      {
         captval = _rxbufr[0];       // capture measurement value 
         captval = (captval << 8) | _rxbufr[1];
         captval = (captval << 8) | _rxbufr[2];
         captval = (captval << 8) | _rxbufr[3];
      }
   }
   return captval;
}


/********************************************************************
 * @brief Configure a pair of inputs for encoder function
 * 
 * @param enc_num - logical encoder number (0 - 7)
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_I2C::config_encoder(uint8_t enc_num)
{
   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = enc_num;
   return i2c_write(CONFIG_ENCODER, (uint8_t *)&_txbufr, 1, true); 
}


/********************************************************************
 * @brief Return last encoder values.
 * 
 * @param enc_num - logical encoder number (0 - 7)
 * @param rot_enc - pointer to a ROT_ENC struct
 * @return ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_I2C::read_encoder(uint8_t enc_num, ROT_ENC * rot_enc)
{
   uint8_t ret;

   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = enc_num;

   ret = i2c_write(READ_ENCODER, (uint8_t *)&_txbufr, 1, false); 
   if(ret == ERR_NONE)
   {
      ret = i2c_read((uint8_t *)&_rxbufr, 5);
      if(ret == ERR_NONE)     // read two bytes from the IOX
      {
         rot_enc->dir = _rxbufr[0];
         rot_enc->count = _rxbufr[1];
         rot_enc->count = (rot_enc->count << 8) | _rxbufr[2];
         rot_enc->speed = _rxbufr[3];
         rot_enc->speed = (rot_enc->speed << 8) | _rxbufr[4];
      }
   }
   return ret;
}


/********************************************************************
 * @brief Enter sleep mode. Wake on GPIO level change.
 * 
 * @param wake_gpio - gpio number to use as wake up.
 * @return uint8_t - ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_I2C::sleep(uint8_t wake_gpio)
{
   _txbufr[0] = wake_gpio;
   uint8_t ret = i2c_write(SLEEP, (uint8_t *)&_txbufr, 1, true);     // send wakeup gpio
   return ret;
}


/********************************************************************
 * @brief I2C helper functions **************************************
 *******************************************************************/

/********************************************************************
 * @brief i2c_write - send bytes to slave register.
 * 
 * @param regnum - register number
 * @param databuf - ptr to xmit data buffer
 * @param bytes2send - number of bytes to send (not including the register num)
 * @param end_comm - true if no bytes to be received from slave (write only transfer)
 * @return ERR_NONE if successful, else failure error code
 */
uint8_t IOX_I2C::i2c_write(uint8_t regnum, uint8_t *databuf, uint8_t bytes2send, bool write_only)
{
   uint8_t i;
   uint8_t ret = ERR_NONE;

   if(databuf == NULL && bytes2send > 0)     // validate buffer
   {
      return ERR_PARAM;
   }
   _wire->beginTransmission(_i2c_adrs);      // transmit to device ABBY-IOX
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
 * @return ERR_NONE if read is successful. If not error code =
 *       
 */
uint8_t IOX_I2C::i2c_read(uint8_t *rcvbuf, uint8_t numbytes)
{
   uint8_t num_bytes_avail;
   uint8_t i;
   uint8_t ret = ERR_NONE;
   if(numbytes > 0)
   {
      ret = _wire->requestFrom(_i2c_adrs, numbytes);   
      num_bytes_avail = _wire->available();     // get # bytes sent from slave
      if(num_bytes_avail > 0)
      {
         if(num_bytes_avail > MAX_BUFR_SIZE)
            num_bytes_avail = MAX_BUFR_SIZE;

         for(i=0; i<num_bytes_avail; i++)
         {
            *rcvbuf++ = _wire->read();
         }
         ret = _wire->endTransmission(true);          // end of bus transaction
      }
      else 
         ret = 5;       // timeout error
   }
   return ret;
}
