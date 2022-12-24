/********************************************************************
 * @file iox_spi.cpp
 * @author J.Hoeppner @ Abbycus
 * @brief 
 * @version 1.0
 * @date 2022-12-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "iox_spi.h"

SPIClass * spi_ctlr = NULL;

/********************************************************************
 * @brief Construct a new IOX_SPI class object
 */
IOX_SPI::IOX_SPI(void)
{
   // empty constructor
}


/********************************************************************
 * @brief Initialize the master UART controller.
 * 
 * @param spi_number - ESP32 spi controller, 0 or 1
 * @param pin_tx - UART TX pin
 * @param pin_rx - UART RX pin
 * @param uart_number - UART controller number. ESP32 has 2 or more UART 
 *       controllers. Typically the controller num would be 0 or 1.
 * @return true if successful
 */
bool IOX_SPI::init(uint8_t spi_number, uint8_t pin_sck, uint8_t pin_miso, uint8_t pin_mosi, uint8_t pin_ss0, uint8_t pin_ss1, uint32_t bus_speed)
{
   /** save SPI pin assignments **/
   _spi_num = spi_number;
   _pin_sck = pin_sck;
   _pin_miso = pin_miso;
   _pin_mosi = pin_mosi;
   _pin_ss0 = pin_ss0;
   _pin_ss1 = pin_ss1;
   _bus_speed = bus_speed;

   if(spi_number == 0)
      spi_ctlr = new SPIClass(FSPI);
   else if(spi_number == 1)
      spi_ctlr = new SPIClass(HSPI);
   else 
      return false;

   /** set the bus speed **/
   spi_ctlr->setFrequency(bus_speed);

   spi_ctlr->begin(pin_sck, pin_miso, pin_mosi, pin_ss0);   // SCLK, MISO, MOSI, SS
   pinMode(pin_ss0, OUTPUT);
   digitalWrite(pin_ss0, HIGH);           // CS is active low
   pinMode(pin_ss1, OUTPUT);
   digitalWrite(pin_ss1, HIGH);           // CS is active low   

   return true;
}


/********************************************************************
 * @brief set IOX UART baudrate to new value.
 * 
 * @param iox_adrs - device adrs (0 or 1)
 * @param baudrate - Baudrate in bits/sec. Example: 19200
 * @return uint8_t 
 */
uint8_t IOX_SPI::set_uart_baud(uint8_t iox_adrs, uint32_t baudrate)
{
   _clear_tx_bufr();
   _txbufr[0] = SET_UART_BAUD;
   _txbufr[1] = baudrate >> 16;
   _txbufr[2] = baudrate >> 8;
   _txbufr[3] = baudrate & 0xFF;

   uint8_t ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], -1);
   // if(ret == ERR_NONE)
   // {
   //    ret = spi_receive((uint8_t *)&_rxbufr[0], 1);
   //    ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   // }
   return ret;
}


/********************************************************************
 * @brief Who Am I checks the bus connection. The IOX device is 
 *       expected to return a value of 0x5D.
 * @return ERR_NONE if connection is valid.
 */
uint8_t IOX_SPI::whoAmI(uint8_t iox_adrs)   // connection test
{
   _clear_tx_bufr();
   _txbufr[0] = WHO_AM_I;
   uint8_t ret = ERR_NONE;

   spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 1);  // bytes to return 
   if(_rxbufr[0] != WHOAMI_MAGIC_NUM)    // validate connection 
      ret = ERR_GP_FAILURE;

   return ret;
}


/********************************************************************
 * @brief Error Check returns a STATUS byte and an ERROR word reflecting
 *       the status from the last operation.
 * @param sys_status - pointer to a SYS_STATUS structure - see IOX_SPI.h
 * 
 * @return ERR_NONE if successful. Otherwise see error codes in IOX_SPI.h 
 */
uint8_t IOX_SPI::read_status(uint8_t iox_adrs, SYS_STATUS *sys_status, uint8_t field_mask)  // connection test
{
   int16_t _field;

   _clear_tx_bufr();
   _txbufr[0] = READ_STATUS;
   _txbufr[1] = field_mask;

   uint8_t ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 3);
   if(ret == ERR_NONE)
   {
      sys_status->status = _rxbufr[0];    // status in high byte, error code in low byte
      _field = _rxbufr[1];
      _field = (_field << 8) | _rxbufr[2];
      sys_status->field = _field;
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
uint8_t IOX_SPI::config_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint8_t io_mode)
{
   uint8_t err = ERR_NONE;
   uint8_t ret;

   _clear_tx_bufr();
   _txbufr[0] = CONFIG_GPIOS;
   _txbufr[1] = (gpio_map >> 16);    // msb of gpio_map word
   _txbufr[2] = (gpio_map >> 8);    // msb of gpio_map word
   _txbufr[3] = (gpio_map & 0xFF);  // lsb of gpio_map word
   _txbufr[4] = io_mode;

   _rxbufr[0] = 0xFF;               // this should be overwritten by the returned status byte

   err = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);
   if(err == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return err;
}


/********************************************************************
 * @brief Return the IO mode for the specified GPIO.
 * 
 * @param gpio_num - Logical GPIO 0 - 19
 * @return 0xFF on error or...
 * GPIO mode encoded as follows:
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
uint8_t IOX_SPI::get_gpio_config(uint8_t iox_adrs, uint8_t gpio_num)
{
   uint8_t ret = 0;

   _clear_tx_bufr();
   _txbufr[0] = GET_GPIO_CONFIG;
   _txbufr[1] = gpio_num;        // param to send to slave

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);
   if(ret == ERR_NONE)
      ret = _rxbufr[0];          // get encoded GPIO mode
   else 
      ret = 0xFF;                // something went wrong!

   return ret;
}


/********************************************************************
 * @brief Write one or more gpio's with the same state.
 * 
 * @param gpio_map - bitmap of gpio's to write (lower 20 bits).
 * @param state_map - bitmap of states to write (lower 20 bits)
 * @return ERR_NONE if successful otherwise an error code 
 */
uint8_t IOX_SPI::write_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint32_t state_map)
{
   uint16_t wrgpio = 0;
   uint8_t ret = ERR_NONE;

   _clear_tx_bufr();
   _txbufr[0] = WRITE_GPIOS;
   _txbufr[1] = gpio_map >> 16;     // msb first
   _txbufr[2] = gpio_map >> 8;   
   _txbufr[3] = gpio_map & 0xFF;
   _txbufr[4] = state_map >> 16;     // msb first
   _txbufr[5] = state_map >> 8;   
   _txbufr[6] = state_map & 0xFF;

   _rxbufr[0] = 0xFF;                  // should be overwritten by the status byte
   
   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Toggle the output state of a range of GPIO's.
 * 
 * @param start - starting GPIO number
 * @param end - ending GPIO number
 * @return true if operation is successful
 */
uint8_t IOX_SPI::toggle_gpios(uint8_t iox_adrs, uint32_t _gpio_map)
{
   uint16_t gpios;
   uint8_t ret;

   _clear_tx_bufr();
   _txbufr[0] = TOGGLE_GPIOS; 
   _txbufr[1] = _gpio_map >> 16;      // msb first
   _txbufr[2] = _gpio_map >> 8;
   _txbufr[3] = _gpio_map & 0xFF;

   _rxbufr[0] = 0xFF;
   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
   }

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
uint8_t IOX_SPI::read_gpio(uint8_t iox_adrs, uint8_t gpio_num, uint8_t *gpio_val)   // connection test
{
   _clear_tx_bufr();
   _txbufr[0] = READ_GPIO;
   _txbufr[1] = gpio_num;

   _rxbufr[1] = 0xFF;                     // clear the returned status

   uint8_t ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 1);
   if(ret == ERR_NONE)
   {
      if(_rxbufr[1] != 0xFF)              // 2nd byte is the status
      {
         *gpio_val = _rxbufr[0];          // 1st byte is GPIO state
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
uint8_t IOX_SPI::read_gpio_all(uint8_t iox_adrs, uint32_t *gpio_map)
{
   uint32_t _gpio_map = 0;
   uint8_t ret;

   if(gpio_map == NULL)
      return ERR_PARAM;

   _clear_tx_bufr();
   _txbufr[0] = READ_GPIO_ALL;

   _rxbufr[3] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 3); 
   if(ret == ERR_NONE)
   {
      if(_rxbufr[3] != 0xFF)                    // check returned status
      {
         _gpio_map = _rxbufr[0];                // 20 bit field MSB first
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
uint8_t IOX_SPI::config_event_output(uint8_t iox_adrs, uint8_t event_type, uint8_t event_io)
{
   uint8_t ret;

   _clear_tx_bufr();
   _txbufr[0] = CONFIG_EXTI_EVENT;
   _txbufr[1] = event_type;
   _txbufr[2] = event_io;      // msb first

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
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
uint8_t IOX_SPI::start_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t clk_div, uint16_t pwm_freq, uint16_t pwm_duty, bool polarity)
{
   uint8_t ret;

   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the pwm values **/
   _clear_tx_bufr();   
   _txbufr[0] = START_PWM;
   _txbufr[1] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[2] = (clk_div >> 8);
   _txbufr[3] = (clk_div & 0xFF);   
   _txbufr[4] = (pwm_freq >> 8);
   _txbufr[5] = (pwm_freq & 0xFF); 
   _txbufr[6] = (pwm_duty >> 8);
   _txbufr[7] = (pwm_duty & 0xFF); 
   _txbufr[8] = polarity; 

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);    
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
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
uint8_t IOX_SPI::update_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t pwm_duty)
{
   uint8_t ret;
   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the new PWM duty cycle value **/
   _clear_tx_bufr();   
   _txbufr[0] = UPDATE_PWM;
   _txbufr[1] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[2] = (pwm_duty >> 8);    
   _txbufr[3] = (pwm_duty & 0xFF);

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);    
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
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
uint8_t IOX_SPI::config_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint8_t adc_resol)
{
  uint8_t ret;

   /** load the _txbufr with the pwm values **/
   _clear_tx_bufr();   
   _txbufr[0] = CONFIG_ADC;
   _txbufr[1] = (adc_chnls >> 8);         // adc channel map MSB
   _txbufr[2] = (adc_chnls & 0xFF);
   _txbufr[3] = adc_resol;

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);  
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
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
uint8_t IOX_SPI::start_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint16_t num_samples)
{
   uint8_t ret;
   uint16_t adcval = 0;

   /** load the _txbufr with the pwm values **/
   _clear_tx_bufr();   
   _txbufr[0] = START_ADC;
   _txbufr[1] = (adc_chnls >> 8);               // adc chnl map
   _txbufr[2] = (adc_chnls & 0xFF);
   _txbufr[3] = (num_samples >> 8);
   _txbufr[4] = (num_samples & 0xFF);

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0);  
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
   }    
   return ret; 
}


/********************************************************************
 * @brief Read result of one ADC channel (0 - 9).
 * 
 * @param adc_num - 0 - 9
 * @param adc_conv - pointer to an ADC_CONVERT structure - see IOX_SPI.h
 * @return ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_SPI::read_ADC(uint8_t iox_adrs, uint8_t adc_num, ADC_CONVERT *adc_conv)
{
   uint8_t ret;
   uint16_t adcval = 0;             // error return value

   if(adc_num > 9 || adc_conv == NULL)
      return ERR_PARAM;             // validate adc number range 0 - 9

   /** load the _txbufr with the adc channel number **/
   _clear_tx_bufr();   
   _txbufr[0] = READ_ADC;
   _txbufr[1] = adc_num;            // adc chnl 0 - 9

   _rxbufr[8] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 8); 
   if(ret == ERR_NONE && _rxbufr[8] != 0xFF) 
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
   else 
      ret = ERR_GP_FAILURE;

   return ret;
}


/********************************************************************
 * @brief Start an Input Capture measurement
 * 
 * @param capt_num - Capture channel number (0-9)
 * @param trig_edge - INPUT_CAPT_RISING, INPUT_CAPT_FALLING, or INPUT_CAPT_BOTH
 * @param cap_type - CAPTURE_TYPE_FREQ or CAPTURE_TYPE_PW
 */
uint8_t IOX_SPI::start_capture(uint8_t iox_adrs, uint8_t capt_num, uint8_t trig_edge, uint8_t capt_type)
{
   uint8_t ret;
   if(capt_num > 9)
      return ERR_PARAM;

   _clear_tx_bufr();
   _txbufr[0] = START_CAPTURE;
   _txbufr[1] = capt_num;            // capture chnl 0 - 9
   _txbufr[2] = trig_edge; 
   _txbufr[3] = capt_type;

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0); 
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read capture measurement.
 * 
 * @param capt_num - capture channel logical number (0-9)
 * @return uint32_t - 32 bit capture measurement
 */
uint32_t IOX_SPI::read_capture(uint8_t iox_adrs, uint8_t capt_num)
{
   uint8_t ret;
   uint32_t captval = 0;

   if(capt_num > 9)
      return 0;

   _clear_tx_bufr();
   _txbufr[0] = READ_CAPTURE;
   _txbufr[1] = capt_num;

   _rxbufr[4] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 4); 
   if(ret == ERR_NONE && _rxbufr[4] != 0xFF)
   {
      captval = _rxbufr[0];       // capture measurement value 
      captval = (captval << 8) | _rxbufr[1];
      captval = (captval << 8) | _rxbufr[2];
      captval = (captval << 8) | _rxbufr[3];
   }
   else 
      captval = 0;;

   return captval;
}


/********************************************************************
 * @brief Configure a pair of inputs for encoder function
 * 
 * @param enc_num - logical encoder number (0 - 7)
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_SPI::config_encoder(uint8_t iox_adrs, uint8_t enc_num)
{
   uint8_t ret;
   if(enc_num > 7)
      return ERR_PARAM;

   _clear_tx_bufr();
   _txbufr[0] = CONFIG_ENCODER;
   _txbufr[1] = enc_num;

   _rxbufr[0] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 0); 
   if(ret == ERR_NONE)
   {
      ret = (_rxbufr[0] != 0xFF) ? ERR_NONE : ERR_GP_FAILURE;
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
uint8_t IOX_SPI::read_encoder(uint8_t iox_adrs, uint8_t enc_num, ROT_ENC * rot_enc)
{
   uint8_t ret;

   if(enc_num > 7)
      return ERR_PARAM;

   _clear_tx_bufr();
   _txbufr[0] = READ_ENCODER;
   _txbufr[1] = enc_num;

   _rxbufr[5] = 0xFF;

   ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 5); 
   if(ret == ERR_NONE && _rxbufr[5] != 0xFF)
   {
      rot_enc->dir = _rxbufr[0];
      rot_enc->count = _rxbufr[1];
      rot_enc->count = (rot_enc->count << 8) | _rxbufr[2];
      rot_enc->speed = _rxbufr[3];
      rot_enc->speed = (rot_enc->speed << 8) | _rxbufr[4];
   }
   else 
      ret = ERR_GP_FAILURE;

   return ret;
}


/********************************************************************
 * @brief Enter sleep mode. Wake on GPIO level change.
 * 
 * @param wake_gpio - gpio number to use as wake up.
 * @note sleep does not return a status byte because it should be sleeping.
 * @return uint8_t - ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_SPI::sleep(uint8_t iox_adrs, uint8_t wake_gpio)
{
   _clear_tx_bufr();
   _txbufr[0] = SLEEP;
   _txbufr[1] = wake_gpio;
   uint8_t ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], -1);     // send wakeup gpio
   return ret;
}


/********************************************************************
 * @brief Return version information to the VERSION_INFO struct
 * 
 * @param iox_adrs - device number 0 or 1
 * @param version - pointer to a VERSION_INFO structure
 * @return ERR_NONE if successful
 */
uint8_t IOX_SPI::get_version(uint8_t iox_adrs, VERSION_INFO *version)
{
   _clear_tx_bufr();
   _txbufr[0] = GET_VERSION;

   _rxbufr[4] = 0xFF;

   uint8_t ret = spi_send(iox_adrs, (uint8_t *)&_txbufr[0], (uint8_t *)&_rxbufr[0], 4);     // request device version info

   if(ret == ERR_NONE && _rxbufr[4] != 0xFF)
   {
      version->reserved = _rxbufr[0];
      version->bus_version = _rxbufr[1];
      version->major_rev = _rxbufr[2];
      version->minor_rev = _rxbufr[3];
      ret = ERR_NONE;
   }
   else 
      ret = ERR_GP_FAILURE;

   return ret;
}


/********************************************************************
 * @brief SPI helper functions *************************************
 *******************************************************************/

/********************************************************************
 * @brief spi_send - send bytes to IOX device and receive (if any).
 *    Due to the syncronous nature of SPI, the library will always 
 *    send a fixed 10 byte packet which includes the command byte and 
 *    up to 9 parameter bytes.
 * 
 * @param device_num - IOX device, 0 or 1
 * @param xmit_buf - pointer to callers xmit data buffer
 * @param rcv_buf - pointer to callers recv buffer
 * @param bytes2rcv - number of bytes IOX device will return
 * @return ERR_NONE if successful, else failure error code
 */
uint8_t IOX_SPI::spi_send(uint8_t device_num, uint8_t *xmit_buf, uint8_t *rcv_buf, int8_t bytes2rcv)
{
   uint8_t ret = ERR_NONE;
   uint8_t retry = 0;
   uint16_t i;
   uint8_t rd_bufr[24];
   uint8_t pinSS = (device_num == 0) ? _pin_ss0 : _pin_ss1;    // choose device, 0 or 1

   /** TRANSMIT command & parameters **/
   digitalWrite(pinSS, LOW);              // pull SS slow to prep other end for transfer
   spi_ctlr->beginTransaction(SPISettings(_bus_speed, MSBFIRST, SPI_MODE0));
   spi_ctlr->transferBytes((uint8_t *)xmit_buf, NULL, SPI_TX_PACKET_SZ);      // send the command & params
   spi_ctlr->endTransaction();
   digitalWrite(pinSS, HIGH);             // pull SS high to signify end of data transfer

   while(true)
   {
      for(i=0; i<16; i++)
      {
         rcv_buf[i] = 0x0;
      }

      /** RECEIVE return data if any **/
      if(bytes2rcv >= 0)
      {
         digitalWrite(pinSS, LOW);              // pull SS slow to prep other end for transfer
         spi_ctlr->beginTransaction(SPISettings(_bus_speed, MSBFIRST, SPI_MODE0));
         /** function returns fixed packet (SPI_RX_PACKET_SZ) **/
         spi_ctlr->transferBytes(NULL, (uint8_t *)rd_bufr, SPI_RX_PACKET_SZ);    // receive data
         spi_ctlr->endTransaction();
         digitalWrite(pinSS, HIGH);             // pull SS high to signify end of data transfer
               
               /** DEBUGGING **/
               // Serial.print("rd_bufr= ");
               // for(uint8_t i=0; i<SPI_RX_PACKET_SZ; i++)
               // {
               //    Serial.printf("0x%X ", rd_bufr[i]);
               // }
               // Serial.println("");

         if(rd_bufr[0] == 0x01)                    // ready status found?
         {
            for(i=0; i<bytes2rcv; i++)             // move return data to callers rcv buffer
            {
               rcv_buf[i] = rd_bufr[i+1];
            }
            break;
         }
         else
         {
            if(retry++ > 10)
            {
               ret = ERR_DATA_NOT_RDY;
               break;
            }
            vTaskDelay(2);                      // small delay for retry
         }
      }
      else 
         break;
   }
   return  ret;      
}


/********************************************************************
 * @brief Clear the txbufr prior to use.
 * 
 */
void IOX_SPI::_clear_tx_bufr(void)
{
   uint8_t i;
   for(i=0; i<10; i++)
   {
      _txbufr[i] = 0x0;
   }
}


/********************************************************************
 * @brief Helper function to convert first bit-wise ADC channel map 
 *       to its logical number.
 * 
 * @param adc_chnls 
 * @return int8_t - logical adc chnl (0 - 9) or -1 if adc_chnls == 0.
 */
int8_t IOX_SPI::find_first_adc(uint16_t adc_chnls)
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
