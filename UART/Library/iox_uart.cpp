/********************************************************************
 * @file iox_uart.cpp
 * @author J.Hoeppner @ Abbycus
 * @brief 
 * @version 1.0
 * @date 2022-11-11
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note This code uses an ESP32-S3 MPU as the bus master. PlatformIO 
 *       was used as the IDE with Arduino framework.
 */

#include "iox_uart.h"

/********************************************************************
 * @brief Construct a new IOX_UART class object
 */
IOX_UART::IOX_UART(void)
{
   // empty constructor
}


/********************************************************************
 * @brief Initialize the bus master UART controller. 
 * 
 * @note *** The 
 * 
 * @param pin_tx - UART TX pin
 * @param pin_rx - UART RX pin
 * @param uart_number - UART controller number. ESP32 has 2 or more UART 
 *       controllers. Typically the controller num would be 0 or 1.
 * @return true if successful
 */
bool IOX_UART::init(uint8_t uart_number, uint8_t pin_tx, uint8_t pin_rx, uint32_t baudrate)
{
   uint8_t ret;
   _uart_num = uart_number;
   _pin_tx = pin_tx;
   _pin_rx = pin_rx;
   uint32_t serial_config = 0x800001C;    // 8 bit data, 1 stop bit, no parity

   switch(uart_number)
   {
      case 0:
         Serial.begin(baudrate, serial_config, pin_rx, pin_tx);
         while(!Serial) {}                // wait for serial to initialize
         break;

      case 1:
         Serial1.begin(baudrate, serial_config, pin_rx, pin_tx);
         _serialBufSize = Serial1.availableForWrite();
         while(!Serial1) {}        
         break;

      case 2:
         Serial2.begin(baudrate, serial_config, pin_rx, pin_tx);
         while(!Serial2) {}        
         break;

      default:
         return false;
   }
   return true;
}


/********************************************************************
 * @brief Set IOX UART baudrate to new value. Obviously should match 
 *       the bus master's baud rate.
 * @note On power up or hard reset the IOX device will default to 115200 baud.
 * 
 * @param iox_adrs - device adrs (0 or 1)
 * @param baudrate - Baudrate in bits/sec. Example: 19200
 * @return uint8_t 
 */
uint8_t IOX_UART::set_uart_baud(uint8_t iox_adrs, uint32_t baudrate)
{
   _txbufr[0] = SET_UART_BAUD | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = baudrate >> 16;
   _txbufr[2] = baudrate >> 8;
   _txbufr[3] = baudrate & 0xFF;

   uint8_t ret = uart_send((uint8_t *)&_txbufr, 4);

   return ret;
}


/********************************************************************
 * @brief Who Am I validates the bus connection. The IOX device is 
 *       expected to return a value of 0x5D or 0xDD (ADR0 = 1).
 * @return ERR_NONE if connection is valid.
 */
uint8_t IOX_UART::whoAmI(uint8_t iox_adrs)   // connection test
{
   _txbufr[0] = WHO_AM_I | ((iox_adrs > 0) ? 0x80 : 0x0);
   uint8_t ret = uart_send((uint8_t *)&_txbufr, 1);

   if(ret == ERR_NONE)
   {
      _rxbufr[0] = 0;
      ret = uart_read((uint8_t *)&_rxbufr, 2);  // returns number of bytes read
      if(ret == 2)                              // whoami should return 2 bytes
      {
         if(_rxbufr[0] == (WHOAMI_MAGIC_NUM | (iox_adrs << 7)))    // validate connection 
            ret = ERR_NONE;
         else
            ret = ERR_GP_FAILURE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read status returns a STATUS byte and an ERROR word reflecting
 *       the status from the last operation.
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param sys_status - pointer to a SYS_STATUS structure - see IOX_UART.h
 * @param field_mask - requested field to be returned in the FIELD word.
 * 
 * @return ERR_NONE if successful. Otherwise see error codes in IOX_UART.h 
 */
uint8_t IOX_UART::read_status(uint8_t iox_adrs, SYS_STATUS *sys_status, uint8_t field_mask)  // connection test
{
   int16_t _field;
   uint8_t ret;

   _txbufr[0] = READ_STATUS | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = field_mask;

   ret = uart_send((uint8_t *)&_txbufr, 2);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 3);
      if(ret > 2)                           // read three bytes from the IOX
      {
         sys_status->status = _rxbufr[0];    // status in high byte, error code in low byte
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param gpio_map - 20 bit map of the GPIOs to be configured. A '1'
 *       in the associated bit position causes that GPIO to be configured.
 * @param io_mode - Encoded mode, type, pullup, speed, and exti enable.
 * @example 
 *    Configure GPIO0 and GPIO4 to be a push-pull output, medium speed.
 *    ---------------------------------------------------------------
 *    config_gpios(0, 0x00000003, IO_OUTPUT_PP | IO_SPEED_MED) 
 * 
 *    Configure GPIO5 - GPIO19 as inputs, weak pullup, ext intr rising edge
 *    ---------------------------------------------------------------
 *    config_gpios(0, 0b11111111111111100000, IO_INPUT | IO_PULLUP | IO_EXTI_RISING);
 * 
 *    Configure GPIO3 & GPIO4 as output, open drain, weak pullup
 *    ---------------------------------------------------------------
 *    config_gpios(0, (GPIO3_b | GPIO4_b), IO_OUTPUT_OD | IO_PULLUP);
 * 
 * @return 0 if successful, error code otherwise.
 */
uint8_t IOX_UART::config_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint8_t io_mode)
{
   uint8_t err = ERR_NONE;
   uint8_t ret;

   _txbufr[0] = CONFIG_GPIOS | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = (gpio_map >> 16);    // msb of gpio_map word
   _txbufr[2] = (gpio_map >> 8);    // msb of gpio_map word
   _txbufr[3] = (gpio_map & 0xFF);  // lsb of gpio_map word
   _txbufr[4] = io_mode;

   err = uart_send((uint8_t *)&_txbufr, 5);
   if(err == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return err;
}


/********************************************************************
 * @brief Return the IO mode for the specified GPIO.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
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
uint8_t IOX_UART::get_gpio_config(uint8_t iox_adrs, uint8_t gpio_num)
{
   uint8_t ret = 0;
 
   _txbufr[0] = GET_GPIO_CONFIG | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = gpio_num;        // param to send to slave

   ret = uart_send((uint8_t *)&_txbufr, 2);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 2);  
      if(ret > 1)                // read one byte from the slave
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param gpio_map - bitmap of gpio's to write (lower 20 bits).
 *       A '1' bit flags the associated GPIO to be written.
 * @param state_map - bitmap of states to write (lower 20 bits)
 *       The bit state to write to the GPIO. 
 * @return ERR_NONE if successful otherwise an error code 
 */
uint8_t IOX_UART::write_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint32_t state_map)
{
   uint16_t wrgpio = 0;
   uint8_t ret = ERR_NONE;

   _txbufr[0] = WRITE_GPIOS | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = gpio_map >> 16;     // msb first
   _txbufr[2] = gpio_map >> 8;   
   _txbufr[3] = gpio_map & 0xFF;
   _txbufr[4] = state_map >> 16;     // msb first
   _txbufr[5] = state_map >> 8;   
   _txbufr[6] = state_map & 0xFF;
   
   ret = uart_send((uint8_t *)&_txbufr, 7);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1); 
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Toggle the output state of a range of GPIO's.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param _gpio_map - A '1' bit flags the associated GPIO to be inverted.
 * @return ERR_NONE if successful otherwise an error code 
 */
uint8_t IOX_UART::toggle_gpios(uint8_t iox_adrs, uint32_t _gpio_map)
{
   uint16_t gpios;
   uint8_t ret;

   _txbufr[0] = TOGGLE_GPIOS | ((iox_adrs > 0) ? 0x80 : 0x0); 
   _txbufr[1] = _gpio_map >> 16;      // msb first
   _txbufr[2] = _gpio_map >> 8;
   _txbufr[3] = _gpio_map & 0xFF;
   ret = uart_send((uint8_t *)&_txbufr, 4);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1); 
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }

   return ret;
}


/********************************************************************
 * @brief Read the state of the specified GPIO
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param gpio_num - 0-19
 * @param gpio_val is a pointer to an 8 bit memory location where the 
 *       return value is written to.
 * @return uint8_t - ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_UART::read_gpio(uint8_t iox_adrs, uint8_t gpio_num, uint8_t *gpio_val)   // connection test
{
   uint8_t ret = 0;

   _txbufr[0] = READ_GPIO | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = gpio_num;

   ret = uart_send((uint8_t *)&_txbufr, 2);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 2);
      if(ret == 2)                        // read two bytes from the IOX
      {
         *gpio_val = _rxbufr[0];         // get GPIO state, 2nd value is status
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param gpio_map - pointer to a 32 bit variable where the 20 
 *       bit map will be written.
 * @return uint8_t - ERR_NONE if succesful, otherwise an error code.
 */
uint8_t IOX_UART::read_gpio_all(uint8_t iox_adrs, uint32_t *gpio_map)
{
   uint32_t _gpio_map = 0;
   uint8_t ret;

   if(gpio_map == NULL)
      return ERR_PARAM;

   _txbufr[0] = READ_GPIO_ALL | ((iox_adrs > 0) ? 0x80 : 0x0);

   ret = uart_send((uint8_t *)&_txbufr, 1); 
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 4);
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param event_type - type of event to trigger output. Values can be:
 *          EVENT_EXTI, EVENT_ADC, EVENT_CAPTURE, or EVENT_ENCODER.
 * @param event_io - logical gpio output. Valid values are GPIO16 - GPIO19.
 * @note - Any event_io values that are not valid (i.e. 0) will disable the 
 *       event function.
 * @return uint8_t - ERR_NONE if succesful, otherwise an error code.
 */
uint8_t IOX_UART::config_event_output(uint8_t iox_adrs, uint8_t event_type, uint8_t event_io)
{
   uint8_t ret;

   _txbufr[0] = CONFIG_EXTI_EVENT | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = event_type;
   _txbufr[2] = event_io;      // msb first

   ret = uart_send((uint8_t *)&_txbufr, 3);
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }

   return ret;
}   


/********************************************************************
 * @brief Configure & Start a PWM output.
 * @note: There are 10 output pins that can function as PWM outputs.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param pwm_num - PWM output. Valid numbers are 0 - 9.
 * @param clk_div - 16 bit value used to set the PWM clock prescale.
 * @param pwm_freq - PWM period frequency = clock speed / pwm_freq
 * @param pwm_duty - PWM duty cycle (on time) is ratio of pwm_freq / pwm_duty
 * @param polarity - 0 = duty cycle is high. 1 = inverse
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_UART::start_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t clk_div, uint16_t pwm_freq, uint16_t pwm_duty, bool polarity)
{
   uint8_t ret;

   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = START_PWM | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[2] = (clk_div >> 8);
   _txbufr[3] = (clk_div & 0xFF);   
   _txbufr[4] = (pwm_freq >> 8);
   _txbufr[5] = (pwm_freq & 0xFF); 
   _txbufr[6] = (pwm_duty >> 8);
   _txbufr[7] = (pwm_duty & 0xFF); 
   _txbufr[8] = polarity; 

   ret = uart_send((uint8_t *)&_txbufr, 9);    
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret; 
}


/********************************************************************
 * @brief Update a configured PWM output.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param pwm_num - PWM output to update
 * @param pwm_duty - 16 bit duty cycle value
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_UART::update_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t pwm_duty)
{
   uint8_t ret;
   if(pwm_num > 9)
      return ERR_PARAM;

   /** load the _txbufr with the new PWM duty cycle value **/
   _txbufr[0] = UPDATE_PWM | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = pwm_num;      // pwm chnl 0 - 9
   _txbufr[2] = (pwm_duty >> 8);    
   _txbufr[3] = (pwm_duty & 0xFF);

   ret = uart_send((uint8_t *)&_txbufr, 4);    
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }   
   return ret;    
}


/********************************************************************
 * @brief Configure one or more ADC channels. This funtion set the ADC
 *       resolution and configures gpio's for analog input.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param adc_chnls - bit-wise map of channels to configure as ADC input.
 *       Each bit from 0 - 9 represents a logical channel.
 * @param adc_resol - 6, 8, 10, or 12 bit resolution - applies to all channels
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_UART::config_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint8_t adc_resol)
{
  uint8_t ret;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = CONFIG_ADC | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = (adc_chnls >> 8);         // adc channel map MSB
   _txbufr[2] = (adc_chnls & 0xFF);
   _txbufr[3] = adc_resol;

   ret = uart_send((uint8_t *)&_txbufr, 4);  
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }

   return ret;       
}

      
/********************************************************************
 * @brief Start an ADC conversion on one or more channels with the 
 *       specified number os samples.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param adc_chnls - bitwise map of channels to convert
 * @param num_samples - number of samples to average over.
 * @return true if successful.
 */
uint8_t IOX_UART::start_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint16_t num_samples)
{
   uint8_t ret;
   uint16_t adcval = 0;

   /** load the _txbufr with the pwm values **/
   _txbufr[0] = START_ADC | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = (adc_chnls >> 8);               // adc chnl map
   _txbufr[2] = (adc_chnls & 0xFF);
   _txbufr[3] = (num_samples >> 8);
   _txbufr[4] = (num_samples & 0xFF);

   ret = uart_send((uint8_t *)&_txbufr, 5);  
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }    
   return ret; 
}


/********************************************************************
 * @brief Read result of one ADC channel (0 - 9).
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param adc_num - 0 - 9
 * @param adc_conv - pointer to an ADC_CONVERT structure - see IOX_UART.h
 * @return ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_UART::read_ADC(uint8_t iox_adrs, uint8_t adc_num, ADC_CONVERT *adc_conv)
{
   uint8_t ret;
   uint16_t adcval = 0;             // error return value

   if(adc_num > 9 || adc_conv == NULL)
      return ERR_PARAM;             // validate adc number range 0 - 9

   /** load the _txbufr with the adc channel number **/
   _txbufr[0] = READ_ADC | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = adc_num;            // adc chnl 0 - 9
   ret = uart_send((uint8_t *)&_txbufr, 2); 
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 9);
      if(ret == 9)             // read 9 bytes from the IOX
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
int8_t IOX_UART::find_first_adc(uint16_t adc_chnls)
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param capt_num - Capture channel number (0-9)
 * @param trig_edge - INPUT_CAPT_RISING, INPUT_CAPT_FALLING, or INPUT_CAPT_BOTH
 * @param cap_type - CAPTURE_TYPE_FREQ or CAPTURE_TYPE_PW
 * @return ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_UART::start_capture(uint8_t iox_adrs, uint8_t capt_num, uint8_t trig_edge, uint8_t capt_type)
{
   uint8_t ret;
   if(capt_num > 9)
      return ERR_PARAM;

   _txbufr[0] = START_CAPTURE | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = capt_num;            // capture chnl 0 - 9
   _txbufr[2] = trig_edge; 
   _txbufr[3] = capt_type;

   ret = uart_send((uint8_t *)&_txbufr, 4); 
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Read capture measurement.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param capt_num - capture channel logical number (0-9)
 * @return uint32_t - 32 bit capture measurement. Returns 0 on error.
 */
uint32_t IOX_UART::read_capture(uint8_t iox_adrs, uint8_t capt_num)
{
   uint8_t ret;
   uint32_t captval = 0;

   if(capt_num > 9)
      return 0;

   _txbufr[0] = READ_CAPTURE | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = capt_num;

   ret = uart_send((uint8_t *)&_txbufr, 2); 
   if(ret == ERR_NONE)
   {
      if(uart_read((uint8_t *)&_rxbufr, 5) == 5)      // read 5 bytes from the IOX
      {
         captval = _rxbufr[0];       // capture measurement value 
         captval = (captval << 8) | _rxbufr[1];
         captval = (captval << 8) | _rxbufr[2];
         captval = (captval << 8) | _rxbufr[3];
      }
      else 
         captval = 0;;
   }
   return captval;
}


/********************************************************************
 * @brief Configure a pair of inputs for rotary encoder function.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param enc_num - logical encoder number (0 - 7)
 * @return ERR_NONE if successful, error code otherwise.
 */
uint8_t IOX_UART::config_encoder(uint8_t iox_adrs, uint8_t enc_num)
{
   uint8_t ret;
   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = CONFIG_ENCODER | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = enc_num;
   ret = uart_send((uint8_t *)&_txbufr, 2); 
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 1);
      ret = (ret == 1) ? ERR_NONE : ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief Return last encoder values.
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param enc_num - logical encoder number (0 - 7)
 * @param rot_enc - pointer to a ROT_ENC struct
 * @return ERR_NONE if successful, otherwise an error code.
 */
uint8_t IOX_UART::read_encoder(uint8_t iox_adrs, uint8_t enc_num, ROT_ENC * rot_enc)
{
   uint8_t ret;

   if(enc_num > 7)
      return ERR_PARAM;

   _txbufr[0] = READ_ENCODER | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = enc_num;

   ret = uart_send((uint8_t *)&_txbufr, 2); 
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 6);
      if(ret == 6)             // read 5 bytes from the IOX
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
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param wake_gpio - gpio number to use as wake up. Valid 0 - 15.
 *    @note If wake_gpio == 255, the wake condition will be UART RX signal.
 *    @note sleep does not return a status byte because it should be sleeping.
 * @return uint8_t - ERR_NONE if successful, error code otherwise
 */
uint8_t IOX_UART::sleep(uint8_t iox_adrs, uint8_t wake_gpio)
{
   _txbufr[0] = SLEEP | ((iox_adrs > 0) ? 0x80 : 0x0);
   _txbufr[1] = wake_gpio;
   uint8_t ret = uart_send((uint8_t *)&_txbufr, 2);     // send wakeup gpio
   return ret;
}


/********************************************************************
 * @brief Return version information to the VERSION_INFO struct
 * 
 * @param iox_adrs - address of the IOX device (0 or 1)
 * @param version - pointer to a VERSION_INFO structure
 * @return ERR_NONE if successful
 */
uint8_t IOX_UART::get_version(uint8_t iox_adrs, VERSION_INFO *version)
{
   _txbufr[0] = GET_VERSION | ((iox_adrs > 0) ? 0x80 : 0x0);
   uint8_t ret = uart_send((uint8_t *)&_txbufr, 1);     // request device version info
   if(ret == ERR_NONE)
   {
      ret = uart_read((uint8_t *)&_rxbufr, 5);
      if(ret == 5)
      {
         version->reserved = _rxbufr[0];
         version->bus_version = _rxbufr[1];
         version->major_rev = _rxbufr[2];
         version->minor_rev = _rxbufr[3];
         ret = ERR_NONE;
      }
      else 
         ret = ERR_GP_FAILURE;
   }
   return ret;
}


/********************************************************************
 * @brief UART helper functions *************************************
 *******************************************************************/

/********************************************************************
 * @brief uart_send - send bytes to slave register.
 * 
 * @param databuf - ptr to xmit data buffer
 * @param bytes2send - number of bytes to send (including the command num)
 * @return ERR_NONE if successful, else failure error code
 */
uint8_t IOX_UART::uart_send(uint8_t *databuf, uint8_t bytes2send)
{
   uint8_t cmd = *databuf;             // set MSB of cmd byte with address flag
   size_t bytesWritten = 0;
   uint32_t timout = millis();

   switch(_uart_num)
   {
      case 0:
         Serial.setTimeout(UART_TIMEOUT);
         bytesWritten = Serial.write(databuf, bytes2send);
         break;

      case 1:
         Serial1.setTimeout(UART_TIMEOUT);
         bytesWritten = Serial1.write(databuf, bytes2send);
         break;

      case 2:
         Serial2.setTimeout(UART_TIMEOUT);
         bytesWritten = Serial2.write(databuf, bytes2send);
         break;                  

   }
   if(bytesWritten != bytes2send)
   {
      Serial.printf("uart_send write error, requested bytes to send=%d, actual bytes sent=%d\n", bytes2send, bytesWritten);
      switch(_uart_num)
      {
         case 0:
            Serial.flush();
            break;

         case 1:
            Serial1.flush();
            break;

         case 2:
            Serial2.flush();
            break;                        
      }
      return ERR_GP_FAILURE;
   }
   return  ERR_NONE;      
}


/********************************************************************
 * @brief uart_read - read bytes from IOX. This function follows a 
 * call to uart_send if there are any bytes to be read.
 * 
 * @param rcvbuf - ptr to rcv data bufr
 * @param numbytes - number of expected bytes to read
 * @return number of bytes actually read from the UART fifo
 */
uint8_t IOX_UART::uart_read(uint8_t *rcvbuf, uint8_t bytes2read)
{
   int16_t rxBytes = 0;
   uint32_t timout = millis();

   switch(_uart_num)
   {
      case 0:
         Serial.setTimeout(UART_TIMEOUT);
         rxBytes = Serial.readBytes(rcvbuf, bytes2read);
         break;

      case 1:
         Serial1.setTimeout(UART_TIMEOUT);
         rxBytes = Serial1.readBytes(rcvbuf, bytes2read);
         break;

      case 2:
         Serial2.setTimeout(UART_TIMEOUT);
         rxBytes = Serial2.readBytes(rcvbuf, bytes2read);
         break;

      default:
         return 0;
   }
      
   return (rxBytes & 0xFF);
}