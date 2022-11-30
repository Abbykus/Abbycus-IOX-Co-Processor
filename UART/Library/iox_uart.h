/********************************************************************
 * @file iox_uart.h
 * @author J.Hoeppner @ Abbycus
 * @brief 
 * @version 1.0
 * @date 2022-09-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _IOX_UART_H
#define _IOX_UART_H

#include <Arduino.h>
#include "Wire.h"
#include "driver/uart.h"

#define SERIAL_SIZE_RX     130
#define WHOAMI_MAGIC_NUM   0x5D
#define UART_TIMEOUT       250         // arbitrary bus transaction timeout in milliseconds

/** ABBY IOx command definitions **/
enum cmd_nums {
   REG_NONE = 0,
	WHO_AM_I,               // WHO AM I == 1
   READ_STATUS,            // return error code from last operation
	CONFIG_GPIOS,           // set range of I/O's to input or output
	GET_GPIO_CONFIG,        // get mode of specified gpio
	WRITE_GPIOS,            // write a range of gpio's with a state bit
   TOGGLE_GPIOS,           // toggle one or more gpio's
	READ_GPIO,              // read state of a single gpio. 
   READ_GPIO_ALL,          // read all gpio's and returns a 20 bit value.
   CONFIG_EXTI_EVENT,      // configure exti event output
   START_PWM,				   // start a PWM output
	UPDATE_PWM,             // update a PWM output
   CONFIG_ADC,             // config one or more ADC channels
   START_ADC,              // start an ADC conversion sequence
   READ_ADC,               // read results of last ADC conversion
	START_CAPTURE,          // arm capture & wait for completion
	READ_CAPTURE,			   // read input capture value
   CONFIG_ENCODER,         // configure an encoder input
   READ_ENCODER,           // read an encoder 
   SLEEP,                  // enter low power sleep mode
   SET_UART_BAUD,          // sets the UART baudrate (UART IF only)
};

enum errors {
   ERR_NONE = 0,           // good to go
   ERR_PARAM,              // 1 = bad parameter
   ERR_GP_FAILURE,         // 2 = general failure
   ERR_WRONG_MODE,         // bus is in wrong mode
   ERR_TIMEOUT,            // bus timeout
   ERR_SEND,
   ERR_RECV,
};

enum status_bits {
	STATUS_NONE = 0x0,
   STATUS_ERROR = 0x1,
	STATUS_EXTI = 0x2,
   STATUS_ADC_BUSY = 0x4,   
	STATUS_ADC_READY = 0x8,
   STATUS_CAPTURE = 0x10,
   STATUS_ENCODER = 0x20,
};

enum field_mask {
   FIELD_ERROR=0x1,
   FIELD_EXTI=0x2,
   FIELD_ADC_READY=0x8,
   FIELD_CAPTURE=0x10,
   FIELD_ENCODER=0x20,
};

typedef struct {
   uint8_t status;
   uint16_t field;
}SYS_STATUS;


typedef struct {
   bool dir;
   int16_t count;
   uint16_t speed;
}ROT_ENC;

typedef struct {
   uint16_t mean;
   uint16_t min;
   uint16_t max;
   uint16_t samples;
}ADC_CONVERT;

enum gpio_bitwise {
   GPIO_0 = 0x1,
   GPIO_1 = 0x2,
   GPIO_2 = 0x4,
   GPIO_3 = 0x8,
   GPIO_4 = 0x10,
   GPIO_5 = 0x20,
   GPIO_6 = 0x40,
   GPIO_7 = 0x80,
   GPIO_8 = 0x100,
   GPIO_9 = 0x200,
   GPIO_10 = 0x400,
   GPIO_11 = 0x800,
   GPIO_12 = 0x1000,
   GPIO_13 = 0x2000,
   GPIO_14 = 0x4000,
   GPIO_15 = 0x8000,
   GPIO_16 = 0x10000,
   GPIO_17 = 0x20000,
   GPIO_18 = 0x40000,
   GPIO_19 = 0x80000,
};

/** Basic mode - inout/output **/
#define IO_INPUT              0x0
#define IO_OUTPUT_PP          0x20     // push-pull output drive (normal)
#define IO_OUTPUT_OD          0x30     // open-drain output (wire-OR applications)
#define IO_LOW_POWER          0x3C     // force I/O to low power mode
#define IO_ANALOG             IO_LOW_POWER

/** Pullup / Pulldown Attributes **/
#define IO_NO_PUPD            0x00     // no pullup or pulldown applied
#define IO_PULLUP             0x04     // enable pullup 
#define IO_PULLDOWN           0x08     // enable pulldown

/** i/o speed constants - only applies if the GPIO is an OUTPUT **/
#define IO_SPEED_LOW          0x0      // output speed (current drive) 2 MHz
#define IO_SPEED_MED          0x1      // 10 MHz
#define IO_SPEED_HIGH         0x3      // 50 MHz

/** i/o EXTI constants - only applies if the GPIO is an INPUT **/
#define IO_EXTI_DISABLE       0x0      // exti disabled
#define IO_EXTI_RISING        0x1      // exti enabled, rising edge trigger
#define IO_EXTI_FALLING       0x2      // exti enabled, falling edge trigger
#define IO_EXTI_BOTH          0x3      // exti enabled, rising & falling edge trigger

/** ADC resolution constants **/
#define ADC_RESOL_12          12
#define ADC_RESOL_10          10
#define ADC_RESOL_8           8
#define ADC_RESOL_6           6

#define NUM_ADC_CHNLS         10
enum adc_chnls {
   ADC_0 = 0x0001,
   ADC_1 = 0x0002,
   ADC_2 = 0x0004,
   ADC_3 = 0x0008,
   ADC_4 = 0x0010,
   ADC_5 = 0x0020,
   ADC_6 = 0x0040,
   ADC_7 = 0x0080,
   ADC_8 = 0x0100,
   ADC_9 = 0x0200,         
};

/** Inputt Capture constants **/
#define INPUT_CAPT_RISING		0
#define INPUT_CAPT_FALLING		1
#define INPUT_CAPT_BOTH			5

#define CAPTURE_TYPE_PW			false		// measure a pulse width (time between one edge and next opposite edge)
#define CAPTURE_TYPE_FREQ		true     // measure frequency (time between same edges)

enum capt_bitwise {
   CAPT_0 = 0x1,
   CAPT_1 = 0x2,
   CAPT_2 = 0x4,
   CAPT_3 = 0x8,
   CAPT_4 = 0x10,
   CAPT_5 = 0x20,
   CAPT_6 = 0x40,
   CAPT_7 = 0x80,
   CAPT_8 = 0x100,
   CAPT_9 = 0x200,
   CAPT_10 = 0x400,
   CAPT_11 = 0x800,
   CAPT_12 = 0x1000,
   CAPT_13 = 0x2000,
   CAPT_14 = 0x4000,
   CAPT_15 = 0x8000,
};

#define MAX_BUFR_SIZE         24

/** PWM period defines **/
#define PWM_CLK_500_KHZ		95			// timer clock 2 usec
#define PWM_CLK_1_MHZ		47			// timer clock 1 usec
#define PWM_CLK_2_MHZ		23			// timer clock 0.5 usec
#define PWM_CLK_4_MHZ		11			// timer clock 0.25 usec
#define PWM_CLK_8_MHZ		5			// timer clock 0.125 usec
#define PWM_CLK_12_MHZ		3			// timer clock 0.083 usec
#define PWM_CLK_16_MHZ		2			// timer clock 0.0625 usec
#define PWM_CLK_24_MHZ		1			// timer clock 0.042 usec
#define PWM_CLK_48_MHZ		0			// timer clock 0.0208 usec

#define PWM_POLARITY_HIGH  0        // duty cycle is high
#define PWM_POLARITY_LOW   1        // duty cycle is low

enum pwm_bitwise {
   PWM_0 = 0x1,
   PWM_1 = 0x2,
   PWM_2 = 0x4,
   PWM_3 = 0x8,
   PWM_4 = 0x10,
   PWM_5 = 0x20,
   PWM_6 = 0x40,
   PWM_7 = 0x80,
   PWM_8 = 0x100,
   PWM_9 = 0x200,
   PWM_10 = 0x400,
   PWM_11 = 0x800,
   PWM_12 = 0x1000,
   PWM_13 = 0x2000,
   PWM_14 = 0x4000,
   PWM_15 = 0x8000,
};


enum event_types {
   EVENT_NONE=0,                // make sure disabled == 0
   EVENT_EXTI,
   EVENT_ADC,
   EVENT_CAPTURE,
   EVENT_ENCODER,
};

enum event_nums {
   EVO0,
   EVO1,
   EVO2,
   EVO3,
};


/** IOX_UART class for communication to the IOX I/O co-processor **/
class IOX_UART {
   public:
      IOX_UART(void);            // constructor
      bool init(uint8_t uart_number, uint8_t pin_tx, uint8_t pin_rx, uint32_t baudrate);
      uint8_t whoAmI(uint8_t iox_adrs);   // connection test
      uint8_t read_status(uint8_t iox_adrs, SYS_STATUS *sys_status, uint8_t field_mask);
      uint8_t config_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint8_t io_mode);
      uint8_t get_gpio_config(uint8_t iox_adrs, uint8_t gpio_num);

      /** gpio write operations **/
      uint8_t write_gpios(uint8_t iox_adrs, uint32_t gpio_map, uint32_t state_map);    // write range of GPIO's to 0 or 1
      uint8_t toggle_gpios(uint8_t iox_adrs, uint32_t _gpio_map);  // toggle a range of GPIO's

      /** gpio read operations **/
      uint8_t read_gpio(uint8_t iox_adrs, uint8_t gpio, uint8_t *ret_value);
      uint8_t read_gpio_all(uint8_t iox_adrs, uint32_t * gpio_map);

      /** external interrupt configuration **/
      uint8_t config_event_output(uint8_t iox_adrs, uint8_t event_type, uint8_t event_io);

      /** PWM configuration & update functions **/
      uint8_t start_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t clk_div, uint16_t pwm_freq, uint16_t pwm_duty, bool polarity);
      uint8_t update_PWM(uint8_t iox_adrs, uint8_t pwm_num, uint16_t pwm_duty);

      /** ADC functions **/
      uint_least8_t config_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint8_t adc_resol);	 // config one or more gpio's for ADC input
      uint8_t start_ADC(uint8_t iox_adrs, uint16_t adc_chnls, uint16_t num_samples);
      uint8_t read_ADC(uint8_t iox_adrs, uint8_t adc_chnl, ADC_CONVERT *adc_conv);
      int8_t find_first_adc(uint16_t adc_chnls);

      /** Timer Input Capture functions **/
      uint8_t start_capture(uint8_t iox_adrs, uint8_t capt_chnl, uint8_t trig_edge, uint8_t capt_type);
      uint32_t read_capture(uint8_t iox_adrs, uint8_t capt_chnl);

      /** Encoder functions **/
      uint8_t config_encoder(uint8_t iox_adrs, uint8_t enc_chnl);
      uint8_t read_encoder(uint8_t iox_adrs, uint8_t enc_chnl, ROT_ENC *rot_enc);

      /** Sleep Mode **/
      uint8_t sleep(uint8_t iox_adrs, uint8_t wake_gpio);

      /** Communications parameters **/
      uint8_t set_uart_baud(uint8_t iox_adrs, uint32_t baudrate);


   private:
      uint8_t uart_send(uint8_t *databuf, uint8_t bytes2send);
      uint8_t uart_read(uint8_t *rcvbuf, uint8_t bytes2read);
      uint8_t _pin_tx;
      uint8_t _pin_rx;
      uint8_t _uart_num;
      uint8_t _adr0;
      uint8_t _last_error;
      uint8_t _rxbufr[MAX_BUFR_SIZE];
      uint8_t _txbufr[MAX_BUFR_SIZE];
      uint16_t _serialBufSize;
};


#endif