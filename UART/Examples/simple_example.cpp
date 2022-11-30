/********************************************************************
 * @file simple_example.cpp
 * @author j.Hoeppner @ Abbycus
 * @brief simple IOX example code
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 * This code demonstrates a simple example of the IOX library usage.
 * This example uses the ESP32-S3 as the serial bus master.
 * 
 */

/** Inlude the IOX library file in the project local source dir **/
#include "iox_uart.h"

/** Definitions **/
#define PIN_TX            18          // UART TX & RX pin assignments - may be different for your project
#define PIN_RX            17
#define BAUDRATE          115200     // default baudrate

/** instanciate IOX library object **/
IOX_UART iox_uart;


/********************************************************************
 * @brief arduino setup()
 * 
 */
void setup(void)
{
   Serial.begin(115200);                                      // ESP32 uses uart ctlr UART_NUM_0
   while(!Serial) {}     

   /** initialize the IOX device **/
   if(!iox_uart.init(UART_NUM_1, PIN_TX, PIN_RX, BAUDRATE);   // init UART ctlr 1 @ 115200 baud
   {
      Serial.println("IOX failed to initialize!");
   }
   else 
   {
      if(iox_uart.whoAmI() != ERR_NONE)    // verify connection to IOX
      {
         Serial.println("IOX failed to connect, check wiring and corect baud rate!");
      }
   }
}

/********************************************************************
 * @brief arduino loop()
 * 
 */
void loop(void)
{
   /** do stuff **/
}
      
