/********************************************************************
 * @file dual_adrs_example.cpp
 * @author j.Hoeppner @ Abbycus
 * @brief IOX example code
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 * This code demonstrates an example of two IOX devices on the same UART bus.
 * This project uses the ESP32-S3 as the UART bus master.
 * 
 */

/** Include the IOX library file in your project source directory **/
#include "iox_uart.h"

/** Definitions **/
#define PIN_TX             18          // SDA & SCK pin assignments - may be different for your project
#define PIN_RX             17
#define BAUDRATE           115200      // default IOX baud rate

/** instanciate IOX library object **/
IOX_I2C iox_uart;


/********************************************************************
 * @brief arduino setup()
 * 
 */
void setup(void)
{
   Serial.begin(115200);
   while(!Serial) {}     

   /** initialize IOX library **/
   err = iox_uart.init(UART_NUM_1, TXD_PIN, RXD_PIN, DEFAULT_BAUDRATE);
   {
      Serial.println("Failed to initialize IOX library");
   }
  
   /** check comm on 1st device **/
   if(iox_uart.whoAmI(0) != ERR_NONE)
   {
      Serial.println("Failed to connect to 1st IOX device");
   }

   /** check comm on 1st device **/
   if(iox_uart.whoAmI(1) != ERR_NONE)
   {
      Serial.println("Failed to connect to 2nd IOX device");
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
