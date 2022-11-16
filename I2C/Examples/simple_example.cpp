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
 * The I2C master used is ESP32-S3.
 * 
 */

/** Inlude the IOX library file in the project local source dir **/
#include "iox_i2c.h"

/** Definitions **/
#define I2C_IOX_ADRS       0x5D        // 0x5D is the default I2C adrs
#define PIN_I2C_SDA        18          // SDA & SCK pin assignments - may be different for your project
#define PIN_I2C_SCK        17
#define I2C_BIT_RATE       400000U     // std rate = 100000, fast rate = 400000, can be up to 1000000 for fast plus

/** create IOX library object and pass I2C address **/
IOX_I2C iox_i2c(I2C_IOX_ADRS);


/********************************************************************
 * @brief arduino setup()
 * 
 */
void setup(void)
{
   Serial.begin(115200);
   while(!Serial) {}     

   /** initialize the IOX device **/
   if(!iox_i2c.init(PIN_I2C_SDA, PIN_I2C_SCK, I2C_BIT_RATE, &Wire))
   {
      Serial.println("IOX failed to initialize!");
   }
   else 
   {
      if(iox_i2c.whoAmI() != ERR_NONE)    // verify connection to IOX
      {
         Serial.println("IOX failed to connect, check wiring and correct I2C address!")
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
