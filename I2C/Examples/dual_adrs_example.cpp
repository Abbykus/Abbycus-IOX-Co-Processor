/********************************************************************
 * @file dual_adrs_example.cpp
 * @author j.Hoeppner @ Abbycus
 * @brief IOX example code
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 * This code demonstrates an example of two IOX devices on the same I2C bus.
 * The I2C master used is ESP32-S3.
 * 
 */

/** Inlude the IOX library file in the project local source dir **/
#include "iox_i2c.h"

/** Definitions **/
#define I2C_IOX_ADRS_A     0x5D        // 0x5D is the default I2C adrs
#define I2C_IOX_ADRS_B     0x5E        // 0x5E is the auxiliary I2C adrs (pin 29 (ADR0) = high)
#define PIN_I2C_SDA        18          // SDA & SCK pin assignments - may be different for your project
#define PIN_I2C_SCK        17
#define I2C_BIT_RATE       400000U     // std rate = 100000, fast rate = 400000, can be up to 1000000 for fast plus

/** create IOX library object and pass I2C address **/
IOX_I2C iox_i2c_A(I2C_IOX_ADRS_A);
IOX_I2C iox_i2c_B(I2C_IOX_ADRS_B);


/********************************************************************
 * @brief arduino setup()
 * 
 */
void setup(void)
{
   Serial.begin(115200);
   while(!Serial) {}     

   /** initialize IOX A device **/
   if(!iox_i2c_A.init(PIN_I2C_SDA, PIN_I2C_SCK, I2C_BIT_RATE, &Wire))
   {
      Serial.println("Failed to initialize IOX A!");
   }
   else if(iox_i2c_A.whoAmI() != ERR_NONE)
   {
      Serial.println("Failed to connect to IOX A!");
   }


   /** initialize IOX B device **/
   if(!iox_i2c_B.init(PIN_I2C_SDA, PIN_I2C_SCK, I2C_BIT_RATE, &Wire))
   {
      Serial.println("Failed to initialize IOX B!");
   }
   else if(iox_i2c_B.whoAmI() != ERR_NONE)
   {
      Serial.println("Failed to connect to IOX B!");
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
