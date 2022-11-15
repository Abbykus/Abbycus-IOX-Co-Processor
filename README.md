# ABBY-IOX-Co-Processor ![](/Photos/IOX_TOP_VIEW_SMALL.png)
The IOX module is a low cost / high speed co-processor which greatly enhances microprocessor I/O capability by using one of three high speed serial buses (I2C, UART, and SPI) to access up to 20 independent GPIO's with very significant features such as Pulse Width Modulation (PWM) outputs, digital frequency measurement inputs, external input event detection, rotary encoder support, fast analog to digital conversion, etc.

## List of Features
- I2C serial bus option supports transfer rates of 100 Kbits/s, 400 Kbits/s, and up to 1 Mbit/s.
- UART serial bus supports baud rates up to 6 Mbits/s.
- 4 wire SPI with clock rate up to 18 Mbits/s in both half and full duplex operation.
- Up to 20 independent GPIO pins (18 for SPI version), configurable as Digital Input, Analog Input, or Output.
- All outputs can be either push-pull or open-drain type with programmable current drive and optional pull-up or pull-down resistors.
- 16 inputs are capable of edge detection (rising, falling, or both). Inputs can also have pull-up or pull-down resistors.
- Up to 10 PWM outputs.
- Up to 10 digital frequency measurement inputs.
- Up to 4 event notification outputs.
- 12 bit fast ADC with up to 10 multiplexed inputs. Background conversion of multiple channels with up to 65535 samples.
- Support for up to 8 rotary encoders - direction, count, and rotational speed for each encoder.
- Digital and I/O operation from 2.5V to 3.6V. NOTE: Module is NOT 5V tolerant on pin 2 or any I/O pins.
- Sleep mode with programmable wake up methods.
- Companion C++ library for use with Arduino, platformIO, or other development environments.
- Optional breakout board with pogo pin socket for development and gang programming.
- DIY programming with the optional breakout board and ST-LINK V2 debugger from STMicroelectronics.

## Mechanical 

## Library Installation for I2C bus
The IOX module comes with a C++ library to enable the user to quickly add expanded capabilities to a project. Simply add two files into the project's source folder: iox_i2c.h and iox_i2c.cpp.

The project should reference the library header file using **"#include iox_i2c.h"**.
Create a library object specifying the I2C library and the I2C bus address **IOX_I2C ioxi2c(0x5D);** Note that the I2C version allows dual addressing such that 2 IOX modules can be connected to the same bus and controlled using two library objects - one with address 0x5D and the other with address 0x5E.

Call the library initialization function to begin communication. The following example works for Arduino / platformIO:
**iox_i2c.init(PIN_I2C_SDA, PIN_I2C_SCK, 400000U, &Wire);** where PIN_I2C_SDA and PIN_I2C_SCK are self explainatory. Bus speed is set to 400000U (I2C fast mode). The Arduino 'Wire' library I2C controller has the name 'Wire'.

Se also the IOX I2C User Manual in the DOCS folder.

**UART:** iox_uart.h and iox_uart.cpp.
**SPI:** iox_spi.h and iox_spi.cpp.
