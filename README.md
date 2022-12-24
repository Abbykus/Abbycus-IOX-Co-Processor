# ABBY-IOX I/O Co-Processor ![](/Photos/IOX_TOP_VIEW_SMALL.png)
The IOX module series are low cost / high performance co-processors which greatly enhance microprocessor I/O capability by using one of three high speed serial buses (I2C, UART, and SPI) to access up to 20 independent GPIO's with very significant features such as Pulse Width Modulation (PWM) outputs, digital (Capture) frequency / pulse width measurement, external input event detection, programmable evnt outputs, rotary encoder support, fast analog to digital conversion, etc.

## List of Features
- High speed serial bus options include I2C, UART, and SPI
- I2C bit rates of 100 Kbits/s (std mode), 400 Kbits/s (fast mode), and up to 1 Mbit/s (fast plus mode).
- UART serial bus supports baud rates up to 3 Mbits/s.
- 4-wire SPI with clock rate up to 10 Mbits/s.
- Up to 20 independent GPIO pins (19 for SPI version), configurable as Digital Input, Analog Input, or Output.
- All outputs can be either push-pull or open-drain type with programmable current drive and optional pull-up or pull-down resistors.
- 16 inputs (15 on SPI version) are capable of signal edge detection (rising, falling, or both). Inputs can also have pull-up or pull-down resistors.
- Up to 10 (9 on SPI version) PWM outputs.
- Up to 10 (9 on SPI version) digital capture measurement inputs (frequency & pulse width).
- Up to 4 event notification outputs.
- 12 bit fast ADC with up to 10 multiplexed inputs. Background conversion of multiple channels with up to 65535 samples.
- Support for up to 8 (7 on SPI version) rotary encoders - direction, count, and rotational speed for each encoder.
- Digital and I/O operation from 2.5V to 3.6V. NOTE: Module is NOT 5V tolerant.
- Sleep mode with programmable wake up methods.
- Companion C++ library for use with Arduino, platformIO, or other development environments.
- Optional breakout board with pogo pin socket for development and rapid programming. 
- DIY programming with the optional breakout board and ST-LINK V2 debugger from STMicroelectronics.

## Mechanical 
The IOX module is a 32 pin 16mm square package that can be mounted on headers, surface mounted to a PCB, or DIY wiring using thru-hole vias around the package.

## Library Installation
The IOX module comes with a C++ library to enable the user to quickly add expanded capabilities to a project (see Library folder). Simply add two files into the project's source folder: iox_xxx.h and iox_xxx.cpp.

The project should reference the library header file using **"#include iox_xxx.h"**.
Create a library object specifying the IOX_xxx library. Note that the I2C and UART version allows dual addressing where 2 IOX modules can be connected in parallel to the same bus. The SPI protocol can support more than 2 devices in parallel.

Call the library initialization function to begin communication. The following example works for Arduino / platformIO:
**iox_xxx.init(.........); (see examples for the IOX module types).


