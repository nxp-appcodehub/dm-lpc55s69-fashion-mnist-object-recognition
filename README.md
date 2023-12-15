# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## LPC55S69 Fashion Mnist Object Recognition


This demo demonstrates the AI capabilities of the LPC55S69. It displays the image captured by the camera on the LCD screen and performs fashion mnist object recognition. It supports ten objects:
- T-shirt/top
- Trouser
- Pullover
- Dress
- Coat
- Sandal
- Shirt
- Sneaker
- Bag
- Ankle boot

The resolution of the image captured by the camera is 224×224, the pixel format is RGB565. The original image will be converted to the image resolution and format that the model requires, i.e. the resolution is 28×28 and grayscale.

The processing flow is shown below:
![flow](./images/flow.png)



#### Boards: LPCXpresso55S69
#### Categories: AI/ML
#### Peripherals: I2C, SPI
#### Toolchains: MDK

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
- MDK Keil 5.38
- SDK 2.10.0

## 2. Hardware<a name="step2"></a>
- [LPCXpresso55S69 Development Board](https://www.nxp.com/design/development-boards/lpcxpresso-boards/lpcxpresso55s69-development-board:LPC55S69-EVK)
  - Need rework to connect the OV7670 camera module.
- OV7670 camera module
- Waveshare 2.8inch Touch LCD Shield for Arduino
  - Old version, not Rev2.1, please check [this](https://www.waveshare.com/wiki/2.8inch_TFT_Touch_Shield#accordion2) for details.
  - Need use 0-ohm resistor or solder to short the SB1,SB2,SB3 pads, please check [this](https://www.waveshare.com/wiki/2.8inch_TFT_Touch_Shield#accordion1) for details.
- USB to TTL Converter

## 3. Setup<a name="step3"></a>


### 3.1 Step 1
Rework the LPCXpresso55S69 Development Board.
- LPC55S69 P0_16  -> OV7670 MCLK
- LPC55S69 P0_13 -> OV7670 VSYNC
- LPC55S69 P0_15 -> OV7670 PCLK
- LPC55S69 P0_0 -> OV7670 D0
- LPC55S69 P0_1 -> OV7670 D1
- LPC55S69 P0_2 -> OV7670 D2
- LPC55S69 P0_3 -> OV7670 D3
- LPC55S69 P0_4 -> OV7670 D4
- LPC55S69 P0_5 -> OV7670 D5
- LPC55S69 P0_6 -> OV7670 D6
- LPC55S69 P0_7 -> OV7670 D7
- LPC55S69 P1_20 -> OV7670 SCL
- LPC55S69 P1_21 -> OV7670 SDA

### 3.2 Step 2
Assemble the Waveshare 2.8inch Touch LCD Shield, OV7670 camera module, and LPCXpresso55S69 Development Board.

### 3.2 Step 3
Complie the project and download to the board.

### 3.2 Step 4
Unplug USB cable from the debugger port P6, plug the USB cable into **P5**, and power the board only through **P5**.

### 3.2 Step 5
Connect P8 RX to USB to TTL Converter RX, P8 GND to USB to TTL Converter GND. The baud rate is 115200.

## 4. Results<a name="step5"></a>
Place the prepared picture within the shooting range of the camera, the UART will display the result of the recognition.
![result](./images/uart_result.png)

## 5. FAQs<a name="step6"></a>
No FAQs have been identified for this project.

## 6. Support<a name="step7"></a>

#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-LPCXPRESSO55S69-blue)](https://github.com/search?q=org%3Anxp-appcodehub+LPCXpresso55S69+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-AI/ML-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+aiml+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-I2C-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+i2c+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-SPI-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+spi+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MDK-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mdk+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 7. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        | September 12<sup>th</sup> 2023 |

