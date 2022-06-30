# STM32-Oscilloscope
STM32 Digital Oscilloscope using the STM32F103C8 MCU and the ILI9341 2.4 inch TFT display.

Specifications:

* One input channel
* 2.57 Msps ADC, accepting signal frequencies up to 1.28 MHz
* Calculates minimum, maximum and average values
* Spectrum FFT analysis
* Fundamental frequency detection
* SD card export of signal wave shape and spectrum
* Freeze function
* Sampling rate selection

Fork modifications:
* ILI9341 on SPI
* Removed SD card code
* Removed buttons and replaced with single encoder

Wiring:
```c
#define TFT_DC      PA0
#define TFT_CS      PA1
#define TFT_RST     PA2
#define TFT_LED     PA3 // Backlight

#define ENC_BUTTON PB7
#define ENC_CLK PB8
#define ENC_DATA PB9

#define TEST_SIGNAL PA8
#define CHANNEL_1 PB0
```

Original post: https://www.gameinstance.com/post/80/STM32-Oscilloscope-with-FFT-and-SD-export
