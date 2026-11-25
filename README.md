# PWM Signal Generation and Frequency Monitoring

ECE 355 team project at the University of Victoria. The firmware measures the
frequency of a 555 timer output or a function generator signal on an STM32F0
Discovery board and displays the result on an SSD1306 OLED.

## How it works

- TIM2 measures the interval between consecutive rising edges from the selected input.
- The USER button switches between inputs. TIM3 provides the debounce timing.
- The firmware reads the potentiometer through the ADC and updates the DAC for
  the external 4N35 optocoupler circuit.
- The OLED shows the selected source, measured frequency, and estimated resistance.

## Hardware

- STM32F0 Discovery board with STM32F051R8T6
- PBMCUSLK SSD1306 OLED module
- 555 timer circuit, 4N35 optocoupler, and potentiometer
- Function generator for the reference input

## Build and test

Use the existing ECE 355 Eclipse project configured for the STM32F051R8T6.
Replace that project's `main.c` contents with
[`ECE355_Project_Final_Improved.c`](ECE355_Project_Final_Improved.c).
The firmware uses the project's `cmsis/cmsis_device.h` and `diag/Trace.h`.
The Eclipse project configuration and support files are required for a build
and are outside this package.

See [the lab testing notes](ECE355_Project_Testing_Notes.txt) for setup,
signal levels, calibration assumptions, and bench checks. This firmware
revision is a hardware integration candidate and still needs build and
bench verification.
