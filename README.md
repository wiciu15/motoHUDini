# motoHUDini
Digital speedometer and instrument cluster for motorcycle. Based on STM32F103 (blue-pill board) and ILI9341 TFT.

Written using Cube HAL and CubeIDE


Done for now:
+FreeRTOS
+TFT library and SPI configuration
+working timers and interrupts for measuring kmh and RPM
+viewing values on TFT screen

Work still in progress:
-mileage, average speed etc. calculation
-stepper motor as RPM meter
-GPIO for controls (blinkers, headlights etc.)
-temperature sensor (PT100 with amplifier connected to ADC)
-nice little GUI
-EEPROM for storing configuration and data after powering off
-configuration of device via computer to STM32's USB CDC (Virtual COM Port)
