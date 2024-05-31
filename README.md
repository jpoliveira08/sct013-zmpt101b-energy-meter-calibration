# Explanation

This code give the values read by the ESP32.

# How to use

This code will give to you the Voltage, Current and Real Power read by the ESP32, so will give values in the interval 0 to 3,3.

So its recommended use diffent values of voltage and current, to build a sample data for a linear regression.

Example: 

Voltage applied: 0V - Voltage read: 0.02501

Change the applied voltage to 20V, and press the "EN" on the esp32 board. With that, a new voltage read should be printed in the screen.

Voltage applied: 20V - Voltage read: 0.16401

So each time that you want to generate a new sample, the setup function should be executed by pressing the "EN" button on the esp32. This should be done to the current and the real power.