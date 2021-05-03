# LeaKDetector

## Group Members 
Bruce Englert 

Mitch Talmedge 

## Purpose 

The purpose of this project to use an STM32F0 Discovery Board in combination with a ADMP401 MEMS Microphone Breakout Module to detect the passage of water through a pipe to identify the appliance being used as well as potential leaks in the house. 

## Functionality 

As of May 2, 2021 the project currently is able to detect the passage of water through the pipe, identify 3 specific use cases in Bruce's home, and recognize potential leaks. 

## Setup 
Required Hardware: 
* STM32F072 Discovery kit 
* ADMP401 MEMS Microphone Breakout Module Board  ([amazon](link:[englert.bruce@utah.edu](https://www.amazon.com/dp/B07W5Z9NJD?psc=1&ref=ppx_yo2_dt_b_product_details))) 
* Jumper Cables ([amazon](link:[englert.bruce@utah.edu](https://www.amazon.com/dp/B01EV70C78?psc=1&ref=ppx_yo2_dt_b_product_details))) 
* UART Cable

Pinout: 
* PA0 - ADC In (Microphone)
* PB10 - UART TX
* 3V - + (Microphone)

## Instructions  
In firmware/Src/main.c you can set the ALARM value for how long water can pass through the pipe before the behavior is seen as a leak. You may also set threshold values for behaviors determined by your own environment (pipe material, water pressure, etc. ). 

Data can be read via the UART TX to your pc. This data can be graphed using graph.py in order to help determine the threshold values for the homes environment. Included is an example of the UART output as a screen log. 
