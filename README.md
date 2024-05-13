# CPE-301-SP-2024-Final
#5/12/24
#Ivan Uy

#Introduction
  This is the final project for CPE 301, where we're supposed to build a swamp cooler. I have some things working, but not everything. The sensors, LED's and LCD work but not properly. My program gets stuck in one state and cannot switch out of it. I couldn't get the LCD to display the data from the sensors, but it could display "Temperature" and "Humidity" text. The motors completely do not work and I could not figure out how to get them to work. 

#Experimental Design
  The red, blue, green and yellow LEDs are connected to arduino pins 49, 22, 37, and 13 respectively through 330 Ohm resistors. The LEDs indicate the state of the swamp cooler, with red being error, blue being running, green being idle, and yellow being disabled. 
  
  There are 2 push buttons that connect to pins A10 and 38 and have 330 Ohm pull down resistors. The A10 button starts and stops the swamps cooler, while the pin 38 button is a reset button for when the cooler gets stuck in error mode.
  
  From left to right, the LCD's pins are connected to arduino pins ground, power, the potentialmeter's middle pin, 12, 11, nothing, nothing, nothing, nothing, 50, 48, 3, 2, power through a 1k Ohm resistor, and ground respectively. The LCD displays the current temperature and humidity levels. 
  
  The potentialmeter is connected to power, the LCD and ground. This controls the brightness of the LCD.

  The water sensor is connected to ground, pin 9, and pin A2. The water sensor keeps track of the water levels.

  The DHT11 humidity sensor is connected to ground, pin 10, and power. The humidity sensor measures the ambient humidity.

  The DC motor is connected to pins 3 and 6 of the L293D motor driver IC. The DC motor is the fan for the swamp cooler.

  The L293D motor driver IC has pins 1, 2, 4, 5, 7, and 8 connected to arduino pins 23, 25, ground, nothing, 27, and power respectively. This IC controls the DC motor. 

  The 9 volt battery is connected to the power module, which then connected to ground and the ULD2003 stepper motor driver's power. This acts as an external power supply for the stepper motor.
  
  The ULN2003 stepper motor driver is connected to the stepper motor through the keyed socket, and has pins L1, L2, L3, and L4 connected to arduino pins 4, 5, 6, and 7 respectively. This module controls the stepper motor. 

#Results
As explained earlier, some parts work and some parts don't. The motors are the only parts that completely don't work, while some others don't work properly or not as specified. 
  
  
  
