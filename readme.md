# CNC Machine
Firmware for STM32F10x(tested with STM32F103ZET6) MCU to control different CNC
machines. Can be used with engraving, 3D printering and laser cuting machines.  
Features:
- 3 axis stepper motor control
- limit switches support 
- 3D printer extruder stepper motor control
- hot table and extruder PID control
- spindle PWM control for engraving
- laser cutter PWM control
- touch LCD support for mannual control and status displaying
- Z position billet detection
- text mode serial interfaces, G code can be sent directly from any platfrom
- usb host for keyboards and usb flash drives (beta, additional MCU require)


# Building from sources
Project can be openned with CooCox IDE ( http://coocox.org/ ) and build with 
GCC ARM Embedded ( https://launchpad.net/gcc-arm-embedded ).  
GCodeSender is a Qt console project, it can be openned with QtCreator.

# Hardware
Firmware can be apadpted for various hardware. There is a configuration file 
"CNC/hardware_config.h" which contains pins and hardware configuration. You are 
free to remap pins and configure firmware for your hardware setup. That's why 
there isn't precise circuit diagram for machine. You can use A4988, tb6600 
or any other stepper motor driver, heaters with different power,any CNC machine 
frames. Everything is configurable. Basically you need to connect limit sensors, 
three stepper motor drivers, USB to UART converter and LCD display. Everything 
else is optional.  
Notes:  
- Motors section contains pin map for driver connection(direction pin and step 
pin) and spindle connection. There is also settings for pulse width and number 
of pulses per milimeter. 'E' axis is an extruder's stepper motor for fillament. 
'A' axis is an additional 'rotary' axis for CNC machines. But it was never 
tested.
- Heaters and cooling contains pin map for extruder and table heaters and 
coolers (laser cooler connected to the same pin as extruder cooler).
- Sensor section contains pin map for limit sensors and temperature sensor. 
Matrixes contains values of ADC for this sensors. First value is 0 celsius 
degree and each next with 10 degree step.
- Beepers, relays and other setcion contains pin map for Beeper, serial ports (
one port for communication with computer to send gcode commands, another serial 
to comminicate with USB host chip for usb keyboard and usb storage support), 
laser pinmap, emergency stop button and two status leds. Status leds are mostly 
for debug purpose. CPULED blink with sime frequency when machine in idle and 
this frequency goes down when cpu load grow up. OVERUSAGELED lights when CPU is 
overloaded and there isn't enough CPU power to handle axis and stepper motors 
signal contains jitters, in other  words this LED shoudn't light if everything 
configured correctly.
- LCD is conntected to chip's FSMC (see chip pinout for details).
- Limits section contains hardware limit for connected devices.

# Hardware sample

# Author
Khabarov Nikolay (c) 2014