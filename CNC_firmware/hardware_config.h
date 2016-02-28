#ifndef __HARDWARE_CONFIG_H_
#define __HARDWARE_CONFIG_H_
/*
 *   Hardware setting for machine
 */

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>

uint32_t getRcc(GPIO_TypeDef* GPIOx);


// Motors ------------------------------------------------------------------------------------------------------------------------
#define PULSE_LENGHT_uS 3
#define AXIS_CALLIBRATION_SPEED 700 // ~400 for 3-axis system - max speed for non acceleration movement while calibration

#define SPINDLE_PORT GPIOC // be careful, timer TIM8 is connected to this port-pin
#define SPINDLE_PWM_PIN GPIO_Pin_6;

#define AXIS_PORT GPIOG
#define AXIS_X_STEPPIN GPIO_Pin_2
#define AXIS_X_DIRPIN GPIO_Pin_3
#define AXIS_Y_STEPPIN GPIO_Pin_4
#define AXIS_Y_DIRPIN GPIO_Pin_5
#define AXIS_Z_STEPPIN GPIO_Pin_6
#define AXIS_Z_DIRPIN GPIO_Pin_7
#define AXIS_E_STEPPIN GPIO_Pin_9
#define AXIS_E_DIRPIN GPIO_Pin_10
#define AXIS_A_STEPPIN GPIO_Pin_9
#define AXIS_A_DIRPIN GPIO_Pin_10

#define PULSES_PER_MM_X 400
#define PULSES_PER_MM_Y 400
#define PULSES_PER_MM_Z 400
#define PULSES_PER_MM_E 96
#define PULSES_PER_MM_A 96

#define AXIS_E_DRIVER_ENABLE_PORT GPIOA
#define AXIS_E_DRIVER_ENABLE_PIN GPIO_Pin_12


// Heaters and cooling --------------------------------------------------------------------------------------------------------------
#define HEATERS_PORT GPIOC
#define EXTRUDER_HEATER_PIN  GPIO_Pin_10
#define TABLE_HEATER_PIN GPIO_Pin_11
#define COOLING_PORT GPIOC
#define COOLING_PIN GPIO_Pin_12
#define EXTRUDER_COOLING_TEMP 50
#define HEATERS_DATA_VEEPROM_PAGE 250

// Sensors -------------------------------------------------------------------------------------------------------------------------
#define TEMPERATURES_SENSORS_PORT GPIOA // be careful, ADC is connected to this sensors
#define TABLE_SENSOR_CHANNEL ADC_Channel_1
#define TABLE_SENSOR_MATRIX_VALUES {4096, 4096, 3850, 3480, 3210, 2970, 2670, 2300, 1948, 1478, \
		1210, 1020, 808, 697, 550, 447, 382, 303, 278, 244, \
		210, 187, 167, 155, 144, 136, 128, 120, 109, 102, \
		94, 84, 79, 76, 0}
#define TABLE_SENSOR_PIN GPIO_Pin_1
#define EXTRUDER_SENSOR_CHANNEL ADC_Channel_2
#define EXTRUDER_SENSOR_MATRIX_VALUES {4096, 4096, 3850, 3480, 3210, 2970, 2670, 2300, 1948, 1478, \
		1210, 1020, 808, 697, 550, 447, 382, 303, 278, 244, \
		210, 187, 167, 155, 144, 136, 128, 120, 109, 102, \
		94, 84, 79, 76, 0}
#define EXTRUDER_SENSOR_PIN GPIO_Pin_2

#define AXIS_PORT_SENSE GPIOB
#define AXIS_XSENSOR_PIN GPIO_Pin_12
#define AXIS_XSENSOR_POSITION 1
#define AXIS_XINVERTED 1
#define AXIS_YSENSOR_PIN GPIO_Pin_13
#define AXIS_YSENSOR_POSITION -1
#define AXIS_YINVERTED 1
#define AXIS_ZSENSOR_PIN GPIO_Pin_14
#define AXIS_ZSENSOR_POSITION 1
#define AXIS_ZINVERTED -1
#define AXIS_ESENSOR_POSITION 1
#define AXIS_EINVERTED 1
#define AXIS_ASENSOR_POSITION 1
#define AXIS_AINVERTED 1

#define SPINDLE_SENS_PIN GPIO_Pin_15

// Beepers, relays and other things--------------------------------------------------------------------------------------------------
#define BEEPER_PORT GPIOD
#define BEEPER_PIN GPIO_Pin_3

#define USBUARTPORT GPIOA // be careful, USART1 is connected to this port-pins
#define USBUARTTXPIN GPIO_Pin_9
#define USBUARTRXPIN GPIO_Pin_10

#define USBHOSTUARTPORT GPIOB // be careful, USART3ws is connected to this port-pins
#define USBHOSTUARTTXPIN GPIO_Pin_10
#define USBHOSTUARTRXPIN GPIO_Pin_11

#define DEBUGLED_PORT GPIOE
#define CPULEDPIN GPIO_Pin_0
#define CPUOVERUSAGELEDPIN GPIO_Pin_2

#define ESTOPBUTTONPORT GPIOD
#define ESTOPBUTTONPIN GPIO_Pin_2 // be careful, INT line 2 is connected to this port-pins

#define TOUCHSCREEN_DATA_VEEPROM_PAGE 251

#define LASER_PORT GPIOA
#define LASER_PIN GPIO_Pin_11

// Limits -------------------------------------------------------------------------------------------------------------------------
#define SPINDLE_MAX_RPM 10000
#define SPINDLE_MIN_RPM 1000

#define X_ACCELERATION 200*PULSES_PER_MM_X // pulse per s^2, 200 mm per sec^2
#define Y_ACCELERATION 200*PULSES_PER_MM_Y // pulse per s^2, 200 mm per sec^2
#define Z_ACCELERATION 100*PULSES_PER_MM_Z // pulse per s^2, 100 mm per sec^2
#define E_ACCELERATION 200*PULSES_PER_MM_E // pulse per s^2, 100 mm per sec^2
#define A_ACCELERATION 200*PULSES_PER_MM_A // pulse per s^2, 100 mm per sec^2
#define X_MAXVELOCITY 1800 // mm/min
#define Y_MAXVELOCITY 1800 // mm/min
#define Z_MAXVELOCITY 700 // mm/min
#define E_MAXVELOCITY 700 // mm/min
#define A_MAXVELOCITY 2400 // degrees/min


#define AXIS_X_TABLE_SIZE_MM 200
#define AXIS_Y_TABLE_SIZE_MM 300
#define AXIS_Z_TABLE_SIZE_MM 48

#define MAX_TABLE_TEMPERATURE 125
#define MAX_EXTRUDER_TEMPERATURE 280

#define LASER_OPTICAL_POWER_mW 300
#define LASER_MIN_PULSE_LENGTH_CPU_TACTS 280 // 240 for laser driver module and optocoupler, 3 for CPU
#define LASER_MIN_PULSE_LENGTH_LOW_ACCURACITY_DIV 2
#define LASER_MIN_PWM_FREQUENCY_HZ 100

#endif //__HARDWARE_CONFIG_H_

