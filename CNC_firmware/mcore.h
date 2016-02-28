#ifndef __MCORE_H__
#define __MCORE_H__

#include "hardware_config.h"
#include "statuses.h"

typedef enum {
	ROUND_CW,
	ROUND_CCW
} InterpolationDirection;

typedef enum {
	PLANE_XY,
	PLANE_ZX,
	PLANE_YZ
} InterpolationPlane;

typedef enum {
	HW_NONE,
	HW_TABLE,
	HW_EXTRUDER,
	HW_TABLEEXTRUDER,
	HW_CALIBRATION
} HEATER_WAIT;

#define AXIS_MIN_SPEED 0.1f
#define SPINDLE_PROBE_MODE -999.0f
#define SPINDLE_OFF -1
#define AXIS_MAX_SPEED 99999.0f
#define AXIS_MAX_SPEED_LOW_ACCELERATION 99998.0f
#define AXIS_CONTINIOUS AXIS_MAX_SPEED
#define TACTS_PER_SEC 12000 // как часто дергается по таймеру TIM7_IRQHandler(),  т.к 400 импульсов на милиметр, максимальная скорость 1800 мм/мин = 30 мм/сек = 12000 импульсов/сек, должно делиться на частоту проца
#define HEATER_OFF -300 // подогрев выключен
#define HEATER_NOT_CONNECTED_TEMP 10

void axis_init();
STATUSES axis_foundzero();
STATUSES axis_isparked();
STATUSES axis_movefromsensors();
STATUSES axis_move_linear(float dx, float dy, float dz, float de, float da, float speed, float laserPower);
STATUSES axis_move_to_linear(float x, float y, float z, float e, float a, float speed, float laserPower);
STATUSES axis_park();
void axis_stop();
void axis_estop();
STATUSES axis_status();
STATUSES axis_pause(int ms, float laserPower);
STATUSES axis_round(float dx, float dy, float dz, float de, float da, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower);
STATUSES axis_round_to(float tx, float ty, float tz, float te, float ta, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower);
STATUSES axis_changeTool();
STATUSES axis_toolChanged();
STATUSES axis_zprobe(float dz);
STATUSES axis_zprobe_to(float z);
STATUSES axis_stopPause();
STATUSES axis_stopResume();
int laser_isWorking();
STATUSES laser_force(float laserPower, int ms);
void laser_force_stop();

float axis_getXPos();
float axis_getYPos();
float axis_getZPos();
float axis_getEPos();
float axis_getAPos();
void axis_setEPos(float e);
void axis_setAPos(float a);

extern unsigned int private_spindle_result;
STATUSES spindle_runp(float percent);
STATUSES spindle_runr(int rpm);
STATUSES spindle_stop();
int spindle_get_rpm();

void raxis_lock_relative_zero_1();
void raxis_lock_relative_zero_1z();
void raxis_free_relative_zero_1();
void raxis_set_relative_zero_2(float x, float y, float z, float e, float a);
STATUSES raxis_move_to_linear(float x, float y, float z, float e, float a, float speed, float laserPower);
STATUSES raxis_round_to(float x, float y, float z, float e, float a, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower);
STATUSES raxis_zprobe_to(float z);
float raxis_get_zero_1_x();
float raxis_get_zero_1_y();
float raxis_get_zero_1_z();
float raxis_get_zero_2_x();
float raxis_get_zero_2_y();
float raxis_get_zero_2_z();
float raxis_get_zero_2_e();
float raxis_get_zero_2_a();
float raxis_getXPos();
float raxis_getYPos();
float raxis_getZPos();
float raxis_getEPos();
float raxis_getAPos();

void private_heater_init();
STATUSES heater_table_heat(int temperature);
int heater_get_table_temperature();
STATUSES heater_table_heat_off();
int  heater_measured_table_temp();
STATUSES heater_table_reached();
STATUSES heater_extruder_heat(int temperature);
int heater_get_extruder_temperature();
STATUSES heater_extruder_heat_off();
int  heater_measured_extruder_temp();
STATUSES heater_extruder_reached();
STATUSES heater_extruder_ready();
STATUSES heaters_wait(HEATER_WAIT what);
HEATER_WAIT heaters_wait_status();
STATUSES heaters_calibrate();
STATUSES heaters_lastCalibrationStatus();
void cooler_laserActivate();

#endif // __MCORE_H__
