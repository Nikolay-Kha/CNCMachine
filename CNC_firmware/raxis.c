#include "mcore.h"

float razero1X = 0.0f;
float razero1Y = 0.0f;
float razero1Z = 0.0f;
float razero2X = 0.0f;
float razero2Y = 0.0f;
float razero2Z = 0.0f;
float razero2E = 0.0f;
float razero2A = 0.0f;


void raxis_lock_relative_zero_1() // для установки нуля с экрана, координаты берет текущие
{
	razero1X = axis_getXPos();
	razero1Y = axis_getYPos();
	razero1Z = axis_getZPos();
}

void raxis_lock_relative_zero_1z()
{
	razero1Z = axis_getZPos();
}

void raxis_free_relative_zero_1()
{
	razero1X = 0.0f;
	razero1Y = 0.0f;
	razero1Z = 0.0f;
}

void raxis_set_relative_zero_2(float x, float y, float z, float e, float a) // для относительных координат из gcode
{
	razero2X = x;
	razero2Y = y;
	razero2Z = z;
	razero2E = e;
	razero2A = a;
}

float raxis_getXPos()
{
	return axis_getXPos() - razero1X - razero2X;
}

float raxis_getYPos()
{
	return axis_getYPos() - razero1Y - razero2Y;
}

float raxis_getZPos()
{
	return axis_getZPos() - razero1Z - razero2Z;
}

float raxis_getEPos()
{
	return axis_getEPos() - razero2E;
}

float raxis_getAPos()
{
	return axis_getAPos() - razero2A;
}

STATUSES raxis_move_to_linear(float x, float y, float z, float e, float a, float speed, float laserPower)
{
	return axis_move_to_linear(razero1X+razero2X+x, razero1Y+razero2Y+y, razero1Z+razero2Z+z, razero2E+e,  razero2A+a, speed, laserPower);
}

STATUSES raxis_zprobe_to(float z)
{
	return axis_zprobe_to(razero1Z+razero2Z+z);
}

STATUSES raxis_round_to(float x, float y, float z, float e, float a, float ri, float rj, float rk, float speed, InterpolationDirection direction, InterpolationPlane plane, float laserPower)
{
	return axis_round_to(razero1X+razero2X+x, razero1Y+razero2Y+y, razero1Z+razero2Z+z, razero2E+e, razero2A+a, ri, rj, rk, speed, direction, plane, laserPower);
}

float raxis_get_zero_1_x()
{
	return razero1X;
}

float raxis_get_zero_1_y()
{
	return razero1Y;
}

float raxis_get_zero_1_z()
{
	return razero1Z;
}

float raxis_get_zero_2_x()
{
	return razero2X;
}

float raxis_get_zero_2_y()
{
	return razero2Y;
}

float raxis_get_zero_2_z()
{
	return razero2Z;
}

float raxis_get_zero_2_e()
{
	return razero2E;
}

float raxis_get_zero_2_a()
{
	return razero2A;
}
