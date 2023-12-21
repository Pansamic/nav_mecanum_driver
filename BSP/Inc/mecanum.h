/*****************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2022-2023, pansamic(Wang GengJie) pansamic@foxmail.com

Filename:    c_4wheel_mecanum.h
Author:      Pansamic
Version:     0.1
Create date: 2023.1.3
Description: The header file that contains all of the information about 4-wheel
    mecanum car model. It contains some applications and driver functions.
Others:      none

History:
1. <author>    <date>                  <desc>
   pansamic  2023.1.3    create v0.1 version.
*****************************************************************************/
#ifndef _MECANUM_H_
#define _MECANUM_H_
#ifdef _cplusplus
extern "C"{
#endif
#include <motor.h>
/**********************************************************************************************
 *                                                                                            *
 *                                         MACROS                                             *
 *                                                                                            *
 **********************************************************************************************/
#define ANGULAR_TO_LINEAR(x) (x*(Car->WheelDiameter/2))
/**********************************************************************************************
 *                                                                                            *
 *                                         TYPEDEF                                            *
 *                                                                                            *
 **********************************************************************************************/
typedef struct Mecanum CarType_t;

struct Mecanum
{
	DCMotor       *LeftFrontMotor;
	DCMotor       *LeftRearMotor;
	DCMotor       *RightFrontMotor;
	DCMotor       *RightRearMotor;

	float          WheelDiameter;            // unit:m

	float         xAxisWheelDistance;        // unit:m
	float         yAxisWheelDistance;        // unit:m

	/* measured value */
	float  CurrentXVelocity;
	float  CurrentYVelocity;
	float  CurrentAngularVelocity;

	/* corrected value */
	float  AdjustedXVelocity;
	float  AdjustedYVelocity;
	float  AdjustedAngularVelocity;

	/* target value */
	float  TargetXVelocity;
	float  TargetYVelocity;
	float  TargetAngularVelocity;
};
/**********************************************************************************************
 *                                                                                            *
 *                                         EXTERN                                             *
 *                                                                                            *
 **********************************************************************************************/
extern CarType_t Car;


/**********************************************************************************************
 *                                                                                            *
 *                                     GLOBAL FUNCTION                                        *
 *                                                                                            *
 **********************************************************************************************/
void    Car_Init               ( CarType_t *Car_instance, float WheelDiameter, float xAxisWheelDistance, float yAxisWheelDistance );
void    Car_MountWheel         ( DCMotor *LeftFront, DCMotor *LeftRear, DCMotor *RightFront, DCMotor *RightRear);
void    Car_SetVelocity        ( CarType_t *Car, float Angle, float Speed, float AngVelocity );
void    Car_AdjustedVelocity   ( CarType_t *Car );
void    Car_VelocityControl    ( CarType_t *Car );


#ifdef _cplusplus
}
#endif
#endif /* C_4WHEEL_MECANUM_H_ */
