/*****************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2022-2023, pansamic(Wang GengJie) pansamic@foxmail.com

Filename:    c_4wheel_mecanum.h
Author:      Pansamic
Version:     0.1
Create date: 2023.1.3
Description: This file contains realization of driver functions and applications
    of 4-wheel mecanum car model.
Others:      none

History:
1. <author>    <date>                  <desc>
   pansamic  2023.1.3    create v0.1 version.
*****************************************************************************/
#ifdef _cplusplus
extern "C"{
#endif

#include <math.h>
#include <mecanum.h>

#define DisToAngle(Radius,Displace) ((Displace)/(Radius))
#define LinVelToAngVel(Radius,LinearVel)  ((LinearVel)/(Radius))
#define AngVelToLinVel(Radius,AngularVel) ((Radius)*(AngularVel))

CarType_t Car;


/*****************************************************************************************************
 * @name:Car_Init
 * @brief:Set all parameters related to kinemactics to zero and load some neccessary parameters to
 *     4-wheel mecanum car model.
 * @params:
 *     1.Car_instance:A pointer of car instance.
 *     2.WheelDiameter:The diameter of wheel.
 *     3.xAxisWheelDistance:The distance between the center of the front and rear wheels.
 *     4.yAxisWheelDistance:The distance between the center of the left and right wheels.
 * @retval:none
 * @author: Wang Geng Jie
 *****************************************************************************************************/
void Car_Init(CarType_t *Car_instance, float WheelDiameter, float xAxisWheelDistance, float yAxisWheelDistance)
{
	/* set the motor pointer as NULL to avoid wild pointer.
	 * when function 'Car_AddWheel()' is called, the pointer
	 * will be correct value. Many of functions check whether
	 * DC motor is added by examining whether the pointer is
	 * NULL or not. */
	Car_instance->LeftFrontMotor = NULL;
	Car_instance->LeftRearMotor  = NULL;
	Car_instance->RightFrontMotor= NULL;
	Car_instance->RightRearMotor = NULL;

	Car_instance->WheelDiameter = WheelDiameter;

	Car_instance->xAxisWheelDistance = xAxisWheelDistance;
	Car_instance->yAxisWheelDistance = yAxisWheelDistance;

	/* All kinematic-related parameters are set to zero to avoid
	 * uninitialized random values affecting the initial state
	 * of the kinematic model. */
	Car_instance->CurrentXVelocity        = 0;
	Car_instance->CurrentYVelocity        = 0;
	Car_instance->CurrentAngularVelocity  = 0;

	Car_instance->AdjustedXVelocity       = 0;
	Car_instance->AdjustedYVelocity       = 0;
	Car_instance->AdjustedAngularVelocity = 0;

	Car_instance->TargetXVelocity         = 0;
	Car_instance->TargetYVelocity         = 0;
	Car_instance->TargetAngularVelocity   = 0;
}
/*****************************************************************************************************
 * @name:Car_MountWheel
 * @brief:set pointer of motor as the pointer of 'DCMotor' type of struct.
 * @params:
 *     1.LeftFront:The pointer of left front 'DCMotor' struct.
 *     2.LeftRear:The pointer of left rear 'DCMotor' struct.
 *     3.RightFront:The pointer of right front 'DCMotor' struct.
 *     4.RightRear:The pointer of right rear 'DCMotor' struct.
 * @retval:none
 * @author: Wang Geng Jie
 *****************************************************************************************************/
void Car_MountWheel(DCMotor *LeftFront, DCMotor *LeftRear, DCMotor *RightFront, DCMotor *RightRear)
{
	Car.LeftFrontMotor  = LeftFront;
	Car.LeftRearMotor   = LeftRear;
	Car.RightFrontMotor = RightFront;
	Car.RightRearMotor  = RightRear;
}

/*****************************************************************************************************
 * @name:Car_SetVelocity
 * @brief:set target velocity of car to input value.
 * @params:
 *     1.Car:Pointer of 'CarType_t' instance.
 *     2.Angle:Move towards the direction of 'Angle' counterclockwise with the front as the reference.
 *     3.Velocity:Linear velocity of car.
 *     4.AngVelocity:Angular velocity of car.
 * @retval:none
 * @author: Wang Geng Jie
 *****************************************************************************************************/
void Car_SetVelocity(CarType_t *Car, float Angle, float Velocity, float AngVelocity)
{
	Car->TargetYVelocity = Velocity * sinf(Angle);
	Car->TargetXVelocity = Velocity * cosf(Angle);
	Car->TargetAngularVelocity = AngVelocity;
}

/*****************************************************************************************************
 * @name:Car_AdjustedVelocity
 * @brief:Adjust attitude and apply velocity to wheels.
 * @params:
 *     1.Car:Pointer of 'CarType_t' instance.
 * @retval:none
 * @author: Wang Geng Jie
 *****************************************************************************************************/
void Car_AdjustedVelocity(CarType_t *Car)
{
	/* caution:this is the velocity of wheel ranther than motor,
	 * so its unit is cm/s rather than rad/s */
	float LeftFrontWheelAngularVelocity   = 0;  // unit:rad/s
	float LeftRearWheelAngularVelocity    = 0;  // unit:rad/s
	float RightFrontWheelAngularVelocity  = 0;  // unit:rad/s
	float RightRearWheelAngularVelocity   = 0;  // unit:rad/s
	float LeftFrontWheelVelocity          = 0;  // unit:m/s
	float LeftRearWheelVelocity           = 0;  // unit:m/s
	float RightFrontWheelVelocity         = 0;  // unit:m/s
	float RightRearWheelVelocity          = 0;  // unit:m/s
	float xAxisVelocity                   = 0;  // unit:m/s
	float yAxisVelocity                   = 0;  // unit:m/s

	/* closed-loop control */
	Car_VelocityControl(Car);

	xAxisVelocity = Car->AdjustedXVelocity;
	yAxisVelocity = Car->AdjustedYVelocity;

	LeftFrontWheelVelocity = xAxisVelocity - yAxisVelocity + Car->AdjustedAngularVelocity * (Car->yAxisWheelDistance + Car->xAxisWheelDistance);
	LeftRearWheelVelocity  = xAxisVelocity + yAxisVelocity + Car->AdjustedAngularVelocity * (Car->yAxisWheelDistance + Car->xAxisWheelDistance);
	RightFrontWheelVelocity  = xAxisVelocity + yAxisVelocity - Car->AdjustedAngularVelocity * (Car->yAxisWheelDistance + Car->xAxisWheelDistance);
	RightRearWheelVelocity   = xAxisVelocity - yAxisVelocity - Car->AdjustedAngularVelocity * (Car->yAxisWheelDistance + Car->xAxisWheelDistance);

	LeftFrontWheelAngularVelocity  = LinVelToAngVel(Car->WheelDiameter/2,LeftFrontWheelVelocity);
	LeftRearWheelAngularVelocity   = LinVelToAngVel(Car->WheelDiameter/2,LeftRearWheelVelocity );
	RightFrontWheelAngularVelocity = LinVelToAngVel(Car->WheelDiameter/2,RightFrontWheelVelocity);
	RightRearWheelAngularVelocity  = LinVelToAngVel(Car->WheelDiameter/2,RightRearWheelVelocity);

	DCMotor_SetVelocity(Car->LeftFrontMotor, LeftFrontWheelAngularVelocity );
	DCMotor_SetVelocity(Car->LeftRearMotor,  LeftRearWheelAngularVelocity  );
	DCMotor_SetVelocity(Car->RightFrontMotor,RightFrontWheelAngularVelocity);
	DCMotor_SetVelocity(Car->RightRearMotor, RightRearWheelAngularVelocity );

	DCMotor_AdjustVelocity(Car->LeftFrontMotor );
	DCMotor_AdjustVelocity(Car->LeftRearMotor  );
	DCMotor_AdjustVelocity(Car->RightFrontMotor);
	DCMotor_AdjustVelocity(Car->RightRearMotor );
}

/*****************************************************************************************************
 * @name:Car_VelocityControl
 * @brief:Adjust the attitude and motion with attitude sensor.
 * @params:
 *     1.Car:Pointer of 'CarType_t' instance.
 * @retval:none
 * @author: Wang Geng Jie
 *****************************************************************************************************/
void Car_VelocityControl(CarType_t *Car)
{
	/* The driver for MPU has not yet been developed yet,
	 * so velocity control has nothing to do */
	Car->AdjustedAngularVelocity = Car->TargetAngularVelocity;
	Car->AdjustedXVelocity       = Car->TargetXVelocity;
	Car->AdjustedYVelocity       = Car->TargetYVelocity;

	Car->CurrentXVelocity = AngVelToLinVel(Car->WheelDiameter/2,(Car->LeftFrontMotor->CurrentVelocity + Car->RightRearMotor->CurrentVelocity + Car->LeftRearMotor->CurrentVelocity + Car->RightFrontMotor->CurrentVelocity))/4.0;
	Car->CurrentYVelocity = AngVelToLinVel(Car->WheelDiameter/2,(Car->LeftFrontMotor->CurrentVelocity + Car->RightRearMotor->CurrentVelocity - Car->LeftRearMotor->CurrentVelocity - Car->RightFrontMotor->CurrentVelocity))/4.0;
	Car->CurrentAngularVelocity = AngVelToLinVel(Car->WheelDiameter/2,(Car->RightFrontMotor->CurrentVelocity + Car->RightRearMotor->CurrentVelocity - Car->LeftFrontMotor->CurrentVelocity - Car->LeftRearMotor->CurrentVelocity))/(4.0*(Car->xAxisWheelDistance + Car->yAxisWheelDistance));
}

#ifdef _cplusplus
}
#endif
