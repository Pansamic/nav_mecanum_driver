/**
 * @file motor.h
 * @author pansamic
 * @brief 
 * @version 0.1
 * @date 2023-07-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MOTOR_H
#define __MOTOR_H
#include <stdint.h>
#include <tim.h>
#include <gpio.h>

#define VELOCITY_CONTROL             (1)
#define POSITION_CONTROL             (2)
#define DCMOTOR_REVERSE_DIRECTION    (1)
#define DCMOTOR_DEFAULT_DIRECTION    (0)
/* unit : clock cycle */
#define MOTOR_PWM_PERIOD             (16800)
#define MOTOR_MAX_VELOCITY           (32.0f)
#define PULSE_PER_ROUND              (1560)
#define ENCODER_UPDATE_INTERVAL      (10) // unit:ms

#define ENCODER_REMAIN_TIMERCNT      (0)
#define ENCODER_CLEAR_TIMERCNT       (1)

#define ENCODER_REVERSE              (-1)
#define ENCODER_DEFAULT              (1)

typedef struct PIDTypeDef
{
	float Kp;
	float Ki;
	float Kd;
	float CurrentVelocityBias;
	float LastVelocityBias;
	float BeforeLastVelocityBias;
	float InputVal;
	float OutputVal;
	float FeedBack;

}PID_t;

typedef struct DCMotortypeDef
{
	/* velocity control related */
	PID_t     VelocityController;
	float     CurrentVelocity;
	float     TargetVelocity;
	float     AdjustedVelocity;

	/* position control related */
	PID_t     PositionController;
	float     CurrentAngle;
	float     TargetAngle;
	float     AdjustedAngle;

	/* encoder related */
	TIM_HandleTypeDef *EncTimer;
	short     CurrentCount;
	int8_t    EncReverse;

	/* DC motor attributes */
	uint8_t   RotateDirectionReverse;
	uint8_t   RunningMode;


	/* motor driver specific control logic */
	uint16_t  RotationControlGPIOPin1;
	GPIO_TypeDef  *RotationControlGPIOPort1;
	uint16_t  RotationControlGPIOPin2;
	GPIO_TypeDef  *RotationControlGPIOPort2;

	/* PWM generating module related */
	TIM_HandleTypeDef  *PWMGeneratingTimer;
	uint32_t  PWMGeneratingTimerChannel;
}DCMotor;

extern DCMotor LeftFrontMotor;
extern DCMotor LeftRearMotor;
extern DCMotor RightFrontMotor;
extern DCMotor RightRearMotor;

void        DCMotor_Init                (DCMotor *Motor, uint8_t RotateDirectionReverse, int8_t EncReverse, TIM_HandleTypeDef* PWM_Module, uint32_t PWM_Channel, GPIO_TypeDef* LogicPort1, uint16_t LogicPin1, GPIO_TypeDef* LogicPort2, uint16_t LogicPin2);
void        PID_Init                    (PID_t *PID_instance, float Kp, float Ki, float Kd);
void        PID_Update                  (PID_t *PID_instance, float Input, float Feedback);
void        DCMotor_SetVelocity         (DCMotor *Motor, float Velocity);
void        DCMotor_SetAngle            (DCMotor *Motor, float Angle);
void        DCMotor_Adjust              (DCMotor *Motor);
void        DCMotor_AdjustVelocity      (DCMotor *Motor);
void        DCMotor_AdjustAngle         (DCMotor *Motor);
void        DCMotor_SetPWM              (DCMotor *Motor, int32_t TimerCounterLoadVal);


#endif
