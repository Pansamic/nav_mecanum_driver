/**
 * @file microros.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief micro-ROS related header file
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MICROROS_H__
#define __MICROROS_H__
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************/
/*                      INCLUDES                      */
/******************************************************/
/* micro ros kernel */
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rclc_parameter/rclc_parameter.h"

/* micro ros messages */
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/time_reference.h"
//#include "tf2_msgs/msg/tf_message.h"
//#include "geometry_msgs/msg/twist.h"
//#include "nav_msgs/msg/odometry.h"
#include "sensor_msgs/msg/joint_state.h"
#include "control_msgs/msg/joint_jog.h"

/******************************************************/
/*                      TYPEDEF                       */
/******************************************************/
typedef struct joint_states_t
{
	rosidl_runtime_c__String joint_names[4];
	double position[4];
	double velocity[4];
	double effort[4];

}joint_states_t;

typedef struct joint_jog_t
{
	char names[4][32];
	rosidl_runtime_c__String joint_names[4];
	double displacements[4];
	double velocities[4];
}joint_jog_t;
/******************************************************/
/*                      DEFINES                       */
/******************************************************/


/******************************************************/
/*                       MACROS                       */
/******************************************************/


/******************************************************/
/*                     VARIABLES                      */
/******************************************************/
extern rclc_executor_t executor;

extern rcl_publisher_t pub_imu;
//extern rcl_publisher_t pub_odometry;
//extern rcl_publisher_t pub_tf2;
extern rcl_publisher_t pub_joint_states;

//extern nav_msgs__msg__Odometry msg_odometry;
extern sensor_msgs__msg__Imu msg_imu;
//extern geometry_msgs__msg__Twist msg_twist;
//extern sensor_msgs__msg__TimeReference msg_time_ref;
//extern tf2_msgs__msg__TFMessage msg_tf2;
//extern geometry_msgs__msg__TransformStamped tf2_data;
extern sensor_msgs__msg__JointState msg_joint_state;
extern joint_states_t joint_states;
/******************************************************/
/*                      FUNCTIONS                     */
/******************************************************/
void microros_init(void);
void microros_spinsome(void);

#ifdef __cplusplus
}
#endif
#endif
