/**
 * @file uxr_client.h
 * @author pansamic(pansamic@fxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __UXR_CLIENT_H__
#define __UXR_CLIENT_H__
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************/
/*                      INCLUDES                      */
/******************************************************/
#include <stdint.h>
#include <uxr/client/client.h>
#include <uxr/client/util/ping.h>
#include <ucdr/microcdr.h>
#include <sensor_msgs/msg/Imu.h>
#include <sensor_msgs/msg/JointState.h> 
#include <control_msgs/msg/JointJog.h>

/******************************************************/
/*                      TYPEDEF                       */
/******************************************************/

/******************************************************/
/*                      DEFINES                       */
/******************************************************/


/******************************************************/
/*                       MACROS                       */
/******************************************************/

/******************************************************/
/*                     VARIABLES                      */
/******************************************************/
extern sensor_msgs_msg_Imu msg_imu;
extern sensor_msgs_msg_JointState msg_joint_state;
extern uxrSession session;
/******************************************************/
/*                      FUNCTIONS                     */
/******************************************************/
uint8_t uxrce_client_init();

uint8_t create_publisher_imu();
uint8_t publish_imu();

uint8_t create_publisher_joint_state();
uint8_t publish_joint_state();

uint8_t create_subscriber_joint_jog();
#ifdef __cplusplus
}
#endif
#endif
