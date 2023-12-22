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

#include "sensor_msgs/msg/Imu.h"
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

/******************************************************/
/*                      FUNCTIONS                     */
/******************************************************/
uint8_t uxrce_client_init();
uint8_t create_publisher(const char * topic_name, const char * data_type);
uint8_t create_publisher_imu();
uint8_t msg_publish(const void* pmsg);
#ifdef __cplusplus
}
#endif
#endif
