/**
 * @file uxr_client.c
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/******************************************************/
/*                      INCLUDES                      */
/******************************************************/
// #include <time.h>
#include <stdio.h>
#include <string.h>

/* STM32 Peripherals */
#include "rtc.h"
#include "tim.h"

#include "uxr_client.h"
#include "uxr_transport.h"

/* Board Peripheral drivers */
#include "motor.h"
#include "led.h"
#include "icm20602.h"

/******************************************************/
/*                      DEFINES                       */
/******************************************************/
#define STREAM_HISTORY  4
#define STREAM_BUFFER_SIZE UXR_CONFIG_CUSTOM_TRANSPORT_MTU * STREAM_HISTORY
#define IMU_MSG_INDEX 0x01
#define JOINT_STATE_MSG_INDEX 0x02
#define JOINT_JOG_MSG_INDEX 0x03
/******************************************************/
/*                       MACROS                       */
/******************************************************/
/******************************************************/
/*                     VARIABLES                      */
/******************************************************/

const uxrQoS_t reliable_qos = {
    .durability = UXR_DURABILITY_VOLATILE,
    .reliability = UXR_RELIABILITY_RELIABLE,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = 10
};
const uxrQoS_t best_effort_qos = {
    .durability = UXR_DURABILITY_VOLATILE,
    .reliability = UXR_RELIABILITY_BEST_EFFORT,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = 5
};
/* Essentials of client and node */
uxrCustomTransport transport;
uxrSession session;
uxrStreamId reliable_out;
uxrStreamId besteffort_in;
uxrObjectId participant_id;
uint16_t participant_req;
uint8_t output_reliable_stream_buffer[STREAM_BUFFER_SIZE];
uint8_t input_reliable_stream_buffer[STREAM_BUFFER_SIZE];

/* Imu message */
sensor_msgs_msg_Imu msg_imu = {0};
uxrObjectId topic_id_imu;
uxrObjectId publisher_id_imu;
uxrObjectId datawriter_id_imu;

/* JointState message */
sensor_msgs_msg_JointState msg_joint_state = {0};
uxrObjectId topic_id_joint_state;
uxrObjectId publisher_id_joint_state;
uxrObjectId datawriter_id_joint_state;


/* JointJog message */
control_msgs_msg_JointJog msg_joint_jog = {0};
uxrObjectId topic_id_joint_jog;
uxrObjectId subscriber_id_joint_jog;
uxrObjectId datareader_id_joint_jog;

/******************************************************/
/*                     DECLARATION                    */
/******************************************************/
void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args);
/******************************************************/
/*                     DEFINITION                     */
/******************************************************/
uint8_t uxrce_client_init()
{
    // Transport
    uxr_set_custom_transport_callbacks(
    &transport,
    true,
    my_custom_transport_open,
    my_custom_transport_close,
    my_custom_transport_write,
    my_custom_transport_read);
    if (!uxr_init_custom_transport(&transport, NULL))
    {
        printf("[ERROR]Error at create transport.\n");
        return 1;
    }
    while(!uxr_ping_agent_attempts(&transport.comm, 200, 1))
    {
        printf("[INFO] No agent found, retrying...\r\n");
    }
    
    // Session
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    uxr_set_topic_callback(&session, on_topic, NULL);
    if(!uxr_create_session(&session))
    {
        printf("[ERROR]Error at create session.\r\n");
        return 1;
    }

    // Streams
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, STREAM_BUFFER_SIZE, STREAM_HISTORY);
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, STREAM_BUFFER_SIZE, STREAM_HISTORY);
    besteffort_in = uxr_create_input_best_effort_stream(&session);

    // Create entities
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    participant_req = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id, 3, "nav_mecanum_driver", UXR_REPLACE|UXR_REUSE);
    uint8_t status;

    if(uxr_run_session_until_all_status(&session, 1000, &participant_req, &status, 1))
    {
        printf("[INFO] Create participant success: %d.\r\n", status);
    }
    else
    {
        printf("[Error]Create participant failed: %d.\r\n", status);
        return 1;
    }
    uxr_sync_session(&session,1000);
    return 0;
}

uint8_t create_publisher_imu()
{
    memcpy(msg_imu.header.frame_id,"base_link",9);
    msg_imu.orientation_covariance[0] = -1.0;
    msg_imu.orientation_covariance[1] = -1.0;
    msg_imu.orientation_covariance[2] = -1.0;
    msg_imu.orientation_covariance[3] = -1.0;
    msg_imu.orientation_covariance[4] = -1.0;
    msg_imu.orientation_covariance[5] = -1.0;
    msg_imu.orientation_covariance[6] = -1.0;
    msg_imu.orientation_covariance[7] = -1.0;
    msg_imu.orientation_covariance[8] = -1.0;

    topic_id_imu = uxr_object_id(IMU_MSG_INDEX, UXR_TOPIC_ID);
    uint16_t topic_req = uxr_buffer_create_topic_bin(&session, 
                                                     reliable_out,
                                                     topic_id_imu,
                                                     participant_id,
                                                     "rt/imu/data",
                                                     "sensor_msgs::msg::dds_::Imu_",
                                                     UXR_REPLACE|UXR_REUSE);

    publisher_id_imu = uxr_object_id(IMU_MSG_INDEX, UXR_PUBLISHER_ID);
    uint16_t publisher_req = uxr_buffer_create_publisher_bin(&session,
                                                             reliable_out,
                                                             publisher_id_imu,
                                                             participant_id,
                                                             UXR_REPLACE|UXR_REUSE);

    datawriter_id_imu = uxr_object_id(IMU_MSG_INDEX, UXR_DATAWRITER_ID);
    uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&session,
                                                               reliable_out,
                                                               datawriter_id_imu,
                                                               publisher_id_imu,
                                                               topic_id_imu,
                                                               reliable_qos,
                                                               UXR_REPLACE|UXR_REUSE);

    // Send create entities message and wait its status
    uint8_t status[3];
    uint16_t requests[3] = {topic_req, publisher_req, datawriter_req};

    if(uxr_run_session_until_all_status(&session, 1000, requests, status, 3))
    {
        printf("[INFO] Create entities success: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
    }
    else
    {
        printf("[Error]Create entities failed: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
        return 1;
    }
    return 0;
}

uint8_t publish_imu()
{
    ucdrBuffer mb;
    uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&msg_imu, 0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id_imu, &mb, topic_size);
    sensor_msgs_msg_Imu_serialize_topic(&mb, &msg_imu);

    // Reliable QoS
    if(uxr_run_session_until_confirm_delivery(&session, 100))
        return 0;
    else
        return 1;
}


uint8_t create_publisher_joint_state()
{
    msg_joint_state.name_size = 4;
    memcpy(msg_joint_state.name[0],"left_front_wheel_joint",22);
    memcpy(msg_joint_state.name[1],"left_rear_wheel_joint",21);
    memcpy(msg_joint_state.name[2],"right_front_wheel_joint",23);
    memcpy(msg_joint_state.name[3],"right_rear_wheel_joint",22);
    msg_joint_state.position_size = 4;
    msg_joint_state.position[0] = 0.0;
    msg_joint_state.position[1] = 0.0;
    msg_joint_state.position[2] = 0.0;
    msg_joint_state.position[3] = 0.0;
    msg_joint_state.velocity_size = 4;
    msg_joint_state.velocity[0] = 0.0;
    msg_joint_state.velocity[1] = 0.0;
    msg_joint_state.velocity[2] = 0.0;
    msg_joint_state.velocity[3] = 0.0;

    topic_id_joint_state = uxr_object_id(JOINT_STATE_MSG_INDEX, UXR_TOPIC_ID);
    uint16_t topic_req = uxr_buffer_create_topic_bin(&session, 
                                                     reliable_out,
                                                     topic_id_joint_state,
                                                     participant_id,
                                                     "rt/joint_states",
                                                     "sensor_msgs::msg::dds_::JointState_",
                                                     UXR_REPLACE|UXR_REUSE);

    publisher_id_joint_state = uxr_object_id(JOINT_STATE_MSG_INDEX, UXR_PUBLISHER_ID);
    uint16_t publisher_req = uxr_buffer_create_publisher_bin(&session,
                                                             reliable_out,
                                                             publisher_id_joint_state,
                                                             participant_id,
                                                             UXR_REPLACE|UXR_REUSE);

    datawriter_id_joint_state = uxr_object_id(JOINT_STATE_MSG_INDEX, UXR_DATAWRITER_ID);
    uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&session,
                                                               reliable_out,
                                                               datawriter_id_joint_state,
                                                               publisher_id_joint_state,
                                                               topic_id_joint_state,
                                                               reliable_qos,
                                                               UXR_REPLACE|UXR_REUSE);

    // Send create entities message and wait its status
    uint8_t status[3];
    uint16_t requests[3] = {topic_req, publisher_req, datawriter_req};

    if(uxr_run_session_until_all_status(&session, 1000, requests, status, 3))
    {
        printf("[INFO] Create entities success: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
    }
    else
    {
        printf("[Error]Create entities failed: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
        return 1;
    }
    return 0;
}

uint8_t publish_joint_state()
{
    ucdrBuffer mb;
    uint32_t topic_size = sensor_msgs_msg_JointState_size_of_topic(&msg_joint_state, 0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id_joint_state, &mb, topic_size);
    sensor_msgs_msg_JointState_serialize_topic(&mb, &msg_joint_state);

    // Reliable QoS
    if(uxr_run_session_until_confirm_delivery(&session, 100))
        return 0;
    else
        return 1;
}

uint8_t create_subscriber_joint_jog()
{
    topic_id_joint_jog = uxr_object_id(JOINT_JOG_MSG_INDEX, UXR_TOPIC_ID);
    uint16_t topic_req = uxr_buffer_create_topic_bin(&session, 
                                                     reliable_out,
                                                     topic_id_joint_jog,
                                                     participant_id,
                                                     "rt/joint_jog",
                                                     "control_msgs::msg::dds_::JointJog_",
                                                     UXR_REPLACE|UXR_REUSE);
    subscriber_id_joint_jog = uxr_object_id(JOINT_JOG_MSG_INDEX, UXR_SUBSCRIBER_ID);
    uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(&session,
                                                               reliable_out,
                                                               subscriber_id_joint_jog,
                                                               participant_id,
                                                               UXR_REPLACE|UXR_REUSE);
    datareader_id_joint_jog = uxr_object_id(JOINT_JOG_MSG_INDEX, UXR_DATAREADER_ID);
    uint16_t datareader_req = uxr_buffer_create_datareader_bin(&session,
                                                               reliable_out,
                                                               datareader_id_joint_jog,
                                                               subscriber_id_joint_jog,
                                                               topic_id_joint_jog,
                                                               best_effort_qos,
                                                               UXR_REPLACE|UXR_REUSE);
    // Send create entities message and wait its status
    uint8_t status[3];
    uint16_t requests[3] = {topic_req, subscriber_req, datareader_req};

    if(uxr_run_session_until_all_status(&session, 1000, requests, status, 3))
    {
        printf("[INFO] Create entities success: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
    }
    else
    {
        printf("[Error]Create entities failed: topic: %i publisher: %i darawriter: %i\r\n", status[0], status[1], status[2]);
        return 1;
    }
    uxrDeliveryControl delivery_control = {0};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session, reliable_out, datareader_id_joint_jog, besteffort_in, &delivery_control);
    return 0;
}


void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length;
    control_msgs_msg_JointJog_deserialize_topic(ub, &msg_joint_jog);
	DCMotor_SetVelocity(&LeftFrontMotor, (float)msg_joint_jog.velocities[0]);
	DCMotor_SetVelocity(&LeftRearMotor, (float)msg_joint_jog.velocities[1]);
	DCMotor_SetVelocity(&RightFrontMotor, (float)msg_joint_jog.velocities[2]);
	DCMotor_SetVelocity(&RightRearMotor, (float)msg_joint_jog.velocities[3]);
	printf("[INFO] recv vel: 1:%f | 2:%f | 3:%f | 4:%f\r\n",
			LeftFrontMotor.TargetVelocity,
			LeftRearMotor.TargetVelocity,
			RightFrontMotor.TargetVelocity,
			RightRearMotor.TargetVelocity);
}