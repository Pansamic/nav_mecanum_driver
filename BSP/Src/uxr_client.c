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
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>


/* Board Peripheral drivers */
#include "motor.h"
#include "led.h"
#include "icm20602.h"
#include "mecanum.h"

/******************************************************/
/*                      DEFINES                       */
/******************************************************/
#define STREAM_HISTORY  4
#define STREAM_BUFFER_SIZE UXR_CONFIG_CUSTOM_TRANSPORT_MTU * STREAM_HISTORY

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

uxrCustomTransport transport;
uxrSession session;
uxrStreamId reliable_out;

uxrObjectId participant_id;
uint16_t participant_req;
uxrObjectId datawriter_id;
uint16_t datawriter_req;



sensor_msgs_msg_Imu msg_imu = {0};

uint8_t output_reliable_stream_buffer[STREAM_BUFFER_SIZE];
uint8_t input_reliable_stream_buffer[STREAM_BUFFER_SIZE];
/******************************************************/
/*                     DECLARATION                    */
/******************************************************/

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
        printf("Error at create transport.\n");
        return 1;
    }
    // Session

    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    // uxr_set_topic_callback(&session, uxr_onTopicCallback, NULL);
    if(!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return 1;
    }

    // Streams
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, STREAM_BUFFER_SIZE, STREAM_HISTORY);
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, STREAM_BUFFER_SIZE, STREAM_HISTORY);
        // Create entities
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    participant_req = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id, 3, "nav_mecanum_driver", UXR_REPLACE|UXR_REUSE);

    uxr_sync_session(&session,1000);
    return 0;
}

uint8_t create_publisher(const char * topic_name, const char * data_type)
{
    static uint16_t publisher_index = 0;

    uxrObjectId topic_id = uxr_object_id(publisher_index, UXR_TOPIC_ID);
    // uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE|UXR_REUSE);
    uint16_t topic_req = uxr_buffer_create_topic_bin(&session, reliable_out, topic_id, participant_id, topic_name, data_type, UXR_REPLACE|UXR_REUSE);

    uxrObjectId publisher_id = uxr_object_id(publisher_index, UXR_PUBLISHER_ID);
    // uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE|UXR_REUSE);
    uint16_t publisher_req = uxr_buffer_create_publisher_bin(&session, reliable_out, publisher_id, participant_id, UXR_REPLACE|UXR_REUSE);

    datawriter_id = uxr_object_id(publisher_index++, UXR_DATAWRITER_ID);
    // datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE|UXR_REUSE);
    datawriter_req = uxr_buffer_create_datawriter_bin(&session, reliable_out, datawriter_id, publisher_id, topic_id, reliable_qos, UXR_REPLACE|UXR_REUSE);

    // Send create entities message and wait its status
    uint8_t status[4];
    uint16_t requests[4] = {participant_req, topic_req, publisher_req, datawriter_req};

    if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
    {
        printf("Error at create entities: participant: %i topic: %i publisher: %i darawriter: %i\n", status[0], status[1], status[2], status[3]);
        return 1;
    }
    return 0;
}

uint8_t msg_publish(const void * pmsg)
{
    ucdrBuffer mb;
    uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(pmsg, 0);
    uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &mb, topic_size);
    sensor_msgs_msg_Imu_serialize_topic(&mb, pmsg);

    // Reliable QoS
    if(uxr_run_session_until_confirm_delivery(&session, 100))
        return 0;
    else
        return 1;
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
    return create_publisher("rt/imu/data", "sensor_msgs::msg::dds_::Imu_");
}

void uxr_onTopicCallback(struct uxrSession* session, uxrObjectId object_id,
  uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub,
  uint16_t length, void* args)
{
  (void)(session); (void)(request_id); (void)(stream_id); (void)(length);
}