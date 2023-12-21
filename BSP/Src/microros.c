/**
 * @file microros.c
 * @author pansamic (pansamic@foxmail.com)
 * @brief micro-ROS related functions
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/******************************************************/
/*                      INCLUDES                      */
/******************************************************/
#include <time.h>

#include <microros.h>

/* STM32 Peripherals */
#include "rtc.h"
#include "tim.h"

/* Board Peripheral drivers */
#include "motor.h"
#include "led.h"
#include "icm20602.h"
#include "mecanum.h"

/******************************************************/
/*                      DEFINES                       */
/******************************************************/


/******************************************************/
/*                       MACROS                       */
/******************************************************/
#define RCCHECK(fn) \
{\
    rcl_ret_t temp_rc = fn;\
    if (RCL_RET_OK != temp_rc) {\
        printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);\
        return 1;\
    }\
}

#define RCSOFTCHECK(fn) \
{\
    rcl_ret_t temp_rc = fn;\
    if(RCL_RET_OK != temp_rc) {\
        printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);\
    }\
}

/******************************************************/
/*                     VARIABLES                      */
/******************************************************/
/*****************************/
/* micro ros core components */
/*****************************/
rclc_support_t support={0};
rcl_allocator_t allocator={0};
rcl_allocator_t freeRTOS_allocator={0};
rclc_executor_t executor={0};
rclc_parameter_server_t param_server={0};

/* micro ros parameter server config */
const rclc_parameter_options_t param_server_options = 
{
    .notify_changed_over_dds = true,
    .max_params = 4,
    .allow_undeclared_parameters = true,
    .low_mem_mode = false 
};
const char * paramname_wheel_dis_x = "wheel_dis_x";
const char * paramname_wheel_dis_y = "wheel_dis_y";
const char * paramname_wheel_radius = "wheel_radius";
const char * paramname_pulse_per_round = "pulse_per_round";
double paramval_wheel_dis_x = 0.193;
double paramval_wheel_dis_y = 0.223;
double paramval_wheel_radius = 0.075;
int paramval_pulse_per_round = 16800;

/*****************************/
/*  micro ros communication  */
/*****************************/
rcl_node_t node={0};
rcl_subscription_t sub_joint_jog={0};
rcl_subscription_t sub_time_ref={0};
rcl_publisher_t pub_imu={0};
rcl_publisher_t pub_joint_states={0};

/*****************************/
/*     micro ros messages    */
/*****************************/
//nav_msgs__msg__Odometry msg_odometry={0};
sensor_msgs__msg__Imu msg_imu={0};
sensor_msgs__msg__TimeReference msg_time_ref={0};
control_msgs__msg__JointJog msg_joint_jog={0};
sensor_msgs__msg__JointState msg_joint_state={0};
joint_states_t joint_states={0};
joint_jog_t joint_jog={0};

/******************************************************/
/*                     DECLARATION                    */
/******************************************************/
/*****************************/
/*    port comm interface    */
/*****************************/
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

/*****************************/
/*     memory management     */
/*****************************/
void * microros_allocate(size_t size, void * state);
void   microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

uint8_t microros_init_kernel(void);
uint8_t microros_init_param_server(void);
uint8_t microros_init_imu_publisher(void);
uint8_t microros_init_joint_states_publisher(void);
uint8_t microros_init_timeref_subscriber(void);
uint8_t microros_init_joint_jog_subscriber(void);

/* set wheel velocity subscriber callback function */
void set_wheel_vel_cb(const void * msgin);
/* set time reference subscriber call back function */
void set_time_ref_cb(const void * msgin);
/* parameter server callback function */
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context);

extern uint16_t ReleaseTime;

/******************************************************/
/*                     DEFINITION                     */
/******************************************************/
void microros_init(void)
{
    uint8_t ret=0;
    ret |= microros_init_kernel();
    ret |= microros_init_param_server();
    ret |= microros_init_imu_publisher();
//    ret |= microros_init_odom_publisher();
//    ret |= microros_init_tf2_publisher();
    ret |= microros_init_joint_states_publisher();
    ret |= microros_init_timeref_subscriber();
    ret |= microros_init_joint_jog_subscriber();
//    ret |= microros_init_twist_subscriber();

    if(ret)
    {
        Set_RGBLED(RED_ON,GREEN_OFF,BLUE_OFF);
        while(1){}
    }
    else
    {
        Set_RGBLED(RED_OFF,GREEN_ON,BLUE_OFF);
    }
}
uint8_t microros_init_kernel(void)
{
    rcl_ret_t ret;

    /***************************************/
    /*           allocator init            */
    /***************************************/
    rmw_uros_set_custom_transport(
          true,
          NULL,
          cubemx_transport_open,
          cubemx_transport_close,
          cubemx_transport_write,
          cubemx_transport_read);
    // detect micro ros agent
    printf("[INFO] waiting for micro ros agent.\r\n");
    while(rmw_uros_ping_agent(100,1)!=RMW_RET_OK)
    {
    }
    /* Here you also give to the allocator the functions
    * that are going to be used in order to allocate memory etc.
    */
    freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
    {
        printf("[ERROR]set default allocators failed!\r\n");
    }

    allocator = rcl_get_default_allocator();
    /***************************************/
    /*            support init             */
    /***************************************/
    // create init_options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, allocator);
    ret = rcl_init_options_set_domain_id(&init_options, 3); // set ROS_DOMAIN_ID=3
    ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if(ret == RCL_RET_OK)
    {
        printf("[INFO] init support OK!\r\n");
    }
    else
    {
        printf("[ERROR]init support failed!\r\n");
        return 1;
    }
    /***************************************/
    /*              node init              */
    /***************************************/
    // create node
    ret = rclc_node_init_default(&node, "mecanum_microros_node", "", &support);
    if(ret == RCL_RET_OK)
    {
        printf("[INFO] init node OK!\r\n");
    }
    else
    {
        printf("[ERROR]init node failed!\r\n");
        return 1;
    }

    /***************************************/
    /*            executor init            */
    /***************************************/
    executor = rclc_executor_get_zero_initialized_executor();
    /* 2 handles, time_ref subscription and cmd_vel_mecanum subscription */
    /* `RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES` are handles that param server needs */
    ret = rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES+2, &allocator);
    if(ret == RCL_RET_OK)
        printf("[INFO] init executor OK\r\n");
    else
    {
        printf("[ERROR]init executor failed!\r\n");
        return 1;
    }

    ret = rmw_uros_sync_session(100);
    if(ret == RMW_RET_OK)
    {
        printf("[INFO] sync time OK!\r\n");
    }
    else
    {
        printf("[ERROR]sync time failed!\r\n");
    }
    return 0;
}
uint8_t microros_init_param_server(void)
{
    rcl_ret_t ret;
    // Initialize parameter server with default configuration
    ret = rclc_parameter_server_init_with_option(&param_server, &node, &param_server_options);
    if (ret == RCL_RET_OK)
    {
        printf("[INFO] init param server OK!\r\n");
        ret = rclc_add_parameter(&param_server, paramname_wheel_dis_x, RCLC_PARAMETER_DOUBLE);
        ret = rclc_parameter_set_double(&param_server, paramname_wheel_dis_x, paramval_wheel_dis_x);
        ret = rclc_add_parameter(&param_server, paramname_wheel_dis_y, RCLC_PARAMETER_DOUBLE);
        ret = rclc_parameter_set_double(&param_server, paramname_wheel_dis_y, paramval_wheel_dis_y);
        ret = rclc_add_parameter(&param_server, paramname_wheel_radius, RCLC_PARAMETER_DOUBLE);
        ret = rclc_parameter_set_double(&param_server, paramname_wheel_radius, paramval_wheel_radius);
        ret = rclc_add_parameter(&param_server, paramname_pulse_per_round, RCLC_PARAMETER_INT);
        ret = rclc_parameter_set_int(&param_server, paramname_pulse_per_round, paramval_pulse_per_round);
        // Add parameter server to the executor including defined callback
        ret = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
    }
    else
    {
        printf("[ERROR]init param server failed!\r\n");
        return 1;
    }
    return 0;
}
uint8_t microros_init_imu_publisher(void)
{
    rcl_ret_t ret;
    // create publisher
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    ret = rclc_publisher_init_default(&pub_imu,&node, type_support,"imu/data");
    if (ret == RCL_RET_OK)
        printf("[INFO] imu publisher init OK!\r\n");
    else
    {
        printf("[ERROR]imu publisher init failed!\r\n");
        return 1;
    }
    msg_imu.header.frame_id.data = "base_link";
    msg_imu.header.frame_id.size = 9;
    msg_imu.header.frame_id.capacity = 9;
    for(uint8_t i=0 ; i<9 ; i++)
        msg_imu.angular_velocity_covariance[i]=-1;
    for(uint8_t i=0 ; i<9 ; i++)
        msg_imu.linear_acceleration_covariance[i]=-1;
    for(uint8_t i=0 ; i<9 ; i++)
        msg_imu.orientation_covariance[i]=-1;
    return 0;
}
//uint8_t microros_init_odom_publisher(void)
//{
//    rcl_ret_t ret;
//    ret = rclc_publisher_init_default(&pub_odometry,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),"/odom/unfiltered");
//    if (ret == RCL_RET_OK)
//    {
//        printf("[INFO] odometry publisher init OK!\r\n");
//        static char odometry_frame_id[] = "odom";
//        static char odometry_child_frame_id[] = "base_link";
//        msg_odometry.header.frame_id.data = odometry_frame_id;
//        msg_odometry.header.frame_id.capacity = 5;
//        msg_odometry.header.frame_id.size = 5;
//        msg_odometry.child_frame_id.data = odometry_child_frame_id;
//        msg_odometry.child_frame_id.capacity = 10;
//        msg_odometry.child_frame_id.size = 10;
//    }
//    else
//    {
//        printf("[ERROR]odometry publisher init failed!\r\n");
//        return 1;
//    }
//    return 0;
//}
//uint8_t microros_init_tf2_publisher(void)
//{
//    rcl_ret_t ret;
//    ret = rclc_publisher_init_default(&pub_tf2,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),"tf");
//    if(ret == RCL_RET_OK)
//    {
//        printf("[INFO] TF publisher init OK!\r\n");
//        msg_tf2.transforms.data = &tf2_data;
//        msg_tf2.transforms.data->header.frame_id.data = "odom";
//        msg_tf2.transforms.data->header.frame_id.capacity = 5;
//        msg_tf2.transforms.data->header.frame_id.size = 4;
//        msg_tf2.transforms.data->header.stamp.sec = 0;
//        msg_tf2.transforms.data->header.stamp.nanosec = 0;
//        msg_tf2.transforms.data->child_frame_id.data = "base_link";
//        msg_tf2.transforms.data->child_frame_id.capacity = 10;
//        msg_tf2.transforms.data->child_frame_id.size = 9;
//        msg_tf2.transforms.data->transform.translation.x = 0;
//        msg_tf2.transforms.data->transform.translation.y = 0;
//        msg_tf2.transforms.data->transform.translation.z = 0;
//        msg_tf2.transforms.data->transform.rotation.w = 1;
//        msg_tf2.transforms.data->transform.rotation.x = 0;
//        msg_tf2.transforms.data->transform.rotation.y = 0;
//        msg_tf2.transforms.data->transform.rotation.z = 0;
//        msg_tf2.transforms.capacity = 1;
//        msg_tf2.transforms.size = 1;
//    }
//    else
//    {
//        printf("[ERROR]TF publisher init failed!\r\n");
//        return 1;
//    }
//    return 0;
//}
uint8_t microros_init_joint_states_publisher(void)
{
	rcl_ret_t ret = rclc_publisher_init_default(&pub_joint_states,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),"joint_states");
	if (ret == RCL_RET_OK)
	{
		printf("[INFO] joint_state publisher init OK!\r\n");

		/* initialize joint_state message, modify data container 'joint_states',
         * rather than the message variable. */
		static char joint1_name_data[] = "left_front_wheel_joint";
		static char joint2_name_data[] = "left_rear_wheel_joint";
		static char joint3_name_data[] = "right_front_wheel_joint";
		static char joint4_name_data[] = "right_rear_wheel_joint";
		joint_states.joint_names[0].data = joint1_name_data;
		joint_states.joint_names[0].capacity = 24;
		joint_states.joint_names[0].size = 24;
		joint_states.joint_names[1].data = joint2_name_data;
		joint_states.joint_names[1].capacity = 23;
		joint_states.joint_names[1].size = 23;
		joint_states.joint_names[2].data = joint3_name_data;
		joint_states.joint_names[2].capacity = 25;
		joint_states.joint_names[2].size = 25;
		joint_states.joint_names[3].data = joint4_name_data;
		joint_states.joint_names[3].capacity = 26;
		joint_states.joint_names[3].size = 26;

		msg_joint_state.name.data = joint_states.joint_names;
		msg_joint_state.name.capacity = 4;
		msg_joint_state.name.size = 4;
		msg_joint_state.position.data = joint_states.position;
		msg_joint_state.position.capacity = 4;
		msg_joint_state.position.size = 4;
		msg_joint_state.velocity.data = joint_states.velocity;
		msg_joint_state.velocity.capacity = 4;
		msg_joint_state.velocity.size = 4;
		msg_joint_state.effort.data = joint_states.effort;
		msg_joint_state.effort.capacity = 4;
		msg_joint_state.effort.size = 4;
	}
	else
	{
		printf("[ERROR]joint_state publisher init failed!\r\n");
		return 1;
	}
	return 0;
}
uint8_t microros_init_timeref_subscriber(void)
{
    rcl_ret_t ret;
    // create a mecanum wheel message
    sensor_msgs__msg__TimeReference__init(&msg_time_ref);
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,TimeReference);
    // Initialize a reliable subscriber
    ret = rclc_subscription_init_best_effort(&sub_time_ref, &node,type_support, "time_reference");

    if (ret == RCL_RET_OK)
        printf("[INFO] time_reference subscriber init OK!\r\n");
    else
    {
        printf("[ERROR]time_reference subscriber init failed!\r\n");
        return 1;
    }

    // Add subscription to the executor
    ret = rclc_executor_add_subscription(&executor, &sub_time_ref, &msg_time_ref,&set_time_ref_cb, ON_NEW_DATA);
    if (ret == RCL_RET_OK)
        printf("[INFO] executor add subscription of /time_reference OK!\r\n");
    else
    {
        printf("[ERROR]executor add subscription of /time_reference failed!\r\n");
        return 1;
    }
    return 0;
}
//uint8_t microros_init_twist_subscriber(void)
//{
//    rcl_ret_t ret;
//    // create a mecanum wheel message
//    geometry_msgs__msg__Twist__init(&msg_twist);
//
//    // Initialize a reliable subscriber
//    ret = rclc_subscription_init_best_effort(&sub_twist, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist), "cmd_vel");
//
//    if (ret == RCL_RET_OK)
//        printf("[INFO] twist msg subscriber init OK!\r\n");
//    else
//    {
//        printf("[ERROR]twist msg subscriber init failed!\r\n");
//        return 1;
//    }
//
//    // Add subscription to the executor
//    ret = rclc_executor_add_subscription(&executor, &sub_twist, &msg_twist,&set_wheel_vel_cb, ON_NEW_DATA);
//    if (ret == RCL_RET_OK)
//        printf("[INFO] executor add subscription of /cmd_vel_mecanum OK!\r\n");
//    else
//    {
//        printf("[ERROR]executor add subscription of /cmd_vel_mecanum failed!\r\n");
//    }
//    return 0;
//}
uint8_t microros_init_joint_jog_subscriber(void)
{
	rcl_ret_t ret;
	control_msgs__msg__JointJog__init(&msg_joint_jog);
	ret = rclc_subscription_init_best_effort(&sub_joint_jog, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs,msg,JointJog), "cmd_mecanum");
	if (ret == RCL_RET_OK)
	{
		printf("[INFO] joint jog msg subscriber init OK!\r\n");
		joint_jog.joint_names[0].data = joint_jog.names[0];
		joint_jog.joint_names[1].data = joint_jog.names[1];
		joint_jog.joint_names[2].data = joint_jog.names[2];
		joint_jog.joint_names[3].data = joint_jog.names[3];

		msg_joint_jog.joint_names.data = joint_jog.joint_names;
		msg_joint_jog.joint_names.size = 4;
		msg_joint_jog.joint_names.capacity = 4;
		msg_joint_jog.displacements.data=joint_jog.displacements;
		msg_joint_jog.displacements.size = 4;
		msg_joint_jog.displacements.capacity = 4;
		msg_joint_jog.velocities.data=joint_jog.velocities;
		msg_joint_jog.velocities.size = 4;
		msg_joint_jog.velocities.capacity = 4;
	}
	else
	{
		printf("[ERROR]join jog msg subscriber init failed!\r\n");
		return 1;
	}

	// Add subscription to the executor
	ret = rclc_executor_add_subscription(&executor, &sub_joint_jog, &msg_joint_jog,&set_wheel_vel_cb, ON_NEW_DATA);
	if (ret == RCL_RET_OK)
		printf("[INFO] executor add subscription of /cmd_mecanum OK!\r\n");
	else
	{
		printf("[ERROR]executor add subscription of /cmd_mecanum failed!\r\n");
	}
	return 0;
}

void microros_spinsome(void)
{
    rclc_executor_spin_some(&executor,100);
}

void set_wheel_vel_cb(const void * msgin)
{
	control_msgs__msg__JointJog * msg = (control_msgs__msg__JointJog*) msgin;
	ReleaseTime = 200;
	DCMotor_SetVelocity(&LeftFrontMotor, (float)msg->velocities.data[0]);
	DCMotor_SetVelocity(&LeftRearMotor, (float)msg->velocities.data[1]);
	DCMotor_SetVelocity(&RightFrontMotor, (float)msg->velocities.data[2]);
	DCMotor_SetVelocity(&RightRearMotor, (float)msg->velocities.data[3]);
	printf("[INFO] recv vel: 1:%f | 2:%f | 3:%f | 4:%f\r\n",
			LeftFrontMotor.TargetVelocity,
			LeftRearMotor.TargetVelocity,
			RightFrontMotor.TargetVelocity,
			RightRearMotor.TargetVelocity);
}

void set_time_ref_cb(const void * msgin)
{
    sensor_msgs__msg__TimeReference * msg = (sensor_msgs__msg__TimeReference*) msgin;
    if(msg == NULL)
    {
        printf("[ERROR]received NULL time reference msg!\r\n");
    }
    else
    {
        RTC_TimeTypeDef rtc_time = {0};
        RTC_DateTypeDef rtc_date = {0};
        time_t timestamp_s = (time_t)msg->time_ref.sec;
        struct tm *_tm = localtime(&timestamp_s);
        rtc_date.Year = _tm->tm_year-70;
        rtc_date.Month = _tm->tm_mon;
        rtc_date.Date = _tm->tm_mday;
        rtc_time.Hours = _tm->tm_hour;
        rtc_time.Minutes = _tm->tm_min;
        rtc_time.Seconds = _tm->tm_sec;
        rtc_time.SubSeconds = (uint32_t)(hrtc.Init.SynchPrediv-(((msg->time_ref.nanosec)/1000000000.0f)*(hrtc.Init.SynchPrediv+1.0f)));
        HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
        HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
        printf("[INFO] recv time. timestamp: (sec:%lu | nsec:%lu)\r\n",msg->time_ref.sec, msg->time_ref.nanosec);
//        rclc_executor_remove_subscription(&executor,&sub_time_ref);
    }
}


/**
 * @brief Micro-ROS parameter server callback function
 * 
 * @param old_param Parameter actual value, `NULL` for new parameter request.
 * @param new_param Parameter new value, `NULL` for parameter removal request.
 * @param context User context, configured on `rclc_executor_add_parameter_server_with_context`.
 * @return true 
 * @return false 
 */
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;

  if (old_param == NULL && new_param == NULL) {
    printf("Callback error, both parameters are NULL\n");
    return false;
  }

  if (old_param == NULL) {
    printf("Creating new parameter %s\n", new_param->name.data);
  } else if (new_param == NULL) {
    printf("Deleting parameter %s\n", old_param->name.data);
  } else {
    printf("Parameter %s modified.", old_param->name.data);
    switch (old_param->value.type) {
      case RCLC_PARAMETER_BOOL:
        printf(
          " Old value: %d, New value: %d (bool)\r\n", old_param->value.bool_value,
          new_param->value.bool_value);
        break;
      case RCLC_PARAMETER_INT:
        printf(
          " Old value: %lld, New value: %lld (int)\r\n", old_param->value.integer_value,
          new_param->value.integer_value);
        break;
      case RCLC_PARAMETER_DOUBLE:
        printf(
          " Old value: %f, New value: %f (double)\r\n", old_param->value.double_value,
          new_param->value.double_value);
        break;
      default:
        break;
    }
  }

  return true;
}
