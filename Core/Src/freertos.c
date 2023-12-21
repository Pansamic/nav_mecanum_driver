/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* C standard library */
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <cpu_utils.h>

#include "arm_math.h"

/* STM32 Peripherals */
#include "rtc.h"
#include "tim.h"

/* Board Peripheral drivers */
#include "motor.h"
#include "led.h"
#include "icm20602.h"
//#include "mecanum.h"
#include "microros.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_READY_SIGNAL (0x00000001)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* system control states */
uint16_t ReleaseTime = 0; // unit:ms
uint8_t u8TaskListBuff[512];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_PublishIMUHandle;
osThreadId Task_ExecuteSpinHandle;
osThreadId Task_PingAgentHandle;
osThreadId Task_PublishJointStatesHandle;
osTimerId Timer_MotorAdjustHandle;
osSemaphoreId MicroROSSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
///*****************************/
///*    port comm interface    */
///*****************************/
//bool cubemx_transport_open(struct uxrCustomTransport * transport);
//bool cubemx_transport_close(struct uxrCustomTransport * transport);
//size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
//size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
//
///*****************************/
///*     memory management     */
///*****************************/
//void * microros_allocate(size_t size, void * state);
//void   microros_deallocate(void * pointer, void * state);
//void * microros_reallocate(void * pointer, size_t size, void * state);
//void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
//
///* set wheel velocity subscriber callback function */
//void set_mecanum_cb(const void * msgin);
///* set time reference subscriber call back function */
//void set_time_ref_cb(const void * msgin);
///* parameter server callback function */
//bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context);
/* mecanum constructure wheel odometry calculation function */
//void Odometry_Update(nav_msgs__msg__Odometry *odom_msg,float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
/* auxiliary function of `Odometry_Update()` */
//void euler_to_quat(float roll, float pitch, float yaw, float* q);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void PublishIMU(void const * argument);
void ExecuteSpin(void const * argument);
void PingAgent(void const * argument);
void PublishJointStates(void const * argument);
void MotorAdjustcb(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of MicroROSSemaphore */
  osSemaphoreDef(MicroROSSemaphore);
  MicroROSSemaphoreHandle = osSemaphoreCreate(osSemaphore(MicroROSSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Timer_MotorAdjust */
  osTimerDef(Timer_MotorAdjust, MotorAdjustcb);
  Timer_MotorAdjustHandle = osTimerCreate(osTimer(Timer_MotorAdjust), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 800);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_PublishIMU */
  osThreadDef(Task_PublishIMU, PublishIMU, osPriorityAboveNormal, 0, 1024);
  Task_PublishIMUHandle = osThreadCreate(osThread(Task_PublishIMU), NULL);

  /* definition and creation of Task_ExecuteSpin */
  osThreadDef(Task_ExecuteSpin, ExecuteSpin, osPriorityNormal, 0, 2048);
  Task_ExecuteSpinHandle = osThreadCreate(osThread(Task_ExecuteSpin), NULL);

  /* definition and creation of Task_PingAgent */
  osThreadDef(Task_PingAgent, PingAgent, osPriorityHigh, 0, 1024);
  Task_PingAgentHandle = osThreadCreate(osThread(Task_PingAgent), NULL);

  /* definition and creation of Task_PublishJointStates */
  osThreadDef(Task_PublishJointStates, PublishJointStates, osPriorityBelowNormal, 0, 1024);
  Task_PublishJointStatesHandle = osThreadCreate(osThread(Task_PublishJointStates), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* Suspend FreeRTOS tasks to avoid running micro-ros before initialization.
   * Tasks are resumed after micro-ros initialization. */
  osThreadSuspend(Task_PublishIMUHandle);
  osThreadSuspend(Task_ExecuteSpinHandle);
  osThreadSuspend(Task_PingAgentHandle);
  osThreadSuspend(Task_PublishJointStatesHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    /***************************************/
    /*           DC motor init             */
    /***************************************/
    DCMotor_Init(&LeftFrontMotor,     // initialize left front motor
          DCMOTOR_DEFAULT_DIRECTION,  // set default rotation as reverse
          ENCODER_DEFAULT,            // set encoder counts to the negative of original value
          &htim1,                     // set PWM generator timer module
          TIM_CHANNEL_1,              // set PWM generator timer output channel
          M1IN1_GPIO_Port,            // set logic control-1 GPIO port
          M1IN1_Pin,                  // set logic control-1 GPIO pin
          M1IN2_GPIO_Port,            // set logic control-2 GPIO port
          M1IN2_Pin);                 // set logic control-2 GPIO pin
    PID_Init(&LeftFrontMotor.VelocityController, 3.0, 0.1, 0.0);
    PID_Init(&LeftFrontMotor.PositionController, 2.0, 0.0, 2.0);
    LeftFrontMotor.EncTimer = &htim3;
    DCMotor_Init(&LeftRearMotor,      // initialize left front motor
          DCMOTOR_REVERSE_DIRECTION,  // set default rotation as reverse
          ENCODER_DEFAULT,            // set encoder counts to the negative of original value
          &htim1,                     // set PWM generator timer module
          TIM_CHANNEL_2,              // set PWM generator timer output channel
          M2IN1_GPIO_Port,            // set logic control-1 GPIO port
          M2IN1_Pin,                  // set logic control-1 GPIO pin
          M2IN2_GPIO_Port,            // set logic control-2 GPIO port
          M2IN2_Pin);                 // set logic control-2 GPIO pin
    PID_Init(&LeftRearMotor.VelocityController, 3.0, 0.1, 0.0);
    PID_Init(&LeftRearMotor.PositionController, 2.0, 0.0, 2.0);
    LeftRearMotor.EncTimer = &htim4;
    DCMotor_Init(&RightFrontMotor,      // initialize left front motor
          DCMOTOR_DEFAULT_DIRECTION,  // set default rotation as reverse
          ENCODER_REVERSE,            // set encoder counts to the negative of original value
          &htim1,                     // set PWM generator timer module
          TIM_CHANNEL_3,              // set PWM generator timer output channel
          M3IN1_GPIO_Port,            // set logic control-1 GPIO port
          M3IN1_Pin,                  // set logic control-1 GPIO pin
          M3IN2_GPIO_Port,            // set logic control-2 GPIO port
          M3IN2_Pin);                 // set logic control-2 GPIO pin
    PID_Init(&RightFrontMotor.VelocityController, 3.0, 0.1, 0.0);
    PID_Init(&RightFrontMotor.PositionController, 2.0, 0.0, 2.0);
    RightFrontMotor.EncTimer = &htim5;
    DCMotor_Init(&RightRearMotor,       // initialize left front motor
          DCMOTOR_REVERSE_DIRECTION,  // set default rotation as reverse
          ENCODER_REVERSE,            // set encoder counts to the negative of original value
          &htim1,                     // set PWM generator timer module
          TIM_CHANNEL_4,              // set PWM generator timer output channel
          M4IN1_GPIO_Port,            // set logic control-1 GPIO port
          M4IN1_Pin,                  // set logic control-1 GPIO pin
          M4IN2_GPIO_Port,            // set logic control-2 GPIO port
          M4IN2_Pin);                 // set logic control-2 GPIO pin
    PID_Init(&RightRearMotor.VelocityController, 3.0, 0.1, 0.0);
    PID_Init(&RightRearMotor.PositionController, 2.0, 0.0, 2.0);
    RightRearMotor.EncTimer = &htim2;

    /***************************************/
    /*         IMU ICM20602 init           */
    /***************************************/
    ICM20602_Init();

    /* enable motor encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

    microros_init();

    osThreadResume(Task_ExecuteSpinHandle);
    osThreadResume(Task_PublishIMUHandle);

    osThreadResume(Task_PublishJointStatesHandle);
    // osThreadResume(Task_PingAgentHandle);
    osTimerStart(Timer_MotorAdjustHandle,ENCODER_UPDATE_INTERVAL);

    /* Infinite loop */
    for(;;)
    {
        /* idle task for FreeRTOS memory management */
        osDelay(1000);
        memset(u8TaskListBuff, 0, 400);
        vTaskGetRunTimeStats((char*)u8TaskListBuff);
		printf("[INFO] CPU Usage:%d%%\r\n",osGetCPUUsage());


    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PublishIMU */
/**
* @brief Function implementing the Task_PublishIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PublishIMU */
void PublishIMU(void const * argument)
{
  /* USER CODE BEGIN PublishIMU */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5;
    xLastWakeTime = xTaskGetTickCount();
    rcl_ret_t ret;
    /* Infinite loop */
    for(;;)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        osSemaphoreWait(MicroROSSemaphoreHandle,0XFFFFFFFF);
        ICM20602_UpdateMessage(&msg_imu);
        ret = rcl_publish(&pub_imu,&msg_imu,NULL);

        if(ret == RCL_RET_ERROR)
        {
            printf("[ERROR]pub imu failed.\r\n");
        }
        else if(ret == RCL_RET_INVALID_ARGUMENT)
        {
        	printf("[ERROR]pub imu failed: Invalid arguments.\r\n");
        }
        else if(ret == RCL_RET_PUBLISHER_INVALID)
        {
        	printf("[ERROR]pub imu failed: Invalid publisher.\r\n");
        }
        osSemaphoreRelease(MicroROSSemaphoreHandle);
    }
  /* USER CODE END PublishIMU */
}

/* USER CODE BEGIN Header_ExecuteSpin */
/**
* @brief Function implementing the Task_ExecuteSpin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ExecuteSpin */
void ExecuteSpin(void const * argument)
{
  /* USER CODE BEGIN ExecuteSpin */
    /* Infinite loop */
    for(;;)
    {
        osSemaphoreWait(MicroROSSemaphoreHandle,0XFFFFFFFF);
        // Spin executor to receive messages
        microros_spinsome();
        osSemaphoreRelease(MicroROSSemaphoreHandle);
        osDelay(10);
    }
  /* USER CODE END ExecuteSpin */
}

/* USER CODE BEGIN Header_PingAgent */
/**
* @brief Function implementing the Task_PingAgent thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PingAgent */
void PingAgent(void const * argument)
{
  /* USER CODE BEGIN PingAgent */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;
    xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for(;;)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        osSemaphoreWait(MicroROSSemaphoreHandle,0XFFFFFFFF);
        if(rmw_uros_ping_agent(100,1)!=RMW_RET_OK)
        {
            printf("[WARN] micro_ros_agent disconnected!\r\n");
            /* NVIC MCU soft reset */
        }
        osSemaphoreRelease(MicroROSSemaphoreHandle);
    }
  /* USER CODE END PingAgent */
}

/* USER CODE BEGIN Header_PublishJointStates */
/**
* @brief Function implementing the Task_PublishJointStates thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PublishJointStates */
void PublishJointStates(void const * argument)
{
  /* USER CODE BEGIN PublishJointStates */
	rcl_ret_t ret;
    rcutils_time_point_value_t current_time;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;
    xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		joint_states.position[0] = LeftFrontMotor.CurrentAngle;
		joint_states.position[1] = LeftRearMotor.CurrentAngle;
		joint_states.position[2] = RightFrontMotor.CurrentAngle;
		joint_states.position[3] = RightRearMotor.CurrentAngle;

		joint_states.velocity[0] = LeftFrontMotor.CurrentVelocity;
		joint_states.velocity[1] = LeftRearMotor.CurrentVelocity;
		joint_states.velocity[2] = RightFrontMotor.CurrentVelocity;
		joint_states.velocity[3] = RightRearMotor.CurrentVelocity;
        ret = rcutils_system_time_now(&current_time);
        if(ret != RCUTILS_RET_OK)
        {
        	printf("[ERROR]read systime failed!\r\n");
        }

		msg_joint_state.header.stamp.sec = RCUTILS_NS_TO_S(current_time);
        msg_joint_state.header.stamp.nanosec = (uint32_t)(current_time - msg_joint_state.header.stamp.sec*1000000000LL);
		osSemaphoreWait(MicroROSSemaphoreHandle,0XFFFFFFFF);
		ret = rcl_publish(&pub_joint_states, &msg_joint_state, NULL);
		osSemaphoreRelease(MicroROSSemaphoreHandle);
		if(ret == RCL_RET_ERROR)
		{
            printf("[ERROR]publish /joint_states failed:publish error.\r\n");
		}
		else if(ret == RCL_RET_INVALID_ARGUMENT)
		{
            printf("[ERROR]publish /joint_states failed:invalid argument.\r\n");
		}
		else if(ret == RCL_RET_PUBLISHER_INVALID)
		{
            printf("[ERROR]publish /joint_states failed:publisher invalid.\r\n");
		}
	}

  /* USER CODE END PublishJointStates */
}

/* MotorAdjustcb function */
void MotorAdjustcb(void const * argument)
{
  /* USER CODE BEGIN MotorAdjustcb */

    if(ReleaseTime == 0)
    {
//        Car.TargetXVelocity = 0;
//        Car.TargetYVelocity = 0;
//        Car.TargetAngularVelocity = 0;
    	DCMotor_SetVelocity(&LeftFrontMotor, 0);
    	DCMotor_SetVelocity(&LeftRearMotor, 0);
    	DCMotor_SetVelocity(&RightFrontMotor, 0);
    	DCMotor_SetVelocity(&RightRearMotor, 0);
    }
    else
    {
        ReleaseTime -= ENCODER_UPDATE_INTERVAL;
    }

//    Car_AdjustedVelocity(&Car);

	DCMotor_AdjustVelocity(&LeftFrontMotor );
	DCMotor_AdjustVelocity(&LeftRearMotor  );
	DCMotor_AdjustVelocity(&RightFrontMotor);
	DCMotor_AdjustVelocity(&RightRearMotor );
  /* USER CODE END MotorAdjustcb */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


//void euler_to_quat(float roll, float pitch, float yaw, float* q)
//{
//    float cy = arm_cos_f32(yaw * 0.5);
//    float sy = arm_sin_f32(yaw * 0.5);
//    float cp = arm_cos_f32(pitch * 0.5);
//    float sp = arm_sin_f32(pitch * 0.5);
//    float cr = arm_cos_f32(roll * 0.5);
//    float sr = arm_sin_f32(roll * 0.5);
//
//    q[0] = cy * cp * cr + sy * sp * sr;
//    q[1] = cy * cp * sr - sy * sp * cr;
//    q[2] = sy * cp * sr + cy * sp * cr;
//    q[3] = sy * cp * cr - cy * sp * sr;
//}
//void Odometry_Update(nav_msgs__msg__Odometry *odom_msg,float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
//{
//    static float heading_ = 0.0f;
//    static float x_pos_ = 0.0f;
//    static float y_pos_ = 0.0f;
//    float delta_heading = angular_vel_z * vel_dt; //radians
//    float cos_h = arm_cos_f32(heading_);
//    float sin_h = arm_sin_f32(heading_);
//    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
//    float delta_y = (linear_vel_x * sin_h - linear_vel_y * cos_h) * vel_dt; //m
//
//    //calculate current position of the robot
//    x_pos_ += delta_x;
//    y_pos_ += delta_y;
//    heading_ += delta_heading;
//
//    //calculate robot's heading in quaternion angle
//    //ROS has a function to calculate yaw in quaternion angle
//    float q[4];
//    euler_to_quat(0, 0, heading_, q);
//
//    //robot's position in x,y, and z
//    odom_msg->pose.pose.position.x = x_pos_;
//    odom_msg->pose.pose.position.y = y_pos_;
//    odom_msg->pose.pose.position.z = 0.0;
//
//    //robot's heading in quaternion
//    odom_msg->pose.pose.orientation.x = (double) q[1];
//    odom_msg->pose.pose.orientation.y = (double) q[2];
//    odom_msg->pose.pose.orientation.z = (double) q[3];
//    odom_msg->pose.pose.orientation.w = (double) q[0];
//
//    odom_msg->pose.covariance[0] = 0.001;
//    odom_msg->pose.covariance[7] = 0.001;
//    odom_msg->pose.covariance[35] = 0.001;
//
//    //linear speed from encoders
//    odom_msg->twist.twist.linear.x = linear_vel_x;
//    odom_msg->twist.twist.linear.y = linear_vel_y;
//    odom_msg->twist.twist.linear.z = 0.0;
//
//    //angular speed from encoders
//    odom_msg->twist.twist.angular.x = 0.0;
//    odom_msg->twist.twist.angular.y = 0.0;
//    odom_msg->twist.twist.angular.z = angular_vel_z;
//
//    odom_msg->twist.covariance[0] = 0.0001;
//    odom_msg->twist.covariance[7] = 0.0001;
//    odom_msg->twist.covariance[35] = 0.0001;
//}

/* USER CODE END Application */
