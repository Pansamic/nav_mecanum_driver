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

#include "uxr_client.h"

/* STM32 Peripherals */
#include "rtc.h"
#include "tim.h"

/* Board Peripheral drivers */
#include "motor.h"
#include "led.h"
#include "icm20602.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UXR_CLIENT_REESTABLISH (0x00000001)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* system control states */
uint16_t ReleaseTime = 0; // unit:ms
// uint8_t u8TaskListBuff[32];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_PublishIMUHandle;
osThreadId Task_ExecuteSpinHandle;
osThreadId Task_PingAgentHandle;
osThreadId Task_PublishJointStatesHandle;
osTimerId Timer_MotorAdjustHandle;
osSemaphoreId UXRSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
  osSemaphoreDef(UXRSemaphore);
  UXRSemaphoreHandle = osSemaphoreCreate(osSemaphore(UXRSemaphore), 1);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_PublishIMU */
  osThreadDef(Task_PublishIMU, PublishIMU, osPriorityAboveNormal, 0, 512);
  Task_PublishIMUHandle = osThreadCreate(osThread(Task_PublishIMU), NULL);

  /* definition and creation of Task_ExecuteSpin */
  osThreadDef(Task_ExecuteSpin, ExecuteSpin, osPriorityBelowNormal, 0, 512);
  Task_ExecuteSpinHandle = osThreadCreate(osThread(Task_ExecuteSpin), NULL);

  /* definition and creation of Task_PingAgent */
  osThreadDef(Task_PingAgent, PingAgent, osPriorityHigh, 0, 512);
  Task_PingAgentHandle = osThreadCreate(osThread(Task_PingAgent), NULL);

  /* definition and creation of Task_PublishJointStates */
  osThreadDef(Task_PublishJointStates, PublishJointStates, osPriorityNormal, 0, 512);
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

    uxrce_client_init();
    
    create_publisher_imu();
    create_publisher_joint_state();
    create_subscriber_joint_jog();
    

    // osThreadResume(Task_ExecuteSpinHandle);
    osThreadResume(Task_PublishIMUHandle);
    osThreadResume(Task_PublishJointStatesHandle);
    // osThreadResume(Task_PingAgentHandle);
    osTimerStart(Timer_MotorAdjustHandle,ENCODER_UPDATE_INTERVAL);

    // uint32_t ulNotifiedValue = 0;
    /* Infinite loop */
    for(;;)
    {
        /* idle task for FreeRTOS memory management */
        osDelay(1000);
        // memset(u8TaskListBuff, 0, 400);
        // vTaskGetRunTimeStats((char*)u8TaskListBuff);
        printf("[INFO] CPU Usage:%d%%\r\n",osGetCPUUsage());
        // xTaskNotifyWait(0x00000000,0xFFFFFFFF,&ulNotifiedValue,1000);
        // if(ulNotifiedValue==UXR_CLIENT_REESTABLISH)
        // {
        //     printf("[INFO] Re-establishing Micro-XRCE-DDS-Client connection...\r\n");
        //     uxr_delete_session(&session);
        //     uxrce_client_init();
        //     create_publisher_imu();
        //     osThreadResume(Task_PublishIMUHandle);
        //     osThreadResume(Task_PingAgentHandle);
        //     osThreadResume(Task_ExecuteSpinHandle);
        //     osThreadResume(Task_PublishJointStatesHandle);
        //     osTimerStart(Timer_MotorAdjustHandle,ENCODER_UPDATE_INTERVAL);
        // }
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
    int64_t current_time_nano = 0;
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        ICM20602_UpdateRaw();
        current_time_nano = uxr_epoch_nanos(&session);
        msg_imu.header.stamp.sec = current_time_nano/1000000000;
        msg_imu.header.stamp.nanosec = current_time_nano%1000000000;
        msg_imu.linear_acceleration.x = ICM20602_dev.Ax;
        msg_imu.linear_acceleration.y = ICM20602_dev.Ay;
        msg_imu.linear_acceleration.z = ICM20602_dev.Az;
        msg_imu.angular_velocity.x = ICM20602_dev.Gx;
        msg_imu.angular_velocity.y = ICM20602_dev.Gy;
        msg_imu.angular_velocity.z = ICM20602_dev.Gz;
        osSemaphoreWait(UXRSemaphoreHandle,0XFFFFFFFF);
        if(publish_imu())
        {
            printf("[ERROR]IMU publish failed!\r\n");
        }
        osSemaphoreRelease(UXRSemaphoreHandle);
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
        // osSemaphoreWait(UXRSemaphoreHandle,0XFFFFFFFF);
        // Spin executor to receive messages
        uxr_run_session_time(&session, 5);
        // osSemaphoreRelease(UXRSemaphoreHandle);
        // osDelay(10);
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
    uint8_t disconnect_count = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;
    xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for(;;)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        osSemaphoreWait(UXRSemaphoreHandle,0XFFFFFFFF);
        if (uxr_ping_agent_session(&session, 10, 1))
        {
            // printf("[INFO]Agent connected.\r\n");
            disconnect_count = 0;
        }
        else
        {
            disconnect_count++;
            if(disconnect_count >= 3)
            {
                printf("[ERROR]Agent disconnected.\r\n");
                HAL_NVIC_SystemReset();
                // disconnect_count = 0;
                // vTaskSuspend(Task_PublishIMUHandle);
                // vTaskSuspend(Task_PingAgentHandle);
                // vTaskSuspend(Task_PublishJointStatesHandle);
                // vTaskSuspend(Task_ExecuteSpinHandle);
                // xTimerStop(Timer_MotorAdjustHandle,0);
                // xTaskNotify(defaultTaskHandle,UXR_CLIENT_REESTABLISH,eSetBits);
            }
        }
        osSemaphoreRelease(UXRSemaphoreHandle);
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
    int64_t current_time_nano = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;
    xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for(;;)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        msg_joint_state.position[0] = LeftFrontMotor.CurrentAngle;
        msg_joint_state.position[1] = LeftRearMotor.CurrentAngle;
        msg_joint_state.position[2] = RightFrontMotor.CurrentAngle;
        msg_joint_state.position[3] = RightRearMotor.CurrentAngle;

        msg_joint_state.velocity[0] = LeftFrontMotor.CurrentVelocity;
        msg_joint_state.velocity[1] = LeftRearMotor.CurrentVelocity;
        msg_joint_state.velocity[2] = RightFrontMotor.CurrentVelocity;
        msg_joint_state.velocity[3] = RightRearMotor.CurrentVelocity;

        osSemaphoreWait(UXRSemaphoreHandle,0XFFFFFFFF);
        current_time_nano = uxr_epoch_nanos(&session);
        msg_joint_state.header.stamp.sec = current_time_nano/1000000000;
        msg_joint_state.header.stamp.nanosec = current_time_nano%1000000000;
        if(publish_joint_state())
        {
            printf("[ERROR]Joint state publish failed!\r\n");
        }
        osSemaphoreRelease(UXRSemaphoreHandle);
    }

  /* USER CODE END PublishJointStates */
}

/* MotorAdjustcb function */
void MotorAdjustcb(void const * argument)
{
  /* USER CODE BEGIN MotorAdjustcb */

    if(ReleaseTime == 0)
    {
        DCMotor_SetVelocity(&LeftFrontMotor, 0);
        DCMotor_SetVelocity(&LeftRearMotor, 0);
        DCMotor_SetVelocity(&RightFrontMotor, 0);
        DCMotor_SetVelocity(&RightRearMotor, 0);
    }
    else
    {
        ReleaseTime -= ENCODER_UPDATE_INTERVAL;
    }

    DCMotor_AdjustVelocity(&LeftFrontMotor );
    DCMotor_AdjustVelocity(&LeftRearMotor  );
    DCMotor_AdjustVelocity(&RightFrontMotor);
    DCMotor_AdjustVelocity(&RightRearMotor );
  /* USER CODE END MotorAdjustcb */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
