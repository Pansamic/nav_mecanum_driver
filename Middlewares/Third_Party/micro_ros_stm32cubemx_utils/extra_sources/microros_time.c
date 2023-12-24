#include <unistd.h>
#include <time.h>
#include "cmsis_os.h"
#include <rtc.h>
#include <stdio.h>

#define MICROSECONDS_PER_SECOND    ( 1000000LL )                                   /**< Microseconds per second. */
#define NANOSECONDS_PER_SECOND     ( 1000000000LL )                                /**< Nanoseconds per second. */
#define NANOSECONDS_PER_TICK       ( NANOSECONDS_PER_SECOND / configTICK_RATE_HZ ) /**< Nanoseconds per FreeRTOS tick. */

void UTILS_NanosecondsToTimespec( int64_t llSource,
                                  struct timespec * const pxDestination )
{
    long lCarrySec = 0;

    /* Convert to timespec. */
    pxDestination->tv_sec = ( time_t ) ( llSource / NANOSECONDS_PER_SECOND );
    pxDestination->tv_nsec = ( long ) ( llSource % NANOSECONDS_PER_SECOND );

    /* Subtract from tv_sec if tv_nsec < 0. */
    if( pxDestination->tv_nsec < 0L )
    {
        /* Compute the number of seconds to carry. */
        lCarrySec = ( pxDestination->tv_nsec / ( long ) NANOSECONDS_PER_SECOND ) + 1L;

        pxDestination->tv_sec -= ( time_t ) ( lCarrySec );
        pxDestination->tv_nsec += lCarrySec * ( long ) NANOSECONDS_PER_SECOND;
    }
}

int clock_gettime( int clock_id,struct timespec * tp )
{
	(void)clock_id;
	RTC_TimeTypeDef rtc_time = {0};
	RTC_DateTypeDef rtc_date = {0};
	struct tm _tm;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
	_tm.tm_year = rtc_date.Year+70; //RTC_Year rang 0-99,but tm_year since 1900
	_tm.tm_mon = rtc_date.Month;   //RTC_Month rang 1-12,but tm_mon rang 0-11
	_tm.tm_mday = rtc_date.Date;     //RTC_Date rang 1-31 and tm_mday rang 1-31
	_tm.tm_hour = rtc_time.Hours;  //RTC_Hours rang 0-23 and tm_hour rang 0-23
	_tm.tm_min = rtc_time.Minutes;   //RTC_Minutes rang 0-59 and tm_min rang 0-59
	_tm.tm_sec = rtc_time.Seconds;
	tp->tv_sec = mktime(&_tm);
	tp->tv_nsec = (long)((float)(hrtc.Init.SynchPrediv - rtc_time.SubSeconds)/(hrtc.Init.SynchPrediv+1.0f)*1000000000.0f);
    return 0;
}

//int clock_gettime( int clock_id,struct timespec * tp )
//{
//    TimeOut_t xCurrentTime = { 0 };
//
//    /* Intermediate variable used to convert TimeOut_t to struct timespec.
//     * Also used to detect overflow issues. It must be unsigned because the
//     * behavior of signed integer overflow is undefined. */
//    uint64_t ullTickCount = 0ULL;
//
//    /* Silence warnings about unused parameters. */
//    ( void ) clock_id;
//
//    /* Get the current tick count and overflow count. vTaskSetTimeOutState()
//     * is used to get these values because they are both static in tasks.c. */
//    vTaskSetTimeOutState( &xCurrentTime );
//
//    /* Adjust the tick count for the number of times a TickType_t has overflowed.
//     * portMAX_DELAY should be the maximum value of a TickType_t. */
//    ullTickCount = ( uint64_t ) ( xCurrentTime.xOverflowCount ) << ( sizeof( TickType_t ) * 8 );
//
//    /* Add the current tick count. */
//    ullTickCount += xCurrentTime.xTimeOnEntering;
//
//    /* Convert ullTickCount to timespec. */
//    UTILS_NanosecondsToTimespec( ( int64_t ) ullTickCount * NANOSECONDS_PER_TICK, tp );
//
//    return 0;
//}
