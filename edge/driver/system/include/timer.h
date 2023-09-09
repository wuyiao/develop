/*!
 * compatible layer
 *
 * \author    xuzhou ( ei.link)
 */
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <libubox/utils.h>
#include <libubox/uloop.h>

/*!
 * \brief Timer object description
 */
typedef struct TimerEvent_s {
    uint32_t Timestamp;                  //! Current timer value
    uint32_t ReloadValue;                //! Timer delay value
    bool IsStarted;                      //! Is the timer currently running
    bool IsNext2Expire;                  //! Is the next timer to expire
    void (*Callback)(void* context); //! Timer IRQ callback function
    void *Context;                       //! User defined data object pointer to pass back
    struct uloop_timeout timer;
} TimerEvent_t;

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#define TIMERTIME_T_MAX                             ( ( uint32_t )~0 )
#endif

/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Structure containing the timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit(TimerEvent_t *obj, void (*callback)(void *context));

/*!
 * \brief Sets a user defined object pointer
 *
 * \param [IN] context User defined data object pointer to pass back
 *                     on IRQ handler callback
 */
void TimerSetContext(TimerEvent_t *obj, void* context);

/*!
 * \brief Starts and adds the timer object to the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStart(TimerEvent_t *obj);

/*!
 * \brief Checks if the provided timer is running
 *
 * \param [IN] obj Structure containing the timer object parameters
 *
 * \retval status  returns the timer activity status [true: Started,
 *                                                    false: Stopped]
 */
bool TimerIsStarted(TimerEvent_t *obj);

/*!
 * \brief Stops and removes the timer object from the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStop(TimerEvent_t *obj);

/*!
 * \brief Resets the timer object
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerReset(TimerEvent_t *obj);

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] obj   Structure containing the timer object parameters
 * \param [IN] value New timer timeout value
 */
void TimerSetValue(TimerEvent_t *obj, uint32_t value);

/*!
 * \brief Read the current time
 *
 * \retval time returns current time
 */
TimerTime_t TimerGetCurrentTime(void);

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \remark TimerGetElapsedTime will return 0 for argument 0.
 *
 * \param [IN] past         fix moment in Time
 * \retval time             returns elapsed time
 */
TimerTime_t TimerGetElapsedTime(TimerTime_t past);

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t TimerTempCompensation(TimerTime_t period, float temperature);

/*!
 * \brief Processes pending timer events
 */
void TimerProcess(void);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_H__
