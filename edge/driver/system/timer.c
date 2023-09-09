#include <timer.h>
#include <string.h>

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerIrqHandler(struct uloop_timeout *timeout);
static void TimerGetTime(struct timeval *tv);

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists(TimerEvent_t *obj);

void TimerInit(TimerEvent_t *obj, void (*callback)(void *context))
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsStarted = false;
    obj->IsNext2Expire = false;
    obj->Callback = callback;
    obj->Context = NULL;
    obj->timer.cb = TimerIrqHandler;
    memset(&obj->timer, 0, sizeof(obj->timer));
}

void TimerSetContext(TimerEvent_t *obj, void* context)
{
    obj->Context = context;
}

void TimerStart(TimerEvent_t *obj)
{
    if (obj == NULL ) {
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsStarted = true;
    obj->IsNext2Expire = false;

	uloop_timeout_add(&obj->timer);
}

bool TimerIsStarted(TimerEvent_t *obj)
{
    return obj->IsStarted;
}

static void TimerIrqHandler(struct uloop_timeout *timeout)
{
    struct TimerEvent_s *obj =  container_of(timeout, struct TimerEvent_s, timer);
    if (obj)
        obj->Callback(obj->Context);
}

void TimerStop(TimerEvent_t *obj)
{
    uloop_timeout_cancel(&obj->timer);
}

void TimerReset(TimerEvent_t *obj)
{
    TimerStop(obj);
    TimerStart(obj);
}

void TimerSetValue(TimerEvent_t *obj, uint32_t msecs)
{
    struct timeval *time = &obj->timer.time;

    if (obj->timer.pending)
        uloop_timeout_cancel(&obj->timer);

    TimerGetTime(time);

    time->tv_sec += msecs / 1000;
    time->tv_usec += (msecs % 1000) * 1000;

    if (time->tv_usec > 1000000) {
        time->tv_sec++;
        time->tv_usec -= 1000000;
    }

    obj->Timestamp = msecs;
    obj->ReloadValue = msecs;
}

TimerTime_t TimerGetCurrentTime(void)
{
    struct timespec ts = {0};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

TimerTime_t TimerGetElapsedTime(TimerTime_t past)
{
    if (past == 0) {
        return 0;
    }

    return TimerGetCurrentTime() - past;
}

static void TimerGetTime(struct timeval *tv)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000;
}

TimerTime_t TimerTempCompensation(TimerTime_t period, float temperature)
{
    return RtcTempCompensation(period, temperature);
}
