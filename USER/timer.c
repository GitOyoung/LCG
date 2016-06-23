#include "timer.h"

/*
* @function: TimerStart
* @note: start the timer
* @param: timer: the timer which should be started
* @return None
 */
unsigned TimerStart(Timer *timer)
{
	timer->Enabled = TimerEnabled;
	return timer->Counter;
}

/*
* @function: TimerStop
* @note: Stop the timer
* @param: the timer which should be stopped
 */
unsigned TimerStop(Timer *timer)
{
	timer->Enabled = TimerDisabled;
	return timer->Counter;
}

/*
* @function: TimerExpired
* @note: expire the timer
* @param: the timer which should be expired
 */
TimerBool TimerExpired(Timer *timer)
{
	if (timer->Enabled != TimerEnabled 
    || timer->Counter != 0) return TM_FALSE;  
  timer->Enabled = TimerDisabled;     
  return TM_TRUE;
}

/*
* @function: TimerGet
* @note: Get the counter of the timer
* @param: the timer(no changing)
 */
unsigned TimerGet(const Timer *timer)
{
	return timer->Counter;
}

/*
* @function: TimerSet
* @note: modify the counter of the timer
* @param: timer, the timer which should be modified
* @param: value, the new counter of the timer
 */
TimerStatus TimerSet(Timer *timer, unsigned value)
{
	timer->Counter = value;
	return timer->Enabled;
}

