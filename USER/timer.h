#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif
	
	typedef enum {
		TimerDisabled = 0,
		TimerEnabled
	} TimerStatus;

	typedef struct {
		TimerStatus Enabled;
		unsigned Counter;
	} Timer;
	
	typedef enum {
		TM_FALSE = 0,
		TM_TRUE = !TM_FALSE
	} TimerBool;
	
/*
* @function: TimerStart
* @note: start the timer
* @param: timer: the timer which should be started
* @return None
 */
unsigned TimerStart(Timer *timer);

/*
* @function: TimerStop
* @note: Stop the timer
* @param: the timer which should be stopped
 */
unsigned TimerStop(Timer *timer);

/*
* @function: TimerExpired
* @note: expire the timer
* @param: the timer which should be expired
 */
TimerBool TimerExpired(Timer *timer);

/*
* @function: TimerGet
* @note: Get the counter of the timer
* @param: the timer(no changing)
 */
unsigned TimerGet(const Timer *timer);
/*
* @function: TimerSet
* @note: modify the counter of the timer
* @param: timer, the timer which should be modified
* @param: value, the new counter of the timer
 */
TimerStatus TimerSet(Timer *timer, unsigned value);



#ifdef __cplusplus
}
#endif

#endif 
/*------------------end---------------------------*/
