#include <stdio.h>

#include <malloc.h>

#include <pthread.h>



#include "wiringPi.h"

#include "digitalPWM.h"



#define	MAX_PINS	64



// The PWM Frequency is derived from the "pulse time" below. Essentially,

//	the frequency is a function of the range and this pulse time.

//	The total period will be range * pulse time in µS, so a pulse time

//	of 100 and a range of 100 gives a period of 100 * 100 = 10,000 µS

//	which is a frequency of 100Hz.

//

//	It's possible to get a higher frequency by lowering the pulse time,

//	however CPU uage will skyrocket as wiringPi uses a hard-loop to time

//	periods under 100µS - this is because the Linux timer calls are just

//	not accurate at all, and have an overhead.

//

//	Another way to increase the frequency is to reduce the range - however

//	that reduces the overall output accuracy...



#define	PULSE_TIME	100 //?



static volatile int marks         [MAX_PINS] ;

static volatile int range         [MAX_PINS] ;

static volatile pthread_t threads [MAX_PINS] ;

static volatile int newPin = -1 ;





static void *digitalPwmThread (void *arg)

{

  int pin, mark, space ;

  struct sched_param param ;



  param.sched_priority = sched_get_priority_max (SCHED_RR) ;

  pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;



  pin = *((int *)arg) ;

  free (arg) ;



  pin    = newPin ;

  newPin = -1 ;



  piHiPri (90) ;



  for (;;)

  {

    mark  = marks [pin] ;

    space = range [pin] - mark ;



    if (mark != 0)

      digitalWrite (pin, HIGH) ;

    delayMicroseconds (mark) ;



    if (space != 0)

      digitalWrite (pin, LOW) ;

    delayMicroseconds (space) ;

  }



  return NULL ;

}





void digitalPwmWrite (int pin, int value)

{

  if (pin < MAX_PINS)

  {

    /**/ if (value < 0)

      value = 0 ;

    else if (value > range [pin])

      value = range [pin] ;



    marks [pin] = value ;

  }

}





int digitalPwmCreate (int pin, int initialValue, int pwmRange)

{

  int res ;

  pthread_t myThread ;

  int *passPin ;



  if (pin >= MAX_PINS)

    return -1 ;



  if (range [pin] != 0)	// Already running on this pin

    return -1 ;



  if (pwmRange <= 0)

    return -1 ;



  passPin = malloc (sizeof (*passPin)) ;

  if (passPin == NULL)

    return -1 ;



  digitalWrite (pin, LOW) ;

  pinMode      (pin, OUTPUT) ;



  marks [pin] = initialValue ;

  range [pin] = pwmRange ;



  *passPin = pin ;

  newPin   = pin ;

  res      = pthread_create (&myThread, NULL, digitalPwmThread, (void *)passPin) ;



  if (res != 0)

    return res ;

  

  while (newPin != -1)

    delay (1) ;



  threads [pin] = myThread ;



  return res ;

}





void digitalPwmStop (int pin)

{

  if (pin < MAX_PINS)

  {

    if (range [pin] != 0)

    {

      pthread_cancel (threads [pin]) ;

      pthread_join   (threads [pin], NULL) ;

      range [pin] = 0 ;

      digitalWrite (pin, LOW) ;

    }

  }

}

