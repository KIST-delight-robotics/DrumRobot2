/**********************************************************************
 *
 *  timer.h
 *
 *  Header of timer.c
 *
 *  v 0.0.0 2013.4.9 by Golden Wang
 *
 *    create
 *
 **********************************************************************/

#ifndef _TIMER_H
#define _TIMER_H

#include <linux/version.h>

#if LINUX_VERSION_CODE == KERNEL_VERSION(2,2,6)
#include <time.h>
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,2,12)
#include <sys/time.h>
#else
#include <time.h>
#endif

typedef unsigned long long int __tod_t;

extern __tod_t _time_get_tod();
extern void _time_set_deadline(unsigned long usec,
							   struct timeval *deadline);
extern int _time_out(struct timeval *deadline);

// extern int _set_timeout(long usec, __sighandler_t h_sigalrm);

#endif							/* _TIMER_H */
