/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <inttypes.h>        // uint64_t
#include <unistd.h>          // usleep
#include "globals.h"

#include <sys/mman.h>

#ifdef OWD_RT
#include <native/intr.h>
#include <native/task.h>
#endif  // OWD_RT

#ifndef __CONTROL_LOOP_H__
#define __CONTROL_LOOP_H__

enum CONTROLLOOP_STATE{CONTROLLOOP_STOP, CONTROLLOOP_RUN};

#ifdef OWD_RT
#include <native/types.h>
#include <native/task.h>
#else // ! OWD_RT
typedef unsigned long long RTIME; // usually defined in xenomai types.h
#endif // ! OWD_RT

namespace OWD {
  class ControlLoop {
  private:
#ifdef OWD_RT
    RT_TASK task;
#else // ! OWD_RT
    pthread_t ctrlthread;
    static void *start_thread(void *data);
#endif // ! OWD_RT
    
    pthread_mutex_t mutex;
    CONTROLLOOP_STATE cls;
    char taskname[20];
    
  public:
    static const double PERIOD = 0.002;
    int task_number; // must be unique on the machine
    void (*ctrl_fnc)(void*);
    void* ctrl_argv;
    
    void lock(){pthread_mutex_lock(&mutex);}
    void unlock(){pthread_mutex_unlock(&mutex);}
    
    ControlLoop(int tasknum, void (*fnc)(void*), void* argv);
    ~ControlLoop();
    
    int start();
    int stop();
    int state_rt();
    void wait_rt(int32_t usecs);
    static RTIME get_time_ns_rt();
    
  };
}; // namespace OWD
#endif // __CONTROL_LOOP_H__
