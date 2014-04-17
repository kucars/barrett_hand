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

#include "ControlLoop.hh"

#ifdef OWD_RT
#include <native/timer.h>
#else // ! OWD_RT
#include <sys/time.h>
#include <time.h>
#endif // ! OWD_RT

#include <ros/ros.h>

namespace OWD {

ControlLoop::ControlLoop(int tasknum, void (*fnc)(void*), void* argv) : 
  task_number(tasknum), ctrl_fnc(fnc), ctrl_argv(argv) {
  
  pthread_mutex_init(&mutex, NULL);
  
  snprintf(taskname,20,"OWDTASK%02d",task_number);
  
#ifdef OWD_RT
  // Xenomai example uses TASK_MODE 0 instead of T_CPU(1)
  int retval = rt_task_create(&task, taskname, 0, 50, T_CPU(1));
  if(retval){
    ROS_FATAL("ControlLoop: failed to create RT task %s: %d", taskname, retval);
    throw OW_FAILURE;
  }
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("ControlLoop: mlockall failed");
    if ((retval=rt_task_delete(&task))) {
      ROS_ERROR("Problem deleting RT task %s: %d", taskname, retval);
      throw OW_FAILURE;
    }
  }
  
#ifdef NOT_NECESSARY // ( causes an error)
  retval = rt_task_set_mode(0, T_PRIMARY, NULL);
  if (retval) {
    ROS_FATAL("Unable to run task %s in Primary mode: return=%d",taskname,retval);
    if ((retval=rt_task_delete(&task))) {
      ROS_ERROR("Problem deleting RT task %s: %d",taskname,retval);
      throw OW_FAILURE;
    }
    throw OW_FAILURE;
  }
#endif   

  // if CONFIG_XENO_OPT_NATIVE_PERIOD was set to zero (default) when building
  // Xenomai, then the time is expressed in nanoseconds.
  RTIME rtperiod = (RTIME)(ControlLoop::PERIOD*1000000000.0); 
  
  if((retval=rt_task_set_periodic(&task, TM_NOW, rtperiod))) {
    ROS_FATAL("ControlLoop: rt_task_set_periodic failed for RT task %s: %d", taskname,retval);
    if ((retval = rt_task_delete(&task))) {
      ROS_ERROR("Problem deleting RT task %s: %d",taskname,retval);
      throw OW_FAILURE;
    }
  }

#endif // OWD_RT
}

#if ! defined(OWD_RT)
void *ControlLoop::start_thread(void *data) {
  ControlLoop *cl = (ControlLoop*)data;
  (*cl->ctrl_fnc)(cl->ctrl_argv);
  return NULL;
}
#endif // ! OWD_RT

int ControlLoop::start(){

  if (cls == CONTROLLOOP_RUN) {
    return OW_SUCCESS;
  }

  lock();
  cls = CONTROLLOOP_RUN;
  unlock();

#ifdef OWD_RT
  RT_TASK_INFO info;
  int retval = rt_task_inquire(&task, &info);
  if (retval) {
    lock();
    cls = CONTROLLOOP_STOP;
    unlock();
    ROS_ERROR("ControlLoop: Unable to get status of RT task %s: %d", taskname, retval);
    return OW_FAILURE;
  }
  if (info.status & T_STARTED) {
    if ((retval=rt_task_resume(&task))) {
      lock();
      cls = CONTROLLOOP_STOP;
      unlock();
      ROS_ERROR("ControlLoop: Unable to resume RT task %s: %d", taskname, retval);
      return OW_FAILURE;
    }
    ROS_DEBUG("Resumed RT task %s", taskname);
    return OW_SUCCESS;
  }

  retval = rt_task_start(&task,ctrl_fnc,ctrl_argv);
  if (retval) {
    lock();
    cls = CONTROLLOOP_STOP;
    unlock();
    ROS_FATAL("ControlLoop: could not start RT task %s: %d",taskname,retval);
    return OW_FAILURE;
  }
  ROS_DEBUG("Started RT task %s",taskname);

#else // ! OWD_RT

  if(pthread_create(&ctrlthread, NULL, &ControlLoop::start_thread, this)) {
    ROS_FATAL("ControlLoop::start: pthread_create failed.");
    lock();
    cls = CONTROLLOOP_STOP;
    unlock();
    return OW_FAILURE;
  }

#endif  // ! OWD_RT

  return OW_SUCCESS;
}

int ControlLoop::stop(){
  if (cls == CONTROLLOOP_STOP) {
    ROS_DEBUG("ControlLoop: RT task %s was already stopped", taskname);
    return OW_SUCCESS;
  }

  lock();
  cls = CONTROLLOOP_STOP;
  unlock();

#ifdef OWD_RT
  int retval = rt_task_suspend(&task);
  if(retval){
    lock();
    cls = CONTROLLOOP_RUN;
    unlock();
    ROS_ERROR("ControlLoop: failed to suspend RT task %s: %d",taskname,retval);
    return OW_FAILURE;
  }
#else // ! OWD_RT
  if(pthread_join(ctrlthread, NULL) != 0){
    ROS_FATAL("ControlLoop::stop: pthread_join failed.");
    return OW_FAILURE;
  }
#endif // ! OWD_RT

  return OW_SUCCESS;
}

int ControlLoop::state_rt(){
  CONTROLLOOP_STATE s;
  //  lock();
  s = cls;
  //  unlock();
  return s;
}

void ControlLoop::wait_rt(int32_t usecs) {
#ifdef OWD_RT
  rt_task_wait_period(NULL);
#else // ! OWD_RT
  usleep(usecs);
#endif // ! OWD_RT
}

RTIME ControlLoop::get_time_ns_rt() {
#ifdef OWD_RT
  return rt_timer_ticks2ns(rt_timer_read());
#else // ! OWD_RT
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec * 1e9 + t.tv_usec * 1e3;
#endif  // ! OWD_RT
}

ControlLoop::~ControlLoop() {
#ifdef OWD_RT
  int retval = rt_task_delete(&task);
  if (retval) {
    ROS_ERROR("ControlLoop: error deleting RT task %s: %d", taskname,retval);
  }
#else // ! OWD_RT
  stop();
#endif // ! OWD_RT
}

}; // namespace OWD
