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


/*
 * This is really just very basic interface that provides a state and a mutex
 * This class is used to derive SE3 controllers and joint controllers
 */
#include <pthread.h>

#ifndef __CONTROLLER_HH__
#define __CONTROLLER_HH__

class Controller{

private:
  pthread_mutex_t mutex;

protected:
  int s;

public:

  static const int STOP = 0;
  static const int RUN  = 1;

  Controller(){
    pthread_mutex_init(&mutex, NULL); 
    stop();
  }
  virtual ~Controller(){}
  void lock()  {pthread_mutex_lock  (&mutex);}
  void unlock(){pthread_mutex_unlock(&mutex);}
  int trylock(){ return pthread_mutex_trylock(&mutex); }

  bool run(){   lock();  s = Controller::RUN;   unlock(); return true; }
  void stop(){  lock();  s = Controller::STOP;  unlock();  }

  int state(){
    int S;
    lock();
    S = s;
    unlock();
    return S;
  }
};

#endif
