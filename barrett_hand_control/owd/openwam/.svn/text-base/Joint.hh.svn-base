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

#include <iostream>
#include <iomanip>
#include <pthread.h>
#include "globals.h"

#ifndef __JOINT_HH__
#define __JOINT_HH__

using namespace std;

class Joint{
private:
  void lock(){pthread_mutex_lock(&mutex);}
  void unlock(){pthread_mutex_unlock(&mutex);}


public:
  pthread_mutex_t mutex;

  int    ID;
  double q;       // position
  double t;       // Nm or puck torque units
  double offset;  // externally applied offset, in radians.  this will
                  //    change the value stored in q as computed by
                  //    WAM::mpos2jpos().  Used for external calibration
                  //    of the joints while OWD is running
  
  static const int J1 = 1;
  static const int J2 = 2;
  static const int J3 = 3;
  static const int J4 = 4;
  static const int J5 = 5;
  static const int J6 = 6;
  static const int J7 = 7;

#ifdef WRIST
  static const int Jn = 7;
#else
  static const int Jn = 4;
#endif

    //  static const double MIN_POS[];
    //  static const double MAX_POS[];

  /// The absolute max torque that should ever be applied to each joint.
  /// Calculated from the max torque for each motor, which is based on each
  /// motor's cable breaking strength.
  static const double MAX_MECHANICAL_TORQ_EXPECTED[7];
  static double MAX_MECHANICAL_TORQ[7];

  /// The maximum that we will ever send to each joint.  We generally keep
  /// these well below the MAX_MECHANICAL limits, but push it a little
  /// for J5, J6, and J7 so that we still have some strength.
  static double MAX_SAFE_TORQ[7];

  /// The largest calculated joint torque that can be safely clipped to
  /// the MAX_SAFE_TORQ level.  If a joint torque exceeds this threshold
  /// then OWD will idle all the motor pucks and trigger a torque fault
  /// by the safety puck.
  /// This level should be greater than or equal to MAX_SAFE_TORQ, but
  /// not so high that controller bugs go undetected.
  static const double MAX_CLIPPABLE_TORQ[7];

    //  static const double VEL[];
    //  static const double ACC[];
  
  Joint(){
    pthread_mutex_init(&mutex, NULL);
    q = 0.0; t = 0.0; offset=0.0;
  }

  int id(){return ID;}
  double trq(){  double T;  lock();  T = t;   unlock();  return T;  }
  double pos(){  double Q;  lock();  Q = q;   unlock();  return Q;  }

  void trq(double T) {  
    lock();  
    t = T; // clip( T, Joint::MIN_TRQ[id()], Joint::MAX_TRQ[id()] ); 
    //t = 0.0;
    unlock();
  }
  void pos(double Q) {  
    lock();  q = Q;  unlock(); 
    //    if(Q <= Joint::MIN_POS[id()] || Joint::MAX_POS[id()] <= Q);
    //cerr << "Joint::pos: WARNING J" << id() << " has reached its limit.\n";
  }

};

#endif

