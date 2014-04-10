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

#include "Trajectory.hh"
#include "Profile.hh"
#include "Joint.hh"
#include <math.h>

#ifndef __JOINTSTRAJ_HH__
#define __JOINTSTRAJ_HH__

/*
 * Implementa a joint trajectory. This is pretty easy as it just evaluates
 * whichever profile you have. I guess you could directly use a profile instead
 * but, I like consistent code and since you wouldn't be able to do the same
 * for Cartesian trajectory, then this class is the equivalent of SE3Trajectory
 */

class JointsTraj : public OWD::Trajectory{
protected:

  Profile *prof[Joint::Jn+1];
  double q1[Joint::Jn+1];     // initial positions
  double u[Joint::Jn+1];      // directions of the movement

public:

  JointsTraj( const double Q1[Joint::Jn+1], 
	      const double Q2[Joint::Jn+1], 
	      Profile* p[Joint::Jn+1] ) : OWD::Trajectory("JointsTraj",""){

    for(int j=Joint::J1; j<=Joint::Jn; j++){
      q1[j] = Q1[j];
      if(Q1[j] < Q2[j]) u[j] = +1;  // moving in the positive direction
      else              u[j] = -1;  // moving in the negative direction
      prof[j] = p[j];
      prof[j]->init( 0.0, fabs(Q2[j]-Q1[j]) );
    }
    stop();
  }

  // word of caution! the trajectory DELETE the profile
  // That implies that 1) you must allocate a profile and 2) don't delete it
  ~JointsTraj(){
    for(int j=Joint::J1; j<=Joint::Jn; j++)
      delete prof[j];
  }

  void run (){
    lock();
    for(int j=Joint::J1; j<=Joint::Jn; j++)
      prof[j]->run();
    unlock();
  }

  void stop(){
    lock();
    for(int j=Joint::J1; j<=Joint::Jn; j++)
      prof[j]->stop();
    unlock();
  }

  int state(){
    bool run = false;
    lock();
    for(int j=Joint::J1; j<=Joint::Jn; j++)
      if(prof[j]->state() == Profile::RUN){run = true;}
    unlock();
    if(run) return OWD::Trajectory::RUN;
    return OWD::Trajectory::STOP;
  }

  void evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
      if(state() == OWD::Trajectory::RUN){
          lock();
          for(int j=Joint::J1; j<=Joint::Jn; j++){
              prof[j]->evaluate(tc.q[j-1], tc.qd[j-1], tc.qdd[j-1], dt);
              tc.q[j-1] = u[j]*tc.q[j-1] + q1[j];
              tc.qd[j-1] = u[j]*tc.qd[j-1];
              tc.qdd[j-1] = u[j]*tc.qdd[j-1];
          }
          unlock();
      }
  }
};

#endif
