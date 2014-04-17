/***********************************************************************

  Copyright 2009-2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#include "ServoTraj.hh"
#include "Plugin.hh"
#include <string.h>

namespace OWD {

  ServoTraj::ServoTraj(int dof, std::string id, double *start_pos,
		     double *lower_joint_limits,
		     double *upper_joint_limits)
  : Trajectory("ServoTraj", id),
    nDOF(dof),
    lasttime(time)
{
  stoptime.resize(nDOF,0.0);
  target_velocity.resize(nDOF,0.0);
  current_velocity.resize(nDOF,0.0);
  jlimit_buffer.resize(nDOF,0.0);
  start_position.SetFromArray(nDOF,start_pos);
  current_position = start_position;
  end_position.resize(nDOF);
  duration=99999999999; // we might want to run a long time

 // space to leave near joint limits to allow for deceleration 
  for (int i=0; i<nDOF; ++i) {
    // (1/2 * v^2 / a) is the distance to decelerate from v to 0
    jlimit_buffer[i] = 0.5
      * Plugin::joint_vel[i] * Plugin::joint_vel[i]
      / Plugin::joint_accel[i];
  }
}

ServoTraj::~ServoTraj() {
}
    
bool ServoTraj::SetVelocity(int j, float v, float duration) {
  if ((j > nDOF) || (j<1)) {
    return false;
  }
  if (v > Plugin::joint_vel[j-1]) {
    v = Plugin::joint_vel[j-1];
  } else if (v < -Plugin::joint_vel[j-1]) {
    v = -Plugin::joint_vel[j-1];
  }
  target_velocity[j-1] = v;
  stoptime[j-1] = time + duration;
  return true;
}
  
void ServoTraj::stop() {
  target_velocity.resize(nDOF,0.0);
  Trajectory::stop();
}

void ServoTraj::evaluate_abs(Trajectory::TrajControl &tc, double t) {
  if (tc.q.size() < (unsigned int)nDOF) {
    runstate=DONE;
    return;
  }
  time = t;
  double dt = time - lasttime;  // need delta time for accel
  lasttime=time;
  bool active = false;
  for (unsigned int i = 0; i<(unsigned int)nDOF; ++i) {
    if (time < stoptime[i]) {
      // check for approaching joint limits
      if ((target_velocity[i] > 0) &&
	  (current_position[i] + jlimit_buffer[i] > Plugin::upper_jlimit[i])) {
	double percent = (Plugin::upper_jlimit[i] - current_position[i])
	  / jlimit_buffer[i];
	double max_vel = Plugin::joint_vel[i] * percent;
	if (target_velocity[i] > max_vel) {
	  target_velocity[i] = max_vel;
	}
      } else if ((target_velocity[i] < 0) &&
		 (current_position[i] - jlimit_buffer[i] < Plugin::lower_jlimit[i])) {
	double percent = (Plugin::lower_jlimit[i] - current_position[i])
	  / jlimit_buffer[i]; // negative
	double min_vel = Plugin::joint_vel[i] * percent; // negative
	if (target_velocity[i] < min_vel) {
	  target_velocity[i] = min_vel;
	}
      }

      // check for accel/decel
      if (target_velocity[i] > current_velocity[i]) {
	// still accelerating to target
	current_velocity[i] += Plugin::joint_accel[i] * dt;
	tc.qdd[i]=Plugin::joint_accel[i];
	if (current_velocity[i] > target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      } else if (target_velocity[i] < current_velocity[i]) {
	// still decelerating
	current_velocity[i] -= Plugin::joint_accel[i] * dt;
	tc.qdd[i]=-Plugin::joint_accel[i];
	if (current_velocity[i] < target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      } else {
	tc.qdd[i]=0.0;
      }

      // update position and velocity
      current_position[i] += current_velocity[i] * dt;
      tc.qd[i] = current_velocity[i];
      active = true;
    } else {
      tc.qd[i]=0.0;
      tc.qdd[i]=0.0;
    }
    tc.q[i] = current_position[i];
  }
  if (!active) {
    end_position = current_position;
    runstate = DONE;
  }

}

};// namespace OWD
