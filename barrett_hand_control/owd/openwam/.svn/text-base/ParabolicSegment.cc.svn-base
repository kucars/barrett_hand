/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
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

#include "ParabolicSegment.hh"
#include <math.h>
#include <ros/ros.h>

namespace OWD {
  ParabolicSegment::ParabolicSegment(int start_i,
				     double start_t,
				     double first_p,
				     double second_p):
    start_index(start_i), start_pos(first_p),
    end_pos(second_p),
    start_time(start_t), end_time(-1.0f),
    time_a(0.0f), time_v(0.0f) {
    if (second_p > first_p + TRAJ_TOLERANCE) {
      dir = UP;
    } else if (second_p < first_p - TRAJ_TOLERANCE) {
      dir = DOWN;
    } else {
      dir = CONST;
    }
  }

  void ParabolicSegment::dump() {
    ROS_DEBUG_NAMED("trajectory","spos=%1.2f epos=%1.2f st=%2.3f et=%2.3f ta=%2.3f tv=%2.5f a=%2.2f sidx=%d",
		    start_pos,end_pos,start_time,end_time,time_a,time_v,accel,start_index);
  }

  void ParabolicSegment::fit_curve(double mv, double a) {
    // at this point the start_pos and end_pos have been set.
    // we know we need v=0 at the beginning and end.
    // calculate the length of the accel, const vel, and decel segments
    // and update time_a1, time_v, time_a2, and end_time;
    
    max_vel = mv; 
    accel = a;
    if (accel < 0) {
      accel = -accel;
    }
    if (max_vel < 0) {
      max_vel = - max_vel;
    }
    if (dir == CONST) {
      // we started out thinking this would be a const curve, because it didn't change
      // much between successive points.  But it's possible that over the long term it
      // changed a lot, so re-check the delta between end points to find out what really
      // happened.
      if (end_pos > start_pos + TRAJ_TOLERANCE) {
	dir = UP;
      } else if (end_pos < start_pos - TRAJ_TOLERANCE) {
	dir = DOWN;
      } else {
	// dir is still CONST.  Not much to do to fit the curve.
	time_a = time_v = 0;
	end_time = start_time;
	return;
      }
    } else if (accel == 0.0f) {
      // if accel is zero, our times will be infinite.
      ROS_ERROR("ParabolicSegment: zero accel specified for non-const segment\n");
      throw "zero accel specified for non-const segment";
    }

    
    // Distance to travel
    double dq = fabs(end_pos-start_pos);
    // Max velocity of parabolic segment if no linear segment
    double v_end = sqrt(accel*dq);
    // Check if this is larger than allowed max vel
    if (v_end >= max_vel) {
      if (max_vel == 0.0f) {
	ROS_ERROR("ParabolicSegment: need a linear segment but max vel is zero\n");
	throw "need a linear segment but max vel is zero";
      }
      // Need to add a linear segment
      time_a = max_vel/accel;
      time_v  = (dq - accel*pow(time_a,2))/max_vel;
    } else {
      // No linear segment
      time_a = v_end/accel;
      time_v = 0.0f;
      max_vel = v_end;
    }
    
    end_time = start_time + 2.0*time_a + time_v;
    return;
  }
    
  void ParabolicSegment::refit_curve(double max_v, double max_a, double new_end_time, double new_accel_time) {
    // same as fit_curve, except instead of going ASAP, we're
    // going to end at exactly the supplied time.
    
    end_time = new_end_time;
    max_vel = max_v;
    time_a = new_accel_time;
    if (dir == CONST) {
      // easy; nothing else to do
      time_v=0;
      return;
    }
    
    if (time_a <= 0.0f) {
      ROS_ERROR("ParabolicSegment: cannot refit with accel time <= 0\n");
      throw "cannot refit with accel time <=0";
    }
    double total_time = end_time - start_time;
    if (time_a < total_time/2.0) {
      // we need to include a linear segment
      time_v = total_time - 2.0*time_a;
      accel = fabs(end_pos - start_pos) / 
	(pow(time_a,2) + time_a*time_v);
    } else {
      // no linear segment; just rescale the accel
      time_v = 0;
      accel = fabs(end_pos - start_pos) / 
	pow(time_a,2);
    }
    if (accel > max_a*1.10) {
      ROS_ERROR("ParabolicSegment: Accel of %3.3g exceeds limit of %3.3g\n",accel,max_a);
      throw "Accel exceeds limit";
    }
    if (accel * time_a > max_vel*1.10) {
      ROS_ERROR("ParabolicSegment: Velocity of %3.3g exceeds limit of %3.3g\n",accel*time_a,max_vel);
      throw "Velocity exceeds limit";
    }
    max_vel = accel * time_a;
    return;
  }

  void ParabolicSegment::evaluate(double &q, double &qd, double &qdd, double t) {
    // return the joint value at time t
    if ((t < start_time) || (t > end_time)) {
      ROS_ERROR("ParabolicSegment: requested time of %3.3f exceeds segment end time of %3.3f\n",t,end_time);
      throw "Requested time exceeds segment end time";
    }
    
    if (dir == CONST) {
      // easy
      q = start_pos;
      qd = 0.0;
      qdd = 0.0;
      return;
    }
    if (t < start_time + time_a) {
      // we're in the first parabolic segment
      q = start_pos + dir * 0.5 * accel * pow(t-start_time,2);
      qd = dir * accel * (t-start_time);
      qdd = dir * accel;
      return;
    } else if (t < start_time + time_a + time_v) {
      // we're in the linear segment
      q = start_pos + dir * (0.5 * accel * pow(time_a,2) // parabolic
			     + accel * time_a * (t-start_time-time_a)); // linear
      qd = dir * accel * time_a;
      qdd = 0.0;
      return;
    } else {
      // we must be in the second parabolic segment
      q = end_pos - dir * 0.5 * accel * pow(end_time - t,2);
      qd = dir * accel * (end_time - t);
      qdd = -dir * accel;
      return;
    }
  }

  double ParabolicSegment::calc_time(double value) {
    // take the joint angle, find which segment (accel, const, decel) it is in,
    // and the calculate the exact time at which we hit it.
    double dq_query = fabs(value - start_pos);
    double dq = fabs(end_pos - start_pos);
    double hit_time;
    
    if (end_time < 0.0f) {
      return -1.0f; // fit_curve was never called
    }
    
    // check to make sure that value is in between start and end
    
    // are we increasing?
    if ((end_pos - start_pos) > 0) {
      if ((value < start_pos) || (value > end_pos)) {
	return -1.0f; // out of range
      }
    } else { // must be decreasing
      if ((value > start_pos) || (value < end_pos)) {
	return -1.0f; // out of range
      }
    }

    if (dir == CONST) {
      if (fabs(value-start_pos)<TRAJ_TOLERANCE*2.0f) {
	return (start_time+end_time)/2.0f; // midpoint of const seg
      } else {
	return -1.0f; // value doesn't match this const seg
      }
    }
    
    if(time_v == 0.0f) {
      // No linear segment
      if(dq_query <= dq/2)
	// Acceleration segment
	hit_time = start_time + sqrt(2*dq_query/accel);
      else
	// Deceleration segment
	hit_time = end_time - sqrt(2*(dq - dq_query)/accel);
    } else {
      // Linear segment exists
      if(dq_query<= 0.5*accel*pow(time_a,2))
	// Acceleration segment
	hit_time = start_time + sqrt(2*dq_query/accel);
      else if(dq_query >= dq - pow(max_vel,2)/(2*accel))
	// Deceleration segment
	hit_time = end_time - sqrt(2*(dq - dq_query)/accel);
      else {
	// Linear segment
	double linear_dist = dq_query-0.5*accel*pow(time_a,2);
	hit_time = start_time + time_a + linear_dist/max_vel;
      }
    }        
    return hit_time;
  }

  bool ParabolicSegment::inflection(double current_pos, double next_pos) {
    if ((next_pos > current_pos + TRAJ_TOLERANCE) && (dir != UP)) {
      return true;
    }
    if ((next_pos < current_pos - TRAJ_TOLERANCE) && (dir != DOWN)) {
      return true;
    }
    if ((fabs(next_pos-current_pos)<TRAJ_TOLERANCE) && (dir != CONST)) {
      return true;
    }
    return false;
  }

}; // namespace OWD

