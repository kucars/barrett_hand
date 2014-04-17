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

#include <math.h>
#include <errno.h>
#include <stdio.h>
#include <algorithm>
#include <assert.h>
#include "MacQuinticSegment.hh"

#define VERBOSE 0

MacQuinticSegment::MacQuinticSegment( OWD::TrajPoint first_p,
				      OWD::TrajPoint second_p,
				      OWD::JointPos max_joint_vel,
				      OWD::JointPos max_joint_accel,
				      double max_jerk):
  MacQuinticElement(first_p,second_p), // sets start_pos, end_pos
  jmax(max_jerk) 
{

  // calculate the distance and unit direction vector
  direction = second_p - first_p;
  distance = direction.length();
  direction /= distance;

  // adjust the endpoints based on anticipated blends
  if (first_p.blend_radius > 0) {
    start_pos += direction * fabs(first_p.blend_radius);
    distance -= first_p.blend_radius;
  }
  if (second_p.blend_radius > 0) {
    end_pos -= direction * fabs(second_p.blend_radius);
    distance -= second_p.blend_radius;
  }
  if (distance <0) {
    throw "MacQuinticSegment: Segment is shorter than the distance needed for blend(s)";
  }

  // find the max vel and accel we can apply in the calculated direction
  // without violating any per-joint limits

  max_path_velocity = -1.0;
  max_path_acceleration = -1.0;

  // go through the joints one-by-one; if we haven't yet set our values
  // and this joint is moving, try using it to set our speed.  If we're
  // already exceeding this joint's limit, then use this joint to recalc.
  for (unsigned int i = 0; i<direction.size(); ++i) {
    // velocity checks
    if (max_path_velocity < 0) {
      if (direction[i] != 0) {
	max_path_velocity = max_joint_vel[i] / fabs(direction[i]);
      }
    } else if ((max_path_velocity * fabs(direction[i])) > max_joint_vel[i]) {
      // this joint is going to exceed its vel limit, so recalc
      max_path_velocity = max_joint_vel[i] / fabs(direction[i]);
    }

    // repeat for acceleration
    if (max_path_acceleration < 0) {
      if (direction[i] != 0) {
	max_path_acceleration = max_joint_accel[i] / fabs(direction[i]);
      }
    } else if ((max_path_acceleration * fabs(direction[i])) > max_joint_accel[i]) {
      // this joint is going to exceed its accel limit, so recalc
      max_path_acceleration = max_joint_accel[i] / fabs(direction[i]);
    }
  }
  if ((max_path_velocity <= 0) || (max_path_acceleration <= 0)) {
    throw "ERROR: could not calculate a valid max velocity or acceleration (perhaps direction vector was all zero?)";
  }
  if (VERBOSE) {
    printf(" max path velocity=%2.3f, max path acceleration=%2.3f\n",
	   max_path_velocity, max_path_acceleration);
  }

}

// accel_rise_dist: returns how much distance is traversed by a single
// acceleration rise from zero to amax, starting at velocity v and 
// subject to jerk limit jmax
double MacQuinticSegment::accel_rise_dist(double v, double amax, double jmax) {
  // distance required for a single accel rise
  // Equation 3.7 / 3.8 of MacFarlane thesis
  double atime = amax/jmax * PI/2.0;
  return(fabs(v) * atime + amax*atime*atime*(.25 - 1.0/PI/PI));
}

// accel_fall_dist: distance covered by a single accel fall from amax to 0,
// starting at velocity v
double MacQuinticSegment::accel_fall_dist(double v, double amax, double jmax) {
  double atime = amax/jmax * PI/2.0;
  return(fabs(v) * atime + amax*atime*atime*(.25 + 1.0/PI/PI));
}

double MacQuinticSegment::AP_dist(double v, double amax, double jmax) {
  // total distance is an acceleration rise starting at velocity v followed
  //  by an acceleration fall starting at v+amax*atime/2 (midpoint v).
  double atime = amax/jmax * PI/2.0;
  return (accel_rise_dist(v,amax,jmax) 
	  + accel_fall_dist(v + amax*atime/2.0, amax, jmax));
}

// AP_max_delta_v: given limited distance, what's the max velocity
// change we can achieve with an acceleration pulse?
// (pulse can be either positive or negative - delta v is the same)
double MacQuinticSegment::AP_max_delta_v(double d, double v, double vmax, double amax, double jmax) {
  d = fabs(d); v=fabs(v); amax=fabs(amax);
  double atime = amax/jmax * PI/2.0;

  // make sure we can reach max accel and still have room for 
  //   a sustained portion
  if (d <= AP_dist(v,amax,jmax)) {
    // there's no room for a sustained portion, so calculate the max pulse
    // we can do

    double A=0;
    double B=v*4.0*jmax/PI;
    double C=-d*4.0*pow(jmax/PI,2);
    
    try {
      amax = cubic_solve(A,B,C);
    } catch (char *err) {
      if (VERBOSE) {
	printf("Could not get a cubic solution for A=%2.4f, B=%2.4f, C=%2.4f\n  %s\n",A,B,C,err);
      }
      throw;
    }
    if (VERBOSE) {
      printf("Used 3rd-order root finder to find a positive accel of %2.3f\n",
	     amax);
      printf("Input was d=%2.4f v=%2.4f jmax=%2.4f\n",
	     d,v,jmax);
    }
    
    // now use the new (lower) amax to calculate the new delta v
    //    printf("New max delta_v=%2.3f\n",atime*amax);
    //free(reason);
    //reason=strdup("distance limit w/ lower amax");
    double max_dv = pow(amax,2)/jmax * PI/2.0;
    if (VERBOSE) {
      printf("Max delta_v for a segment %2.3f long starting at v=%2.3f is %2.4f\n",
	     d,v,max_dv);
    }
    return max_dv;
  }
  // we're rising to amax, holding as long as we can, then falling back to 0
  // first, subtract out the distance consumed by the first rise
  d -= accel_rise_dist(v,amax,jmax);
  //  printf("AP_max_delta_v: distance remaining = %2.3f\n",d);
  // next, calculate the time that the sustain portion can last
  // uses quadratic formula based on calculating remaining distance
  // d=d_sustain + d_fall  (a function of t_sustain^2 and t_sustain)
  double t_sustain = (-v -1.5*amax*atime
		+ sqrt(v*v + 3*v*amax*atime + 2.25*amax*amax*atime*atime
		       -2*amax*(atime*v+amax*atime*atime*(1.0/PI/PI+.75)-d)))
    /amax;  // updated 10/4/08
  if (t_sustain < 0) {
    throw "ERROR: negative sustain time in this segment";
  }
  
  // finally, compute the total velocity change
  // we're still just working with positive velocities (directions can be neg)
  double delta_v = amax*atime + t_sustain*amax;
  if ((v + delta_v) > vmax) {
    // should stay below max_path_velocity
    //    printf("AP_max_delta_v condition 3: limiting max delta_v of %2.3f to only %2.3f to stay below %2.3f\n",delta_v,max_path_velocity-v,max_path_velocity);
    //free(reason);
    //    reason=strdup("velocity limit");
    return vmax - v;
  } else {
    //    printf("AP_max_delta_v condition 4: max delta_v of %2.3f\n",delta_v);
    //    printf("t_sustain=%2.3f, amax=%2.3f\n",t_sustain,amax);
    //    free(reason);
    //    reason=strdup("distance limit with sustain");
    return delta_v;
  }
}

void MacQuinticSegment::setVelocity(double v1, double v2) {
  start_vel = (v1 == VEL_MAX) ? max_path_velocity : fabs(v1);
  end_vel = (v2 == VEL_MAX) ? max_path_velocity : fabs(v2);
  enforceSpeedLimits();
  return;
}

void MacQuinticSegment::setStartVelocity(double v) {
  start_vel = (v==VEL_MAX) ? max_path_velocity : fabs(v);
  enforceSpeedLimits();
  return;
}

void MacQuinticSegment::setEndVelocity(double v) {
  end_vel = (v==VEL_MAX) ? max_path_velocity : fabs(v);
  enforceSpeedLimits();
  return;
}

double MacQuinticSegment::StartVelocity() const {
  return start_vel;
}

double MacQuinticSegment::EndVelocity() const {
  return end_vel;
}

void MacQuinticSegment::enforceSpeedLimits() {
  if (start_vel > max_path_velocity) {
    start_vel = max_path_velocity;
  }
  if (end_vel > max_path_velocity) {
    end_vel = max_path_velocity;
  }
  
  double delta_v = end_vel - start_vel;
  double max_delta;
  if (delta_v >= 0) {
    max_delta = AP_max_delta_v(distance, start_vel, max_path_velocity, max_path_acceleration, jmax);
  } else {
    // if start_vel > end_vel then we'll later swap them, so compute based
    // on end_vel instead of start_vel
    max_delta = AP_max_delta_v(distance, end_vel, max_path_velocity, max_path_acceleration, jmax);
  }
  if (delta_v > max_delta) {
    // speedup is too great: reduce the ending velocity
    if (VERBOSE) {
      printf("Reducing the ending velocity from %2.3f to %2.3f due to accel pulse limits\n",
	     end_vel, start_vel+max_delta);
    }
    end_vel = start_vel + max_delta;
  } else if (delta_v < - (double) max_delta) {
    // slowdown is too great: reduce the starting velocity
    if (VERBOSE) {
      printf("Reducing the starting velocity from %2.3f to %2.3f due to decel pulse limits\n",
	     start_vel, end_vel+max_delta);
    }
    start_vel = end_vel + max_delta;
  }
  return;
}

void MacQuinticSegment::verify_end_conditions() {
  if (accel_elements.size() == 0) {
    throw "MacQuinticSegment: no accel elements defined";
  }
  MacAccelElement *ap = accel_elements.front();
  if (fabs(ap->start_pos())>MacAccelPulse::epsilon) {
    if (VERBOSE) {
      printf("Error: starting pulse didn't start at zero\n");
      dump();
    }
    throw "MacQuinticSegment: generated pulses don't have correct starting position";
  }
  if (fabs(ap->start_vel()-start_vel)>MacAccelPulse::epsilon) {
    if (VERBOSE) {
      printf("Error: pulse start vel=%2.3f, target start vel=%2.3f\n",ap->start_vel(),start_vel);
      dump();
    }
    throw "MacQuinticSegment: generated pulses don't have correct starting velocity";
  }
  ap = accel_elements.back();
  if (fabs(ap->end_pos()-distance)>MacAccelPulse::epsilon) {
    if (VERBOSE) {
      printf("Error: end pos=%2.3f, distance=%2.3f\n",ap->end_pos(), distance);
      dump();
    }
    throw "MacQuinticSegment: generated pulses don't have correct ending position";
  }
  if (fabs(ap->end_vel()-end_vel)>MacAccelPulse::epsilon) {
    if (VERBOSE) {
      printf("Error: pulse end vel=%2.3f, target end vel=%2.3f\n",ap->end_vel(),end_vel);
      dump();
    }
    throw "MacQuinticSegment: generated pulses don't have correct ending velocity";
  }
  return;
}

void MacQuinticSegment::BuildProfile() {
  bool reverse=false;
  if (end_vel < start_vel) {
    // for a slowdown, compute it as if it were a speedup, but
    // remember to reverse things at the end
    reverse = true;
    double tmp_vel = end_vel;
    end_vel = start_vel;
    start_vel = tmp_vel;
  }

  if (distance == 0) {
    throw "MacQuinticSegment::BuildProfile: distance was zero";
  }
  
  double cruise_start=0;
  double cruise_start_time=start_time;
  double cruise_start_vel = start_vel;
  duration=0;

  // ============================================================
  // Test #1: Max Velocity?
  //
  // check to see whether we have room to reach max velocity
  // during this segment
  // ============================================================
  MacAccelElement *ap1 = NULL;
  MacAccelElement *ap2 = NULL;
  double distance_remaining = distance;
  if (VERBOSE) {
    printf(">> Applying test #1 (attempt to reach max_v)\n");
    printf("  start_vel=%2.3f, end_vel=%2.3f\n",start_vel, end_vel);
    printf("  max_path_vel=%2.3f, max_path_accel=%2.3f\n",
	   max_path_velocity, max_path_acceleration);
  }
  if (start_vel < max_path_velocity) {
    if (VERBOSE) {
      printf(" building ap1; %2.4f distance remaining\n",distance_remaining);
    }
    ap1 = new MacAccelPulse(0,
			    start_vel,
			    (max_path_velocity-start_vel),
			    max_path_acceleration,
			    jmax,
			    start_time);
    distance_remaining -= ap1->distance();
    cruise_start = ap1->end_pos();
    cruise_start_vel = ap1->end_vel();
    cruise_start_time = ap1->end_time();
    if (VERBOSE) {
      printf(" ap1 built; %2.4f distance remaining\n",distance_remaining);
    }
  }
  if (end_vel < max_path_velocity) {
    if (VERBOSE) {
      printf(" building ap2\n");
    }
    ap2 = new MacAccelPulse(cruise_start,
			    max_path_velocity,
			    (end_vel - max_path_velocity),
			    -max_path_acceleration,
			    -jmax,
			    cruise_start_time);
    distance_remaining -=ap2->distance();
    if (VERBOSE) {
      printf(" ap2 built; %2.4f distance remaining\n",distance_remaining);
    }
  }
  if (fabs(distance_remaining) < MacAccelPulse::epsilon) {
    if (VERBOSE) {
      printf("<< Test 1 succeeded without a sustain; wrapping up\n");
    }
    // in the unlikely event that we matched the distance, we
    // can wrap things up with what we've already used for testing
    if (ap1) {
      duration += ap1->duration();
      accel_elements.push_back(ap1);
    }
    if (ap2) {
      duration += ap2->duration();
      accel_elements.push_back(ap2);
    }
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  }

  if (distance_remaining > 0) {
    // we have confirmed that we can accel up to max vel and back down,
    // and still have room left over for a velocity cruise.  finishing up
    // is just a matter of creating the cruise.
    if (ap1) {
      duration += ap1->duration();
      accel_elements.push_back(ap1);
    }
    double cruise_end = cruise_start + distance_remaining;
    if (VERBOSE) {
      printf("<< Test 1 succeeded: adding a cruise\n");
    }
    
    ap1 = new MacZeroAccel(cruise_start,
			   cruise_start_vel,
			   cruise_end,
			   cruise_start_time);
    duration += ap1->duration();
    accel_elements.push_back(ap1);
    if (ap2) {
      // modify ap2 to start at the cruise ending position and time
      if (VERBOSE) {
	printf("         modifying ap2\n");
      }
      ap2->reset(cruise_end,
		 cruise_start_vel,
		 max_path_velocity - end_vel,
		 -max_path_acceleration,
		 -jmax,
		 cruise_start_time + ap1->duration());
      duration += ap2->duration();
      accel_elements.push_back(ap2);
    }
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  }

  // ok, we've established that we can't reach max_velocity.  throw
  // away our test pulses.
  if (ap1) {
    delete ap1;
    ap1=NULL;
  }
  if (ap2) {
    delete ap2;
    ap2=NULL;
  }
  if (VERBOSE) {
    printf("<< Test 1 failed: cannot reach max_velocity in available space\n");
  }
			   
  // ============================================================
  // TEST #2: Pure Acceleration
  //
  // check to see whether accelerating from start_vel to end_vel
  // exactly uses up the distance available
  // ============================================================

  if (VERBOSE) {
    printf(">> Applying test #2 (pure accel from start to end)\n");
  }
  distance_remaining=distance;
  // first, construct the accel pulse that satisfies the velocity change
  if (end_vel != start_vel) {
    if (VERBOSE) {
      printf(" building ap1; distance remaining is %2.4f\n",distance_remaining);
    }
    ap1 = new MacAccelPulse(0,
			   start_vel,
			   (end_vel-start_vel),
			   max_path_acceleration,
			   jmax,
			   start_time);
    cruise_start=ap1->distance();
    cruise_start_time=ap1->end_time();
    distance_remaining -= ap1->distance();
    if (distance_remaining < -MacAccelPulse::epsilon) {
      // something's wrong; we don't even have the available distance
      // to reach end_vel
      if (VERBOSE) {
	printf("distance available is %2.8f, pulse requires %2.8f\n",
	       distance_remaining,ap1->distance());
	ap1->dump();
      }
      delete ap1;
      throw "Error: velocity change cannot be met with motion limits in available distance";
    }
    if (distance_remaining < MacAccelPulse::epsilon) {
      // we used up all the distance just making the velocity change,
      // so we're done.
      if (VERBOSE) {
	printf("<< Test 2 succeeded with a single pulse; wrapping up\n");
      }
      duration = ap1->duration();
      accel_elements.push_back(ap1);
      if (reverse) {
	reverse_accel_pulses();
      }
      condition=1;
      verify_end_conditions();
      return;
    }
    if (VERBOSE) {
      printf("<< Test 2 failed: %2.4f extra space left after pure accel\n",
	     distance_remaining);
    }
  }
  double test2_distance_remaining = distance_remaining;

  // ============================================================
  // Peak Velocity Tests
  //
  // we've established that there's extra distance available for
  // a velocity peak, and that peak is less than max_path_velocity.
  // now we'll go through some checks to solve for the peak velocity.
  //
  // since start_vel <= end_vel, we will end up with one of the
  // following cases (corresponding to the subsequent tests):
  //    #3: Both accel pulse and decel pulse reach max_accel
  //    #4: Decel pulse doesn't reach max_accel (accel may or may not).
  //
  // ============================================================
  // TEST #3: Max Deceleration
  //
  // see whether there's room to get in an Accel Pulse of amplitude
  // max_path_acceleration when decelerating from Vpeak to end_vel.
  // if so, we know that both accel and decel pulses will reach
  // max_path_acceleration, and it will just be a matter of computing
  // the appropriate sustain durations for each.
    
  if (VERBOSE) {
    printf(">> Applying test #3 (max decel from peak velocity to end)\n");
  }

  double vpeak = end_vel + 
    max_path_acceleration * max_path_acceleration / jmax
    * PI / 2;
  if (ap1) {
    delete ap1;
  }
  if (VERBOSE) {
    printf(" building ap1\n");
  }
  ap1 = new MacAccelPulse(0,
			  start_vel,
			  (vpeak - start_vel),
			  max_path_acceleration,
			  jmax,
			  start_time);
  if (VERBOSE) {
    printf(" building ap2\n");
  }
  ap2 = new MacAccelPulse(ap1->end_pos(),
			  vpeak,
			  end_vel - vpeak,
			  -max_path_acceleration,
			  -jmax,
			  ap1->end_time());
  distance_remaining = distance - ap1->distance() - ap2->distance();
  if (fabs(distance_remaining) < MacAccelPulse::epsilon) {
    // we got lucky; it fits!
    if (VERBOSE) {
      printf("<< Test 3 succeeded without any sustain; wrapping up\n");
    }
    accel_elements.push_back(ap1);
    accel_elements.push_back(ap2);
    duration = ap1->duration() + ap2->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  } else if (distance_remaining > 0) {
    // we can make it fit by adding some sustain portions to both
    // accel and decel pulses.  the integrated area added to each
    // pulse must be equal in order for the end_velocity to come
    // out right, which means that the two sustain durations will
    // be equal (since the amplitude is at max_accel).
    // also, each sustain portion will use exactly half of
    // distance_remaining.

    // compute velocity at end of ap1 sustain
    double ap1_sustain_v = ap1->end_vel() 
      - max_path_acceleration * max_path_acceleration / jmax * PI / 4.0;


    // compute length of sustain extension
    // solve for t using quadratic formula, based on
    //   d = 1/2 * a * t*t  + (v+a*atime) * t  (where d = distance_remaining/2)
    double atime = max_path_acceleration/jmax * PI/2.0;
    
    double extra_sustain_t =
      (sqrt(ap1_sustain_v * ap1_sustain_v
	    + 2 * ap1_sustain_v * max_path_acceleration * atime
	    + pow(max_path_acceleration*atime,2)
	    + max_path_acceleration*distance_remaining)
       - (ap1_sustain_v + max_path_acceleration*atime))
      / max_path_acceleration;
    if (VERBOSE) {
      printf("  Calculated sustain extension of %2.3fs to make up remaining dist %2.4f\n",extra_sustain_t,distance_remaining);
    }
    if (extra_sustain_t <0) {
      // something went wrong
      delete ap1;
      delete ap2;
      throw "Error: bad calculation when extending pulses";
    }
    if (VERBOSE) {
      printf("  Extending ap1 sustain\n");
    }
    ap1->extend_sustain_time_by(extra_sustain_t);
    if (VERBOSE) {
      printf("  Extending ap2 sustain\n");
    }
    ap2->reset(ap1->end_pos(),
	       ap1->end_vel(),
	       end_vel-ap1->end_vel(),
	       -max_path_acceleration,
	       -jmax,
	       ap1->end_time());
    // don't need to explicitly extend the sustain time because
    // the reset will already calculate the appropriate sustain in
    // order to reach the delta_v.

    if (fabs(ap1->distance() + ap2->distance()- distance) > MacAccelPulse::epsilon) {
      if (VERBOSE) {
	printf("problem after extending these pulses:\n");
	ap1->dump();
	ap2->dump();
      }
      delete ap1;
      delete ap2;
      throw "Error: overall distance doesn't add up after extending pulses";
    }
    if (VERBOSE) {
      printf("<< Test 3 succeeded; wrapping up\n");
    }
    accel_elements.push_back(ap1);
    accel_elements.push_back(ap2);
    duration = ap1->duration() + ap2->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  } else {
    if (VERBOSE) {
      printf("<< Test 3 failed: not enough distance for peak decel\n");
    }
  }
  delete ap1;
  delete ap2;

  // ============================================================
  // TEST #4: Scaled decel
  //
  // We know that the decel won't reach max acceleration, so we'll have
  // to match a scaled decel with either a scaled accel or a max accel
  // with a sustain.  In either case, it comes down to a 4th order equation
  // without a closed-form solution, so we'll run a quick search on the
  // size of the decel pulse that, combined with a matching accel pulse,
  // uses up all the available distance.

  if (VERBOSE) {
    printf(">> Applying test #4\n");
  }

  // start by assuming we can reach max acceleration at the beginning
  vpeak = start_vel + 
    max_path_acceleration * max_path_acceleration / jmax
    * PI / 2.0;
  if (vpeak < end_vel) {
    // this case comes up when max_path_acceleration is so small
    // that a pure accel pulse doesn't surpass end_vel.

    // how long will it take (approximately) to go a quarter of
    // the distance left over by test 2, if we travel at env_vel?
    double extra_accel_time = test2_distance_remaining/4.0/end_vel;

    // how much extra v will we get from the extra time?
    double extra_v = extra_accel_time * max_path_acceleration;

    // start with a vpeak equal to end_vel plus our extra v
    vpeak = end_vel + extra_v;
  }
  if (VERBOSE) {
    printf(" building ap1: start_v=%2.4f, end_v=%2.4f\n",
	   start_vel, vpeak);
  }
  ap1 = new MacAccelPulse(0,
			  start_vel,
			  (vpeak - start_vel),
			  max_path_acceleration,
			  jmax,
			  start_time);
  if (VERBOSE) {
    printf(" building ap2: start_v=%2.4f, end_v=%2.4f\n",
	   vpeak, end_vel);
  }
  ap2 = new MacAccelPulse(ap1->end_pos(),
			  vpeak,
			  end_vel - vpeak,
			  -max_path_acceleration,
			  -jmax,
			  ap1->end_time());
  distance_remaining = distance - ap1->distance() - ap2->distance();
  if (fabs(distance_remaining) < MacAccelPulse::epsilon) {
    // we're done
    if (VERBOSE) {
      printf("<< Test 4 succeeded without rescaling; wrapping up\n");
    }
    accel_elements.push_back(ap1);
    accel_elements.push_back(ap2);
    duration = ap1->duration() + ap2->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  } else {

    // search for bigger or smaller pulses that use the available distance
    if (search_for_decel_pulse(distance,ap1,ap2)) {
      if (VERBOSE) {
	printf("<< Test 4 succeeded after search for matching pulses\n");
      }
      duration = ap1->duration() + ap2->duration();
      accel_elements.push_back(ap1);
      ap1 = check_for_cruise(distance,ap1,ap2);
      if (ap1) {
	if (VERBOSE) {
	  printf(" Adding velocity cruise of duration %2.4f, distance %2.4f\n",
		 ap1->duration(), ap1->distance());
	}
	duration += ap1->duration();
	accel_elements.push_back(ap1);
      }
      accel_elements.push_back(ap2);
      if (reverse) {
	reverse_accel_pulses();
      }
      verify_end_conditions();
      return;
    }
  }
  throw "Unable to find matching pulses";
}

MacAccelElement *MacQuinticSegment::check_for_cruise(double distance,
					       MacAccelElement *ae1,
					       MacAccelElement *ae2) {
  // make up any remaining distance with a velocity cruise
						     
  double dist_remaining = distance - ae1->distance() - ae2->distance();
  if (dist_remaining > 0) {
    double cruise_vel = ae1->end_vel();
    double cruise_end_pos = ae1->end_pos() + dist_remaining;
    ae1 = new MacZeroAccel(ae1->end_pos(),
			   cruise_vel,
			   cruise_end_pos,
			   ae1->end_time());
    ae2->reset(cruise_end_pos,
	       cruise_vel,
	       ae2->end_vel() - cruise_vel,
	       ae2->accel(),
	       ae2->jerk(),
	       ae1->end_time());
    return ae1;
  } else {
    return NULL;
  }
}

bool MacQuinticSegment::search_for_decel_pulse(double distance,
					       MacAccelElement *ae1,
					       MacAccelElement *ae2) {
  MacAccelPulse *ap1 = dynamic_cast<MacAccelPulse *>(ae1);
  MacAccelPulse *ap2 = dynamic_cast<MacAccelPulse *>(ae2);
  if (!ap1 || !ap2) {
    throw "Error: MacAccelElements must be pointers to MacAccelPulses";
  }
  
  double dist_remaining = distance - ap1->distance() - ap2->distance();
  if (VERBOSE) {
    printf("Searching for matching pulses; total distance=%2.3f\n",distance);
  }
  bool accel_sustain;
  if (dist_remaining >0) {
    accel_sustain = true; // we will need to go even faster by adding a
                          // sustain to pulse 1 and increasing pulse 2
    if (VERBOSE) {
      printf(" Searching for matching tsustain and a2\n");
    }
  } else {
    accel_sustain = false; // we will need to go slower by downsizing
                           // pulses 1 and 2
    if (VERBOSE) {
      printf(" Searching for matching a1 and a2\n");
    }
  }
  double start_vel = ap1->start_vel();
  double end_vel = ap2->end_vel();
  const double PI2 = pow(PI,2);
  int iterations = 0;
  double amax = max_path_acceleration;
  double amax2 = pow(amax,2);
  // double amax3 = pow(amax,3);
  double jmax2 = pow(jmax,2);
  double P=amax/2.0;
  double Q=start_vel + 0.75*amax2*PI/jmax;
  // double R=start_vel * amax * PI/jmax + amax3 * PI2 * 0.25/jmax2;
  double S=end_vel * PI/jmax;
  double U=0.75*PI2/jmax2;
  double W=0.5*PI/jmax/amax;
  double X=(end_vel-start_vel)/amax - 0.5*amax*PI/jmax;
  double K=(end_vel-start_vel) * 2.0 * jmax / PI;
  double jump_factor=0.9; // how big a jump we take
  double decel = ap2->accel();
  while ((++iterations < 100) && 
	 (dist_remaining > 0.0001 || dist_remaining < 0)) {
    // each unit increase in the acceleration of the decel pulse
    // will increase the distance by (a^2)*(PI^2/2/jmax^2) + Vend*PI/jmax.
    // thus we can predict how much of a step we need.
    double dist_step;
    if (dist_remaining < 0) {
      // make sure we get into pos range
      dist_step = dist_remaining * 1.25 * jump_factor;
    } else {
      dist_step = dist_remaining * jump_factor; // starts at 90%
    }

    //    double a_increment = sqrt((fabs(dist_step) - ap2->end_vel()*PI/jmax)
    //			      * 2 * pow(jmax,2)/PI2);
    double d_dist;  // derivative of distance wrt decel
    double decel2 = pow(decel,2);
    double decel3 = pow(decel,3);
    double jmax2 = pow(jmax,2);
    if (accel_sustain) {
      d_dist = decel3 *4.0*pow(W,2)*P
	+ decel2 *3.0 * U
	+ decel *2.0*W*(2.0*X*P + Q)
	+S;
    } else {
      d_dist = decel * pow(decel2 + K,-0.5)*start_vel*PI/jmax
	+ decel * pow(decel2 + K,0.5) * 0.75 * (PI2/jmax2)
	+ S
	+ decel2 * 3.0 * U;
    }
    double a_increment = dist_step / d_dist;
    //    if (dist_step > 0) {
    //    } else {
    //    }
    decel -= a_increment;  // remember decel is negative
    if (decel >= 0.0) {
      // oops, we overshot.  reduce the jump factor and try again
      decel += a_increment;
      jump_factor /= 2.0;
      if (VERBOSE) {
	printf("WARNING: overshoot resulted in positive decel;\n"
	       "         retrying with jump factor %2.0f%%\n",
	       jump_factor);
      }
      continue;
    }
    double vpeak = end_vel + decel*decel / jmax * PI / 2.0;
    if (VERBOSE) {
      printf("  Step %3d: d1=%2.3f d2=%2.3f remain=%2.3f step=%1.4f a_inc=%1.3f\n"
             "            vpeak=%2.4f accel=%2.4f decel=%2.4f\n",
	     iterations,ap1->distance(),ap2->distance(),
	     dist_remaining,dist_step,a_increment,vpeak,ap1->accel(),decel);
    }
    ap1->reset(0,start_vel,vpeak - start_vel,max_path_acceleration,jmax,ap1->start_time());
    ap2->reset(ap1->end_pos(),vpeak,end_vel - vpeak,decel,-jmax,ap1->end_time());
    double new_dist_remaining = distance - ap1->distance() - ap2->distance();
    if (dist_remaining > 0) {
      if ((new_dist_remaining > dist_remaining) ||
	  (new_dist_remaining<0)) {
	// we overshot, so repeat with a smaller jump factor
	if (VERBOSE) {
	  printf("  WARNING: overshot target; reducing jump factor from %2.0f%% to %2.0f%%\n", jump_factor*100.0, jump_factor*50.0);
	}
	jump_factor /= 2.0;
	// don't update dist_remaining or decel; just try again
	decel += a_increment;   // put it back the way it was
	continue;
      }
    } else {
      if ((new_dist_remaining < dist_remaining) ||
	  (new_dist_remaining > fabs(dist_remaining))) {
	// we overshot, so repeat with a smaller jump factor
	if (VERBOSE) {
	  printf("Warning: overshot target; reducing jump factor from %2.0f%% to %2.0f%%\n", jump_factor*100.0, jump_factor*50.0);
	}
	jump_factor /= 2.0;
	// don't update dist_remaining or decel; just try again
	decel += a_increment;   // put it back the way it was
	continue;
      }
    }
    dist_remaining = new_dist_remaining;
    decel = ap2->accel();
    if (jump_factor < 0.9) {
      // slowly return to our regular jump factor
      jump_factor *= 1.5;
      if (jump_factor > 0.9) {
	jump_factor = 0.9;
      }
    }
  }
  if (iterations == 100) {
    throw "Unable to find matching pulses after 100 tries\n";
  }
  if (VERBOSE) {
    printf("done!\n");
    printf("ap1 dist=%2.4f, ap2 dist=%2.4f\n",
	   ap1->distance(), ap2->distance());
    ap1->dump();
    ap2->dump();
  }

  return true;
}

void MacQuinticSegment::reverse_accel_pulses() {
  if (VERBOSE) {
    printf("reversing pulses.  Before:\n");
    dump();
  }
  // swap the start and end vels back to what they were
  double tmp_vel = end_vel;
  end_vel = start_vel;
  start_vel = tmp_vel;
  
  // reverse the order of the accel elements
  std::vector<MacAccelElement *> temp;
  for (int i=accel_elements.size()-1; i>=0; --i) {
    temp.push_back(accel_elements[(unsigned int)i]);
  }
  accel_elements.clear();
  accel_elements=temp;

  // run through them and reassign the proper values
  double t(start_time);
  double pos(0);
  double vel(start_vel);

  for (unsigned int i=0; i<accel_elements.size(); ++i) {
    double accel = accel_elements[i]->accel();
    double jerk = accel_elements[i]->jerk();
    double delta_v = accel_elements[i]->delta_vel();
    accel_elements[i]->reset(pos,vel,delta_v,-accel,-jerk,t);

    // save ending values to use for start of next segment
    t=accel_elements[i]->end_time();
    pos=accel_elements[i]->end_pos();
    vel=accel_elements[i]->end_vel();
  }
  if (VERBOSE) {
    printf("\nAfter reversing:\n");
    dump();
  }

  verify_end_conditions();
  return;
}

double MacQuinticSegment::PathVelocity() const {
  return current_path_vel;
}

double MacQuinticSegment::PathAcceleration() const {
  return current_path_accel;
}

void MacQuinticSegment::evaluate(OWD::Trajectory::TrajControl &tc, double t) {
  if (duration <0) {
    throw "MacQuinticSegment: BuildProfile() was never called";
  }
  if (t<start_time) {
    t = start_time;
  }
  if (t > start_time + duration) {
    t= start_time + duration;
  }
  unsigned int i=0;
  while (i<accel_elements.size() 
	 && (t>accel_elements[i]->end_time())) {
    ++i;
  }
  if (i == accel_elements.size()) {
    if (t > accel_elements.back()->end_time() + 0.001) {
      // if we're way beyond the end, throw an error
      if (VERBOSE) {
	printf("time of t=%2.8f exceeds last element ending time of %2.8f\n",
	       t, accel_elements.back()->end_time());
      }
      throw "MacQuinticSegment: accel elements don't match overall time";
    } else {
      // but if we're only a little beyond, ignore it
      i = accel_elements.size() -1;
      t=accel_elements[i]->end_time();  // bring t back within range
    }
  }
  // first, get the 1-D values (relative position, path vel, path accel),
  double path_pos;
  accel_elements[i]->eval(path_pos,current_path_vel,current_path_accel,t);
  // now, compute the n-D values
  OWD::JointPos pos, vel, accel;
  pos = start_pos + direction * path_pos;
  vel = direction * current_path_vel;
  accel = direction * current_path_accel;

  // copy the values into the output
  tc.q=pos;
  tc.qd=vel;
  tc.qdd=accel;
  return;
}

double MacQuinticSegment::calc_time(OWD::JointPos value) const {
  return 0;
}

void MacQuinticSegment::dump() {
  //printf("MacQuinticSegment [limited by %s, condition %d]:\n",reason,condition);
  printf("MacQuinticSegment\n");
  MacQuinticElement::dump();
  printf("  start_pos: ");  start_pos.dump();
  printf("  end_pos: "); end_pos.dump();

  if (accel_elements.size() == 0) {
    printf("ERROR: no accel elements defined\n");
    return;
  }
  printf("  eval at t=%2.3f: ",start_time+duration);
  OWD::Trajectory::TrajControl tc(direction.size());
  evaluate(tc,start_time+duration);
  OWD::JointPos jpy = tc.q;
  jpy.dump();

  printf("  start_vel=%2.3f  end_vel=%2.3f\n",start_vel,end_vel);
  printf("  distance=%2.3f\n",distance);
  printf("  accel elements:\n");
  for (unsigned int i=0; i<accel_elements.size(); ++i) {
    accel_elements[i]->dump();
  }
}

double MacQuinticSegment::safe_pow(double val, double exp) {
  errno=0;
  double result= pow(val,exp);
  if (errno == EDOM) {
    if (VERBOSE) {
      printf("WARNING: tried to call pow with value %2.4f, exponent %2.4f\n",
	     val,exp);
    }
    throw "pow generated EDOM";
  }
  return result;
}

double MacQuinticSegment::safe_sqrt(double val) {
  errno=0;
  double result= sqrt(val);
  if (errno == EDOM) {
    if (fabs(val) < 1.0e-6) {
      // only slightly negative; just return zero
      return 0;
    }
    if (VERBOSE) {
      printf("WARNING: tried to call sqrt with negative value %2.4f\n",
	     val);
    }
    throw "sqrt generated EDOM";
  }
  return result;
}


double MacQuinticSegment::cubic_solve(double A, double B, double C) {
  double Q=(safe_pow(A,2)-3.0*B)/9.0;
  double R=(2.0*safe_pow(A,3) - 9.0*A*B +27.0*C) / 54.0;
  if (VERBOSE) {
    printf("A=%2.4f B=%2.4f C=%2.4f Q=%2.4f R=%2.4f\n",A,B,C,Q,R);
    printf("pow(Q,3)=%2.4f\n",safe_pow(Q,3));
  }
  //  if (safe_pow(R,2) < safe_pow(Q,3) + 1.0e-6) {
  if (safe_pow(R,2) < safe_pow(Q,3)) {
    if (VERBOSE) {
      printf("Using case 1\n");
    }
    double theta=acos(R / safe_sqrt(safe_pow(Q,3)));
    double root1 = -2.0 * safe_sqrt(Q) * cos(theta/3.0) - A/3.0;
    double root2 = -2.0 * safe_sqrt(Q) * cos((theta+2.0*PI)/3.0) - A/3.0;
    double root3 = -2.0 * safe_sqrt(Q) * cos((theta-2.0*PI)/3.0) - A/3.0;
    if (root1 > 0) {
      return root1;
    } else if (root2 > 0) {
      return root2;
    } else if (root3 > 0) {
      return root3;
    } else {
      if (VERBOSE) {
	printf("found roots %2.4f, %2.4f, and %2.4f\n",root1,root2,root3);
      }
      throw "could not find a positive root";
    } 
  } else {
    if (VERBOSE) {
      printf("Using case 2\n");
    }
    double AA = safe_pow(fabs(R) + safe_sqrt(safe_pow(R,2) - safe_pow(Q,3)), 1.0/3.0);
    double BB;
    if (R>0.0) {
      AA = -AA;
    }
    if (AA != 0.0) {
      BB = Q/AA;
    } else {
      BB = 0;
    }
    double root1 = AA + BB - A/3.0;
    // root2 = -0.5 * (AA+BB) - A/3.0 + safe_sqrt(-3)/2.0*(AA-BB);
    // root2 = -0.5 * (AA+BB) - A/3 - safe_sqrt(-3)/2.0*(AA-BB);
    if (root1>0) {
      return root1;
    } else {
      if (VERBOSE) {
	printf("found real root %2.4f\n",root1);
      }
      throw "could not find a positive root";
    }
  } 
}

const double MacAccelElement::epsilon;

MacQuinticSegment::MacQuinticSegment(BinaryData &bd) 
  // first let the base class extract itself
  : MacQuinticElement(bd) {

  // now get our own members
  direction              =bd.GetDoubleVector();
  jmax                   =bd.GetDouble();
  distance               =bd.GetDouble();
  start_vel              =bd.GetDouble();
  end_vel                =bd.GetDouble();
  condition              =bd.GetInt();
  current_path_vel       =bd.GetDouble();
  current_path_accel     =bd.GetDouble();
  int accel_elements_size=bd.GetInt();
  for (int i=0; i<accel_elements_size; ++i) {
    int type=bd.GetInt();
    BinaryData bd2(bd.GetString());
    if (type == 0) {
      accel_elements.push_back(new MacAccelPulse(bd2));
    } else if (type == 1) {
      accel_elements.push_back(new MacZeroAccel(bd2));
    } else {
      throw "Unknown type while extracting MacAccelElements";
    }
  }
}

BinaryData MacQuinticSegment::serialize(int firstdof, int lastdof) {
  if (lastdof == -1) {
    lastdof = direction.size()-1;
  }

  // first have the Element base class put in its info
  BinaryData bd(MacQuinticElement::serialize(firstdof, lastdof));
  
  // now add our additional members
  OWD::JointPos d;
  d.insert(d.begin(),
	   direction.begin()+firstdof,
	   direction.begin()+lastdof+1);

  bd.PutDoubleVector(d);
  bd.PutDouble(      jmax);
  bd.PutDouble(      distance);
  bd.PutDouble(      start_vel);
  bd.PutDouble(      end_vel);
  bd.PutInt(         condition);
  bd.PutDouble(      current_path_vel);
  bd.PutDouble(      current_path_accel);
  bd.PutInt(         accel_elements.size());
  for (unsigned int i=0; i<accel_elements.size(); ++i) {
      if (dynamic_cast<MacAccelPulse *>(accel_elements[i])) {
	bd.PutInt(0);
      } else if (dynamic_cast<MacZeroAccel *>(accel_elements[i])) {
	bd.PutInt(1);
      } else {
	throw "Unknown element type in macpieces pointer list";
      }
    bd.PutString(accel_elements[i]->serialize());
  }
  return bd;
}

