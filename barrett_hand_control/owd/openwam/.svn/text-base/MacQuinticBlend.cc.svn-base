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

#include "MacQuinticBlend.hh"
#include <math.h>
#include <stdio.h>

#define VERBOSE 0

#define PI MacAccelElement::PI

MacQuinticBlend::MacQuinticBlend(MacQuinticSegment *seg1,
				 MacQuinticSegment *seg2,
				 double blend_rad,
				 OWD::JointPos max_joint_v,
				 OWD::JointPos max_joint_a,
				 double max_jerk) 
  : MacQuinticElement(seg1->end_pos,seg2->start_pos),
    start_direction(seg1->direction),
    end_direction(seg2->direction),
    velocity(-1.0),
    max_joint_vel(max_joint_v),
    max_joint_accel(max_joint_a),
    blend_radius(fabs(blend_rad))
{
  // save the minimum of the two accel limits
  max_path_acceleration = 
    seg1->max_path_acceleration > seg2->max_path_acceleration?
    seg2->max_path_acceleration : seg1->max_path_acceleration;

  // save the minimum of the two velocity limits
  max_path_velocity = seg1->max_path_velocity > seg2->max_path_velocity?
    seg2->max_path_velocity : seg1->max_path_velocity;

  OWD::JointPos peak_joint_accel = (end_direction - start_direction) * 
    (pow(max_path_velocity/2/blend_radius,2)*2.5*blend_radius);
  double peak_path_accel=peak_joint_accel.length();
  if (peak_path_accel > max_path_acceleration) {
    free(reason);
    reason=strdup("peak joint acceleration limit");
    max_path_velocity /= sqrt(peak_path_accel / max_path_acceleration);
    free(reason);
    reason=strdup("peak acceleration limit");
  }

  if (VERBOSE) {
    printf("max velocity set: seg1=%2.3f, seg2=%2.3f, min=%2.3f\n",
	   seg1->max_path_velocity,
	   seg2->max_path_velocity,
	   max_path_velocity);
  }

  /*    double accel_limited_s=sqrt(0.8*max_path_acceleration*blend_radius
				     / sqrt((1+cos_theta)/2.0));
  double accel_limited_velocity = max_path_velocity / (2.0 * blend_radius)
    * accel_limited_s;
  if (max_path_velocity>accel_limited_velocity) {
    //    printf("Blend velocity of %2.3f reduced by accel limits to %2.3f\n",
    //	   velocity, accel_limited_velocity);
    max_path_velocity=accel_limited_velocity * (2.0 * blend_radius)
      / accel_limited_s;
    free(reason);
    reason=strdup("path acceleration limit");
  }
  */
  double cos_theta=(start_direction*end_direction)/(-pow(blend_radius,2));
  OWD::JointPos dir_diff = end_direction - start_direction;
  double theta=acos(cos_theta);
  double jerk_limited_s=pow(2*max_jerk*pow(blend_radius,2)
				   /(15*cos(theta/2)),1/3);
  double jerk_limited_velocity=max_path_velocity / (2 *blend_radius)
    * jerk_limited_s;

  if (max_path_velocity > jerk_limited_velocity) {
    if (VERBOSE) {
      printf("WARNING: reduced max velocity from %2.3f to %2.3f due to jerk limits\n",
	     max_path_velocity, jerk_limited_velocity);
    }
    max_path_velocity = jerk_limited_velocity;
    free(reason);
    reason=strdup("path jerk limit");
  }

  /*  if (max_path_velocity>jerk_limited_velocity) {
    //    printf("Blend velocity of %2.3f reduced by jerk limit to %2.3f\n",
    //	   velocity, jerk_limited_velocity);
    max_path_velocity=jerk_limited_velocity * (2 * blend_radius)
      / jerk_limited_s;
    free(reason);
    reason=strdup("path jerk limit");
  }
  */
  if (VERBOSE) {
    printf("final max path vel=%2.3f (%s)\n",max_path_velocity,reason);
  }
}

void MacQuinticBlend::setStartVelocity(double v) {
  if (v<=0) {
    throw "Invalid velocity: must be positive";
  }

  velocity = (v > max_path_velocity) ?
    max_path_velocity : v;

  if (VERBOSE) {
    printf("velocity set: requested %2.3f, max %2.3f, result %2.3f\n",
	   v,max_path_velocity,velocity);
  }
  // first, determine the max path velocity that doesn't exceed any
  // of the individual joint velocities.  The joint velocities will hit
  // their max values at the beginning and end of the blend, so we only
  // have to run through the joints and check them at each end.
  double max_vel_ratio = 0.0;
  OWD::JointPos starting_joint_vel = start_direction * velocity;
  OWD::JointPos ending_joint_vel = end_direction * velocity;
  int limiting_joint = -1;
  for (unsigned int j=0; j<max_joint_vel.size(); ++j) {
    if (fabs(starting_joint_vel[j])/max_joint_vel[j] > max_vel_ratio) {
      max_vel_ratio = starting_joint_vel[j] / max_joint_vel[j];
      limiting_joint=j;
    }
    if (fabs(ending_joint_vel[j])/max_joint_vel[j] > max_vel_ratio) {
      max_vel_ratio = ending_joint_vel[j] / max_joint_vel[j];
      limiting_joint=j;
    }
  }
  if (max_vel_ratio > 1.0) {
    // we don't really care which joint exceeded its limit at which end;
    // all we need to do is to scale the overall path velocity by the max
    // ratio we found and all joints will be within their limits (with at
    // least one joint right at its upper limit).
    //    printf("Blend velocity limited by joint %d velocity limit\n",limiting_joint);
    //    printf("Requested velocity was %2.3f, new velocity is %2.3f\n",
    //	   velocity, velocity/max_vel_ratio);
    free(reason);
    reason=strdup("joint velocity limit [WARNING: safety case reduced v]");
    velocity /= max_vel_ratio;
  } // BTW, even if we could go faster, we won't since we don't want to
    // exceed the speed of the adjacent segments.

  // check acceleration limits
  // based on Equation E.28 of MacFarlane thesis, and scaled to real-world
  // value by multiplying by (v/(2*blend_radius))^2
  OWD::JointPos peak_joint_accel = (end_direction - start_direction) * 
    (pow(velocity/2/blend_radius,2)*2.5*blend_radius);

  double max_accel_ratio = 0.0;
  limiting_joint = -1;
  for (unsigned int j=0; j<max_joint_accel.size(); ++j) {
    if (fabs(peak_joint_accel[j])/max_joint_accel[j] > max_accel_ratio) {
      max_accel_ratio = fabs(peak_joint_accel[j])/max_joint_accel[j];
      limiting_joint=j;
    }
  }
  if (max_accel_ratio > 1.0) {
    // reduce the velocity in order to bring the peak acceleration under
    // the limit
    //    printf("Joint %d accel limit of %2.3f forced blend velocity reduction from %2.3f to %2.3f\n",limiting_joint,max_joint_accel[limiting_joint],
    //	   velocity, velocity/sqrt(max_accel_ratio));
    free(reason);
    reason=(char *)malloc(200*sizeof(char));
    sprintf(reason,"joint acceleration limit (reduced from %2.3f to %2.3f due to joint %d [WARNING: safety case reduced v]",
	    velocity,velocity/sqrt(max_accel_ratio),limiting_joint);
    velocity /= sqrt(max_accel_ratio); // accel is proportional to v^2
  }
  if (VERBOSE) {
    printf("velocity set: requested %2.3f, max %2.3f, result %2.3f\n",
	   v,max_path_velocity,velocity);
  }
}

void MacQuinticBlend::setEndVelocity(double v) {
  setStartVelocity(v);
  return;
}

void MacQuinticBlend::BuildProfile() {
  duration = 2 * blend_radius / velocity;
  b1=start_pos;
  m1=start_direction *2*blend_radius;
  m2=end_direction*2*blend_radius;
  b2=end_pos-m2;
  m2_minus_m1 = m2-m1;
}

double MacQuinticBlend::PathVelocity() const {
  return current_path_vel; // vel at last evaluate()
}

double MacQuinticBlend::PathAcceleration() const {
  return current_path_accel; // accel at last evaluate()
}


void MacQuinticBlend::evaluate(OWD::Trajectory::TrajControl &tc, double t) {
  if (velocity <0) {
    throw "Error: must set blend velocity before evaluating";
  }
  if (duration < 0) {
    throw "Error: must call BuildProfile() before evaluating";
  }
  if (t<start_time) {
    t = start_time;
  }
  if (t > start_time + duration) {
    t= start_time + duration;
  }
  double sigma = (t-start_time)/duration;
  double sigma2=pow(sigma,2);
  double sigma3=pow(sigma,3);
  double sigma4=pow(sigma,4);
  double sigma5=pow(sigma,5);
  double sigma6=pow(sigma,6);
  double alpha=6 * sigma5 - 15 * sigma4 + 10 * sigma3;
  double alphadot = 30 * sigma4 - 60 * sigma3 + 30 * sigma2;
  double alphadotdot = 120 * sigma3 - 180 * sigma2 + 60 * sigma;
  double beta = sigma6 - 3 * sigma5 + 3 * sigma4 - sigma3;
  double betadot = 6 * sigma5 - 15 * sigma4 + 12 * sigma3 - 3 * sigma2;
  double betadotdot = 30 * sigma4 - 60 * sigma3 + 36 * sigma2 - 6 * sigma;
  static const double k=7.5;

  OWD::JointPos x2_minus_x1 = b2 - b1 + m2_minus_m1*sigma;
  
  OWD::JointPos x = b1
    + m1*sigma 
    + x2_minus_x1*alpha
    - m2_minus_m1*(k*beta);

  OWD::JointPos xdot = (m1 + x2_minus_x1*alphadot
		   + m2_minus_m1*alpha
		   - m2_minus_m1*k*betadot)
    * velocity/(2*blend_radius);

  OWD::JointPos xdotdot = (x2_minus_x1*alphadotdot
		      + m2_minus_m1*2*alphadot
		      - m2_minus_m1*k*betadotdot)
    * pow(velocity/(2*blend_radius),2);

  // Set our return values
  tc.q=x;
  tc.qd=xdot;
  tc.qdd=xdotdot;

  current_path_vel = xdot.length();
  current_path_accel=xdotdot.length();

  return;
}

double MacQuinticBlend::calc_time(OWD::JointPos value) const {
  if (velocity <0) {
    throw "Error: must set blend velocity before calling calc_time";
  }
  return 0;
}

void MacQuinticBlend::dump() {
  printf("MacQuinticBlend [velocity determined by %s]:\n", reason);
  MacQuinticElement::dump();
  printf("  start_pos: ");  start_pos.dump();
  printf("  end_pos: "); end_pos.dump();

  //  printf("  pos at t=%2.3f: ",start_time);
  //  double *y = (double *) malloc (sizeof(double) * start_direction.size());
  //  evaluate(y,0,0,start_time);
  //  JointPos jpy(start_direction.size());
  //  memcpy(&jpy[0],y,sizeof(double)*start_direction.size());
  //  jpy.dump();
  //  free(y);

  printf("  velocity=%2.3f  blend_radius=%2.3f\n",velocity,blend_radius);
}

MacQuinticBlend::MacQuinticBlend(BinaryData &bd)
  // first let the base class extract itself
  : MacQuinticElement(bd) {

  // now get our own members
  start_direction=bd.GetDoubleVector();
  end_direction=bd.GetDoubleVector();
  velocity=bd.GetDouble();
  max_joint_vel=bd.GetDoubleVector();
  max_joint_accel=bd.GetDoubleVector();
  b1=bd.GetDoubleVector();
  b2=bd.GetDoubleVector();
  m1=bd.GetDoubleVector();
  m2=bd.GetDoubleVector();
  m2_minus_m1=bd.GetDoubleVector();
  blend_radius=bd.GetDouble();
  current_path_vel=bd.GetDouble();
  current_path_accel=bd.GetDouble();
}

OWD::JointPos strip_vector(OWD::JointPos j, int firstdof, int lastdof) {
  OWD::JointPos jp;
  jp.insert(jp.begin(), j.begin()+firstdof, j.begin()+lastdof+1);
  return jp;
}
 
BinaryData MacQuinticBlend::serialize(int firstdof, int lastdof) {
  if (lastdof == -1) {
    lastdof = start_direction.size()-1;
  }
  // first have the Element base class put in its info
  BinaryData bd(MacQuinticElement::serialize(firstdof, lastdof));

  // now add our additional members
  bd.PutDoubleVector(strip_vector(start_direction,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(end_direction,firstdof,lastdof));
  bd.PutDouble(velocity);
  bd.PutDoubleVector(strip_vector(max_joint_vel,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(max_joint_accel,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(b1,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(b2,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(m1,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(m2,firstdof,lastdof));
  bd.PutDoubleVector(strip_vector(m2_minus_m1,firstdof,lastdof));
  bd.PutDouble(blend_radius);
  bd.PutDouble(current_path_vel);
  bd.PutDouble(current_path_accel);

  return bd;
}

