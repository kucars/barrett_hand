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

// Implementation of jerk-limited concatenation of quintics, based
// on the 2001 thesis of Sonja Macfarlane, University of British Columbia

#ifndef MACQUINTICSEGMENT_H
#define MACQUINTICSEGMENT_H

#define TRAJ_TOLERANCE 0.003f

#include "MacQuinticElement.hh"
#include "MacAccelPulse.hh"

class MacQuinticSegment:public MacQuinticElement {
  friend class MacQuinticBlend;

public:

  // direction is an n-dof unit vector pointing from start to end
  OWD::JointPos direction;

  double jmax;  // maximum path jerk

  // distance is the straightline jointspace distance from start to end
  double distance;

  // velocities are the tangential jointspace velocities
  double start_vel, end_vel;

  // keep track of which "if" condition we used to build the segment
  int condition;


  double current_path_vel, current_path_accel;
  std::vector<MacAccelElement *> accel_elements;

  void enforceSpeedLimits();
  void reverse_accel_pulses();
  void verify_end_conditions();
  bool search_for_decel_pulse(double dist,
			      MacAccelElement *ae1,
			      MacAccelElement *ae2);
  MacAccelElement *check_for_cruise(double distance,
				    MacAccelElement *ae1,
				    MacAccelElement *ae2);

  MacQuinticSegment( OWD::TrajPoint first_p,
                     OWD::TrajPoint second_p,
		     OWD::JointPos max_joint_vel,
		     OWD::JointPos max_joint_accel,
		     double max_jerk);

  MacQuinticSegment(BinaryData &bd);

  // functions required by the base class
  void setStartVelocity(double v);
  void setEndVelocity(double v);
  void BuildProfile();
  
  double StartVelocity() const;
  double EndVelocity() const;
  
  void evaluate(OWD::Trajectory::TrajControl &tc, double t);
  double calc_time(OWD::JointPos value) const;
  double PathVelocity() const;
  double PathAcceleration() const;

  // extra functions specific to a segment
  void setVelocity(double v1, double v2);  // both start and end at once
  OWD::JointPos Direction() const;

  static double accel_rise_dist(double v, double amax, double jmax);
  static double accel_fall_dist(double v, double amax, double jmax);
  static double AP_dist(double v, double amax, double jmax);
  static double AP_max_delta_v(double d, double v,
			       double vmax, double amax, double jmax);
  static double cubic_solve(double A, double B, double C);
  static double safe_pow(double val, double exp);
  static double safe_sqrt(double val);

  void dump();
  virtual BinaryData serialize(int firstdof=0, int lastdof=-1);
};


#endif // MACQUINTICSEGMENT_H

