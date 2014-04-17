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

#ifndef MACQUINTICBLEND_H
#define MACQUINTICBLEND_H

#include "MacQuinticElement.hh"
#include "MacQuinticSegment.hh"
class MacQuinticBlend : public MacQuinticElement {

protected:
  // directions are n-dof unit vectors
  OWD::JointPos start_direction, end_direction;

  // velocity is the starting and ending tangential jointspace velocity
  double velocity;

  // Dynamic limits stored by the constructor
  OWD::JointPos max_joint_vel, max_joint_accel;

  // Blend constants used by the evaluate function
  OWD::JointPos b1,b2,m1,m2,m2_minus_m1;

  double blend_radius;
  double current_path_vel, current_path_accel;

  /*
  static const double k=7.5;

  inline double alpha(double sigma) const {
    return 6*pow(sigma,5) - 15*pow(sigma,4) + 10*pow(sigma,3);
  }
  
  inline double alphadot(double sigma) const {
    return 30*pow(sigma,4) - 60*pow(sigma,3) + 30*pow(sigma,2);
  }
  
  inline double alphadotdot(double sigma) const {
    return 120*pow(sigma,3) -180*pow(sigma,2) + 60*sigma;
  }
  
  inline double beta(double sigma) const {
    return pow(sigma,6) - 3*pow(sigma,5) + 3*pow(sigma,4) - pow(sigma,3);
  }
  
  inline double betadot(double sigma) const {
    return 6*pow(sigma,5) - 15*pow(sigma,4) + 12*pow(sigma,3) - 3*pow(sigma,2);
  }
  
  inline double betadotdot(double sigma) const {
    return 30*pow(sigma,4) - 60*pow(sigma,3) + 36*pow(sigma,2) - 8*sigma;
  }
*/

public:
  MacQuinticBlend(MacQuinticSegment *seg1,
		  MacQuinticSegment *seg2,
		  double blend_rad,
		  OWD::JointPos max_joint_vel,
		  OWD::JointPos max_joint_accel,
		  double max_jerk);

  MacQuinticBlend(BinaryData &bd);

  // functions required by the base class
  // blends start and end with the same velocity, so these next two
  //   functions do the same thing
  void setStartVelocity(double v);  
  void setEndVelocity(double v);
  void BuildProfile();
  
  inline double StartVelocity() const {
    if (velocity < 0) {
      throw "Must set velocity first";
    }
    return velocity;
  }
  inline double EndVelocity() const {
    if (velocity < 0) {
      throw "Must set velocity first";
    }
    return velocity;
  }
  
  void evaluate(OWD::Trajectory::TrajControl &tc, double t);
  double calc_time(OWD::JointPos value) const;
  double PathVelocity() const;
  double PathAcceleration() const;

  void dump();

  virtual BinaryData serialize(int firstdof=0, int lastdof=-1);
};


#endif // MACQUINTICBLEND_H

