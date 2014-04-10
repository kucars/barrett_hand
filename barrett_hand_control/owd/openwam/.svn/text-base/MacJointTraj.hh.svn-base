/***********************************************************************

  Copyright 2007-2011 Carnegie Mellon University and Intel Corporation
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

#include <iostream>
#include <math.h>
#include "MacQuinticSegment.hh"
#include "TrajType.hh"
#include "Trajectory.hh"
#include "BinaryData.hh"
#include <vector>

#ifndef __MACJOINTTRAJ_HH__
#define __MACJOINTTRAJ_HH__

namespace OWD {

class MacJointTraj : public Trajectory {
public:
  int DOF;
private:
  JointPos max_joint_vel;
  JointPos max_joint_accel;
public:
  std::vector<MacQuinticElement *>::iterator current_piece;
  std::vector<MacQuinticElement *> macpieces;

private:
  int rescale_to_slowest(int slowest_joint,
			 double max_end_time,
			 double accel_time,
			 const std::vector<double> &max_joint_vel,
			 const std::vector<double> &max_joint_accel);

  inline int sgn(double x) { return (x>0.0f)?1:(x<0.0f)?-1:0; }

  bool check_for_bend(TrajPoint &segstart, TrajPoint &p1, TrajPoint &p2);

public:
  MacJointTraj(TrajType &vtraj, 
	       const JointPos &max_joint_vel, 
	       const JointPos &max_joint_accel,
	       double max_jerk,
	       bool bWaitForStart,
	       bool bCancelOnStall,
	       bool bCancelOnForceInput,
	       bool bCancelOnTactileInput);

  MacJointTraj(BinaryData &bd);
  virtual BinaryData serialize(int firstdof=0, int lastdof=-1);

  virtual ~MacJointTraj();
    
  virtual void log(char *prefix);
    
  virtual void run();
  virtual void evaluate_abs(Trajectory::TrajControl &tc, double t);
  virtual void get_path_values(double *path_vel, double *path_accel) const;
  virtual void get_limits(double *max_path_vel, double *max_path_accel) const;
  //  virtual void rebuild_from_current();
  virtual void dump();
  virtual void reset(double t);
};

}; // namespace OWD

#endif
