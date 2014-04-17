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

#include <iostream>
#include <math.h>
#include "Trajectory.hh"
#include "Kinematics.hh"
#include <vector>

#ifndef __CONSTRAINEDFORCETRAJECTORY_HH
#define __CONSTRAINEDFORCETRAJECTORY_HH

using namespace OWD;

class ConstrainedForceTrajectory : public Trajectory {
public:

  class EndCondition {
  public:
    typedef enum {
      ANGLE=1,
      DISTANCE=2
    } Type;
    Type type;
    double value;
    double stall_vel;
  };
    
  ConstrainedForceTrajectory(const JointPos &start,
			     const JointPos &starting_force_vector,
			     const EndCondition end_condition,
			     Link wam_links[],
			     double max_velocity,
			     std::string trajid);
  virtual ~ConstrainedForceTrajectory();
    
  //  void evaluate(double y[], double yd[], double ydd[], double dt);
  void evaluate(Trajectory::TrajControl &tc, double dt);
  void update_torques(double t[]);

private:
  int DOF;
  JointPos initial_force_vector, current_force_vector;
  EndCondition end_cond;
  double max_vel;
  double *old_y;
  Link *links;
  double distance_moved;
};

#endif  // __CONSTRAINEDFORCETRAJECTORY_HH
