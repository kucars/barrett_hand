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

#include "ConstrainedForceTrajectory.hh"
#include "Joint.hh"
#include <stdlib.h>

extern "C" {
  void dgemv_(char *trans, int *m, int *n, double *alpha, double *a, int *lda,
	      double *x, int *incx, double *beta, double *y, int *incy);
}

ConstrainedForceTrajectory::ConstrainedForceTrajectory(
      const OWD::JointPos &startpos,
      const OWD::JointPos &starting_force_vector,
      const EndCondition end_condition,
      Link wam_links[],
      double max_velocity,
      std::string trajid) :
  OWD::Trajectory("ConstrainedForceTrajectory", trajid),
  initial_force_vector(starting_force_vector),
  current_force_vector(starting_force_vector),
  end_cond(end_condition),
  links(wam_links),
  distance_moved(0)
{
  // initialize the base-class members
  start_position=startpos;
  WaitForStart=false;
  CancelOnStall=false;
  
  DOF=start_position.size();
  old_y = (double *) malloc (DOF*sizeof(double));

}

void ConstrainedForceTrajectory::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  
  if (tc.q.size() < (unsigned int) DOF) {
    runstate = DONE;  // can't do anything
    return;
  }

  // compute the joint movement since last call
  static double *y_diff=(double *)malloc(DOF*sizeof(double));
  for (int i=0; i<DOF; ++i) {
    y_diff[i]=tc.q[i]-old_y[i];
    old_y[i]=tc.q[i]; // save the y for next time
  }

  // compute the workspace movement of the hand
  static double *ws_diff = (double *)malloc(6*sizeof(double));
  // calculate ws_diff = J*y_vel
  char TRANSN = 'N';  int NEQS = 6; int LDJ = 6; int INC = 1;
  double ALPHA =  1.0;  double BETA  =  0.0;
  dgemv_(&TRANSN, &NEQS,&DOF, &ALPHA,
	 &OWD::Kinematics::Jacobian0[0][0],        &LDJ,
	 y_diff,    &INC, &BETA,
	 ws_diff, &INC);
  OWD::JointPos WSdiff;
  WSdiff.SetFromArray(DOF,ws_diff);

  // calculate motion since last call
  distance_moved += sqrt(pow(ws_diff[0],2)
			 +pow(ws_diff[1],2)
			 +pow(ws_diff[2],2));
  double total_angle = acos((initial_force_vector*WSdiff)
			    / initial_force_vector.length()
			    / WSdiff.length());

  // check for ending condition
  if (((end_cond.type == EndCondition::ANGLE)
       && (total_angle > end_cond.value)) 
      || ((end_cond.type == EndCondition::DISTANCE)
	  && (distance_moved > end_cond.value))) {
    stop();
    // don't need to change y[]; just leave it as is.
    for (int i=0; i<DOF; ++i) {
      tc.qd[i]=tc.qdd[i]=0;
    }
    return;
  }

  // compute how big a WS step to take (based on velocity, distance)

  // compute the new joint angles for the step

}

ConstrainedForceTrajectory::~ConstrainedForceTrajectory() {
  free(old_y);
}
