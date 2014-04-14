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

#include "StepTraj.hh"
#include "Joint.hh"

namespace OWD {

  StepTraj::StepTraj(std::string trajid, int dof, int joint,
		     double *start_pos, double step_size)
    : Trajectory("StepTraj", trajid),
      nDOF(dof)
  {
    start_position.SetFromArray(nDOF,start_pos);
    end_position = start_position;
    duration=3.0;  // always 3 seconds total
    if ((joint<1) || (joint>Joint::Jn)) {
      static char msg[200];
      snprintf(msg,200,"Joint must be between 1 and %d",Joint::Jn);
      throw((const char *)msg);
    }
    end_position[joint-1] += step_size;
  }

  StepTraj::~StepTraj() {
  }
  
  void StepTraj::evaluate_abs(Trajectory::TrajControl &tc, double t) {
    if (tc.q.size() < (unsigned int)nDOF) {
      runstate=DONE;
      return;
    }
    time = t;
    if (time < 1.0) {
      tc.q=start_position;
      tc.qd.resize(nDOF,0);
      tc.qdd.resize(nDOF,0);
    } else {
      // after the 1 second point we step the joints to the final position
      tc.q=end_position;
      tc.qd.resize(nDOF,0);
      tc.qdd.resize(nDOF,0);
    }
    if (time>duration) {
      runstate = DONE;
    }
    
  }
  
}; // namespace OWD
