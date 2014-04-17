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
#include <vector>

#ifndef __STEPTRAJ_HH__
#define __STEPTRAJ_HH__

namespace OWD {

class StepTraj : public Trajectory {
private:
  int nDOF;

public:

  StepTraj(std::string trajid, int DOF, int joint, double *start_pos, double step_size);
  virtual ~StepTraj();

  // mandatory functions inherited from Trajectory
  virtual void evaluate_abs(Trajectory::TrajControl &tc, double t);
};

}; // namespace OWD

#endif
