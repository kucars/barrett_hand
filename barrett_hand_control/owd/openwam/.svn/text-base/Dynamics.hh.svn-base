/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include <vector>

#include "Joint.hh"

#include "../openmath/SE3.hh"
#include "../openmath/R6.hh"

//#include <mkl_lapack.h>
//#include <mkl_blas.h>

#include "Kinematics.hh"

using namespace OWD;

namespace Dynamics {
  extern R3 up; // direction of gravity vector
}

#ifndef __DYNAMICS_HH__
#define __DYNAMICS_HH__

void RNE(double* tau, Link* links, double* qd, double* qdd, R6& fext);
void CCG(double* ccg, Link* links, double* qd);

R6 bias_acceleration(Link* links, double *qd);
R6      acceleration(Link* links, double *qd, double* qdd);

void WSinertia(double A[6][6],               Link* links);
void JSinertia(double A[Link::Ln][Link::Ln], Link* links);

void WSdynamics(double* trq, Link* links, double* qd, R6& F);
std::vector<double> JSdynamics(Link* links, double* qd, double* qdd);

#endif
