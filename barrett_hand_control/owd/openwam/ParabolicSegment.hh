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

#ifndef PARABOLICSEGMENT_H
#define PARABOLICSEGMENT_H

#define TRAJ_TOLERANCE 0.003f

namespace OWD {
  class ParabolicSegment {
  public:
    int start_index, end_index;
    double start_pos, end_pos;
    double start_time, end_time;
    double time_a, time_v;  // times spent in initial/final accel and vel
    double max_vel, accel;
    typedef enum {
        DOWN = -1,
        CONST = 0,
        UP = 1
    } Direction;
    Direction dir;

    ParabolicSegment(int start_i,
                     double start_t,
                     double first_p,
                     double second_p);

    void fit_curve(double mv, double a);

    void refit_curve(double max_v, double max_a, double new_end_time, double new_accel_time);

    void evaluate(double &y, double &yd, double &ydd, double t);
    
    double calc_time(double value);
    
    bool inflection(double current_pos, double next_pos);
    
    void dump();
  };
}; // namespace OWD

#endif // PARABOLICSEGMENT_H

