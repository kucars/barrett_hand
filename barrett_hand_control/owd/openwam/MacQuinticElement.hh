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

#ifndef MACQUINTICELEMENT_H
#define MACQUINTICELEMENT_H

#include "MacAccelPulse.hh"
#include "Trajectory.hh"
#include "BinaryData.hh"
#include <string.h>

class MacQuinticElement {
public:

  // positions are n-dof vectors of joint angles
  OWD::JointPos start_pos, end_pos;

  // times are with respect to the start of the full trajectory
  double start_time, duration;

  double max_path_velocity, max_path_acceleration;

  static const double VEL_MAX = -1; // flag to specify max velocity
  static const double PI = 3.141592654;

  char *reason;

  virtual void setStartVelocity(double v) = 0;
  virtual void setEndVelocity(double v) = 0;
  inline virtual void setStartTime(double t) {start_time = t;}
  virtual void BuildProfile() = 0;
  
  virtual double StartVelocity() const =0;
  virtual double EndVelocity() const =0;
  virtual double PathVelocity() const=0;
  virtual double PathAcceleration() const=0;
  inline virtual double MaxPathVelocity() const {return max_path_velocity;}
  inline virtual double MaxPathAcceleration() const {return max_path_acceleration;}

  inline virtual double StartTime() const {return start_time;}
  inline virtual double EndTime() const {
    if (duration<0) {
      throw "Must BuildProfile() before getting time";
    } else {
      return start_time+duration;
    }
  }
  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double t)=0;
  virtual double calc_time(OWD::JointPos value) const =0;

  MacQuinticElement(OWD::JointPos start, OWD::JointPos end)
    : start_pos(start), end_pos(end), start_time(0), duration(-1) {
    reason = strdup("initial limits");
  }

  virtual ~MacQuinticElement() {}

  virtual void dump() {
    //    printf("  start_pos=%2.3f  end_pos=%2.3f\n",start_pos,end_pos);
    printf("  start_time=%2.3f  dur=%2.3f  end_time=%2.3f\n",
	   start_time,duration,start_time+duration);
  }

  MacQuinticElement(BinaryData &bd) {
    start_pos            =bd.GetDoubleVector();
    end_pos              =bd.GetDoubleVector();
    start_time           =bd.GetDouble();
    duration             =bd.GetDouble();
    max_path_velocity    =bd.GetDouble();
    max_path_acceleration=bd.GetDouble();
    reason=strdup(        bd.GetString().c_str());
  }

  virtual BinaryData serialize(int firstdof=0, int lastdof=-1) {
    if (lastdof == -1) {
      lastdof = start_pos.size()-1;
    }

    OWD::JointPos sp,ep;
    sp.insert(sp.begin(),
	      start_pos.begin()+firstdof,
	      start_pos.begin()+lastdof+1);
    ep.insert(ep.begin(),
	      end_pos.begin()+firstdof,
	      end_pos.begin()+lastdof+1);

    BinaryData bd;
    // Insert all our data members
    bd.PutDoubleVector(sp);
    bd.PutDoubleVector(ep);
    bd.PutDouble(      start_time);
    bd.PutDouble(      duration);
    bd.PutDouble(      max_path_velocity);
    bd.PutDouble(      max_path_acceleration);
    std::string s(     reason); bd.PutString(s);

    return bd;
  }

};

#endif // MACQUINTICELEMENT_H
