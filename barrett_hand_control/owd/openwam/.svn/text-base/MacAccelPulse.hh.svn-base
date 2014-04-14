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

#ifndef MACACCELPULSE_H
#define MACACCELPULSE_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "BinaryData.hh"

#define VERBOSE 0

class MacAccelElement {
protected:
  double start_p, dist;
  double start_v, a, j;
  double start_t, dur;
  double atime;

public:
  static const double PI = 3.141592654;
  static const double PI2 =3.141592654 * 3.141592654;
  static const double epsilon = 1.0e-10;

  MacAccelElement() {};
  MacAccelElement(double start_pos, double start_vel,
		  double accel, double jerk, double start_time) :
    start_p(start_pos), start_v(start_vel), a(accel),
    j(jerk), start_t(start_time) {};

  MacAccelElement(BinaryData &bd) {
    start_p =bd.GetDouble();
    dist    =bd.GetDouble();
    start_v =bd.GetDouble();
    a       =bd.GetDouble();
    j       =bd.GetDouble();
    start_t =bd.GetDouble();
    dur     =bd.GetDouble();
    atime   =bd.GetDouble();
  }

  virtual BinaryData serialize() {
    BinaryData bd;
    bd.PutDouble(start_p);
    bd.PutDouble(dist);
    bd.PutDouble(start_v);
    bd.PutDouble(a);
    bd.PutDouble(j);
    bd.PutDouble(start_t);
    bd.PutDouble(dur);
    bd.PutDouble(atime);
    return bd;
  }

  inline virtual double start_pos() const {return start_p;}
  inline virtual double start_vel() const {return start_v;}
  inline virtual double start_time() const {return start_t;}
  inline virtual double accel() const {return a;}
  inline virtual double jerk() const {return j;}
  inline virtual double duration() const {return dur;}
  inline virtual double distance() const {return dist;}
  inline virtual double end_time() const {return start_t + dur;}
  inline virtual double end_pos() const {return start_p + dist;}
  inline virtual double delta_vel() const {return 0;}

  virtual double end_vel() const =0;
  virtual void set_constant_accel(double t) =0;
  virtual void extend_sustain_time_by(double t) =0;
  virtual void reset(double pos, double vel, double delta_vel,
		     double accel, double jerk, double t) =0;
  virtual void eval(double &q, double &qd, double &qdd, double t) const =0;
  virtual void dump() const {
    printf("     start_pos=%2.3f  dist=%2.3f  end_pos=%2.3f\n",
	   start_p,dist,start_p+dist);
    printf("     start_vel=%2.3f  accel=%2.3f\n",start_v,a);
    printf("     start_time=%2.3f  dur=%2.3f  end_time=%2.3f\n",
	   start_t,dur,start_t+dur);
  }

  virtual ~MacAccelElement() {}
};

class MacAccelPulse : public MacAccelElement {
protected:
  double sustain_t;
  double pa, sa, pb, sb; // intermediate positions and speeds
  double delta_v, end_v;

  MacAccelPulse() {}

  void fit_curve() {
    // delta_v has already been set positive
    atime = a/j * PI/2;
    if (VERBOSE) {
      printf("   MacAccelPulse::fit_curve  a=%2.4f j=%2.4f atime=%2.4f\n"
	     "                             delta_v=%2.4f a*atime=%2.4f\n",
	     a,j,atime,delta_v,fabs(a)*atime);
    }
    if (delta_v > (fabs(a)*atime + epsilon)) {
      // we need a sustain portion
      sustain_t = (delta_v - fabs(a)*atime) / fabs(a);
      if (VERBOSE) {
	printf("   MacAccelPulse: adding a sustain of t= %2.3f to achieve delta_v\n",sustain_t);
      }
      double actual_delta_v = fabs(a) * (atime + sustain_t);
      if (fabs(actual_delta_v - delta_v) > epsilon) {
	if (VERBOSE) {
	  printf("desired delta_v=%2.3f, actual delta_v=%2.3f\n",delta_v,
		 actual_delta_v);
	}
	throw"MacAccelPulse still has wrong delta_v after adding sustain";
      }
    } else if (delta_v < (fabs(a)*atime - epsilon)) {
      // calculate reduced acceleration to achieve velocity change
      double old_a=a;
      // recalc accel, still observing max jerk
      if (a>0) {
	a = sqrt(delta_v * j * 2.0 / PI);
      } else {
	a = -sqrt(- delta_v * j * 2.0 / PI);
      }
      atime = a/j * PI/2;
      if (VERBOSE) {
	printf("   MacAccelPulse: reducing accel from %2.3f to %2.3f to achieve requested delta_v\n",
	       old_a,a);
      }
      sustain_t=0;
      double actual_delta_v = fabs(a) * atime;
      if (fabs(actual_delta_v - delta_v) > epsilon) {
	if (VERBOSE) {
	  printf("desired delta_v=%2.3f, actual delta_v=%2.3f\n",delta_v,
		 actual_delta_v);
	}
	throw"MacAccelPulse still has wrong delta_v after reducing accel";
      }
    } else {
      // works out perfectly at max accel with no sustain
      // printf("MacAccelPulse: no sustain necessary (a*atime=%2.3f, delta_v only %2.3f)\n",fabs(a)*atime,delta_v);
      sustain_t = 0;
    }
    recalc_curve_constants();
  }

  void recalc_curve_constants() {
    // total time
    dur = 2*atime + sustain_t;

    // compute the total distance, in stages
    double atime2 = pow(atime,2);
    static const double PI2 = pow(PI,2);
    // first, the initial accel rise
    sa = start_v + a*atime/2;
    dist = start_v*atime + a*atime2*(.25-1/PI2);
    pa=start_p + dist;

    // next, the sustain (if any)
    sb = sa + sustain_t*a;
    dist += sustain_t * (sa + 0.5*a*sustain_t);
    pb = start_p + dist;
    
    // finally, the accel fall
    end_v = sb + a*atime/2;
    dist += sb*atime + a*atime2*(.25+1/PI2);

    delta_v=end_v-start_v;
  }
  
public:

  MacAccelPulse(double start_pos, double start_vel, double delta_vel,
		double accel, double jerk, double start_time) :
    MacAccelElement(start_pos,start_vel, accel, jerk, start_time),
    delta_v(fabs(delta_vel)) 
  {
    if (a == 0 || delta_v == 0 || j == 0) {
      if (VERBOSE) {
	printf("accel=%2.3f delta_vel=%2.3f jerk=%2.3f\n",
	       a,delta_v,j);
      }
      throw "MacAccelPulse requires non-zero accel, jerk, and delta_v";
    }
    if ((a >0 && j < 0) || (a<0 && j>0)) {
      throw "MacAccelPulse requires accel and jerk of the same sign";
    }

    fit_curve();
  }

  inline double end_vel() const {
    return end_v;
  }

  inline virtual double delta_vel() const {
    return delta_v;
  }

  void set_constant_accel(double t) {
    sustain_t = t;
    recalc_curve_constants();
  }

  void extend_sustain_time_by(double t) {
    sustain_t += t;
    recalc_curve_constants();
  }

  void reset(double pos, double vel, double delta_vel,
	     double accel, double jerk, double t) {
    start_p = pos; start_v = vel; delta_v = fabs(delta_vel);
    a = accel; j = jerk; start_t = t;
    if ((a>0 && j<0) || (a<0 && j>0)) {
      throw "Error during MacAccelPulse::reset(): accel and jerk must have the same sign";
    }
    fit_curve();
  }

  void eval(double &y, double &yd, double &ydd, double t) const {
    t -= start_t; // normalize to start at t=0
    if (t < 0) {
      t=0;
    }
    if (t > 2*atime + sustain_t) {
      t = 2*atime + sustain_t;
    }
    if (t<atime) {
      // initial rise
      y = start_p
	+ start_v*t
	+ a*0.25*t*t
	- a*atime*atime/PI/PI * 0.5*(sin(t*PI/atime - 0.5*PI) +1);
      yd = start_v
	+ a*0.5*t
	- a*0.5*atime/PI * cos(t*PI/atime - 0.5*PI);
      ydd = a*0.5 * (sin(t*PI/atime - 0.5*PI) +1);
      return;

    } else if (t<(atime + sustain_t)) {
      // linear sustain
      t -= atime;  // adjust t to be the time inside the sustain
      y = pa
	+ t * (sa + 0.5*a*t);
      yd = sa + a*t;
      ydd = a;
      return;

    } else {
      // final fall
      t -= atime + sustain_t; // adjust t to be inside the fall
      y = pb               // pb is correct
	+ sb*t              // sb matches sa
	+ a*0.25*t*t        // a is the same
	- a*atime*atime/PI/PI * 0.5*(sin(t*PI/atime + 0.5*PI) -1.0);
      yd = sb
	+ a*0.5*t
	- a*0.5*atime/PI * cos(t*PI/atime + 0.5*PI);
      ydd = a*0.5 * (sin(t*PI/atime + 0.5*PI) + 1);
      return;
    }
  }

  void dump() const {
    printf("   MacAccelPulse (a=%2.4f):\n",a);
    printf("     rise from t=%2.4f to t=%2.4f, pos=%2.4f to pos=%2.4f (d=%2.4f)\n",
	   start_t, start_t+atime,start_p,pa,pa-start_p);
    printf("          v=%2.4f to v=%2.4f (delta_v=%2.4f)\n",
	   start_v, sa, sa-start_v);
    double q,qd,qdd;
    eval(q,qd,qdd,start_t+atime);
    printf("          eval at t=%2.4f: pos=%2.4f, vel=%2.4f\n",
	   start_t+atime,q,qd);

    if (sustain_t > 0) {
      printf("     sustain from t=%2.4f to t=%2.4f, pos=%2.4f to pos=%2.4f (d=%2.4f)\n",
	     start_t+atime, start_t+atime+sustain_t, pa, pb, pb-pa);
      printf("          v=%2.4f to v=%2.4f (delta_v=%2.4f)\n",
	     sa, sb, sb-sa);
      eval(q,qd,qdd,start_t+atime+sustain_t);
      printf("          eval at t=%2.4f: pos=%2.4f, vel=%2.4f\n",
	     start_t+atime+sustain_t,q,qd);
    }
    printf("     fall from t=%2.4f to t=%2.4f, pos=%2.4f to pos=%2.4f (d=%2.4f)\n",
	   start_t+atime+sustain_t, start_t+2*atime+sustain_t,pb,start_p+dist,start_p+dist-pb);
    printf("          v=%2.4f to v=%2.4f (delta_v=%2.4f)\n",
	   sb, end_v, end_v-sb);
    eval(q,qd,qdd,start_t+2*atime+sustain_t);
    printf("          eval at t=%2.4f: pos=%2.4f, vel=%2.4f\n",
	   start_t+2*atime+sustain_t,q,qd);
  }
    
public:
  MacAccelPulse(BinaryData &bd) 
    // first let the base class extract itself
    : MacAccelElement(bd) {

    // now get our own members
    sustain_t=bd.GetDouble();
    pa       =bd.GetDouble();
    sa       =bd.GetDouble();
    pb       =bd.GetDouble();
    sb       =bd.GetDouble();
    delta_v  =bd.GetDouble();
    end_v    =bd.GetDouble();
    
  }

  virtual BinaryData serialize() {
    // first let the base class serialize itself
    BinaryData bd(MacAccelElement::serialize());

    // now add our own members
    bd.PutDouble(sustain_t);
    bd.PutDouble(pa);
    bd.PutDouble(sa);
    bd.PutDouble(pb);
    bd.PutDouble(sb);
    bd.PutDouble(delta_v);
    bd.PutDouble(end_v);

    return bd;
  }

};

class MacZeroAccel : public MacAccelElement {
protected:
  double vel;

public:
  MacZeroAccel(double start_pos, double start_vel, double end_pos, double start_time) {
    start_p = start_pos;
    start_v = start_vel;
    dist = end_pos - start_pos;
    if (((dist < -epsilon) && (start_v >= 0)) ||
	((dist > epsilon) && (start_v <= 0))) {
      if (VERBOSE) {
	printf("dist was %2.3f, start_v was %2.3f\n",dist,start_v);
      }
      throw "MacZeroAccel: velocity sign does not match sign of end-start";
    }
    start_t = start_time;
    dur = dist/start_v;
    a=j=0;
  }

  MacZeroAccel(BinaryData &bd)
    // first let the base class extract itself
    : MacAccelElement(bd) {

    // now get our own members
    vel = bd.GetDouble();
  }

  virtual BinaryData serialize() {
    // first let the base class serialize itself
    BinaryData bd(MacAccelElement::serialize());

    // now put our own members
    bd.PutDouble(vel);
    return bd;
  }
  
  inline double end_vel() const {
    return start_v;
  }

  void set_constant_accel(double t) {
    dur=t;
  }

  void extend_sustain_time_by(double t) {
    dur += t;
  }

  void reset(double pos, double vel, double dvel, double accel, 
	     double jerk, double t) {
    start_p = pos;
    start_v = vel;
    start_t = t;
    dur = dist/start_v;
    if (dvel != 0) {
      throw "MacZeroAccel: cannot reset to a non-zero delta velocity";
    }
    if (fabs(accel) > epsilon) {
      throw "MacZeroAccel: cannot reset to a non-zero accel";
    }
    if (fabs(jerk) > epsilon) {
      throw "MacZeroAccel: cannot reset to a non-zero jerk";
    }
    if (dur < 0) {
      throw "MacZeroAccel: new start velocity is the wrong sign";
    }
  }

  void eval(double &y, double &yd, double &ydd, double t) const {
    t -= start_t;
    if (t<0) {
      t = 0;
    }
    if (t>dur) {
      t = dur;
    }
    y = start_p + t*start_v;
    yd = start_v;
    ydd = 0;
    return;
  }

  void dump() const {
    printf("   MacZeroAccel:\n");
    MacAccelElement::dump();
  }
};

#endif // MACACCELPULSE_H
