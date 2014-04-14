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


#include "JointCtrl.hh"

#ifndef __JOINTCTRLPID_HH__
#define __JOINTCTRLPID_HH__

class JointCtrlPID : public JointCtrl{

private:
  double Kp, Kd, Ki;
  double last_e;
  int last_e_valid;
  double se;         // Sum of error
  static const double Isaturation = 5;

    double _GetSign(double in)
    {
        if(in >= 0)
            return 1.0;
        else
            return -1.0;

    }

public:

  JointCtrlPID(double _kp, double _kd, double _ki){
    Kp=_kp;
    Kd=_kd;
    Ki=_ki;
    reset();
  }
  
  double evaluate(double qs, double q, double dt){
    if(state() == Controller::RUN){
      //      lock();
      double e = qs - q;
      double ed = last_e_valid ? (e - last_e)/dt : 0.0;
      se += e;
      last_e = e;
      last_e_valid = 1;

      double Isign = _GetSign(se);
      if(Isign*se > Isaturation)
      {
          se = Isign*Isaturation;
      }
      //      unlock();
      return Kp*e + Kd*ed + Ki*se;
    }
    return 0.0;
  }


  void reset(){
    //    lock();
    s = Controller::STOP;
    last_e = 0;
    last_e_valid = 0;
    se=0;
    //    unlock();
  }

  inline void set_gains(std::vector<double> gains) {
    if (gains.size() != 3) {
      throw "expected a vector of 3 gains (Kp, Kd, Ki)";
    }
    Kp=gains[0];
    Kd=gains[1];
    if (gains[2] != Ki) {
      // we'll use a lot of care in changing the integral gain, since
      // there may have been a build-up of the integrated error.
      if (gains[2] == 0) {
	// no problem
	Ki = 0;
      } else {
	// how much torque was being created by the integral term?
	double oldtorque = Ki * se;
	// if the new torque would be higher with the existing integrated
	// error, then drop the integrator value to keep the same torque
	if (gains[2] * se > oldtorque) {
	  se = oldtorque / gains[2];
	} else if (gains[2] * se < - oldtorque) {
	  se = - oldtorque / gains[2];
	}
	Ki = gains[2];
      }
    }
  }

  inline std::vector<double> get_gains() const {
    std::vector<double> gains;
    gains.push_back(Kp);
    gains.push_back(Kd);
    gains.push_back(Ki);
    return gains;
  }

};

#endif
