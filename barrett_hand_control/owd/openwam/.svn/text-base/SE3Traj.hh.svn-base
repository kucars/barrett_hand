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

#include "Trajectory.hh"
#include "Profile.hh"

#include "../openmath/SE3.hh"

#ifndef __SE3TRAJ_HH__
#define __SE3TRAJ_HH__

using namespace std;

/*
 * Implements a trajectory generator in Cartesian space.
 * All that you need is two Profiles that determines how the rotational and 
 * linear motions will be generated
 */

class SE3Traj : public OWD::Trajectory{

protected:
  SE3 E01 ;  // initial (frame 1) transformations 
             // relative to base frame (frame 0)
  SO3 R01;   // just the rotation of the transformation above 
             // (used to rotate the vectors back in frame 0 at each evaluation)

  R3 u1;     // unit vector of the translation (relative to frame 1)
  R3 w1;     // rotation unit vector (relative to frame 1)

  Profile *linprof;
  Profile *rotprof;

public:

  SE3Traj(const SE3& e01, const SE3& e02, 
	  Profile* lp, Profile* rp) : OWD::Trajectory("SE3Traj",""){
    E01=e01; 
    linprof=lp;
    rotprof=rp;

    R01 = (SO3)e01;             // extract the rotation component (so we
                                // don't have to do it at every evaluation)

    SE3 E12 = (e01^-1)*e02;     // get the transformation relative to frame 1

    R3 t1 = (R3)E12;            // extract the translation relative to frame 1
    double d = t1.norm();
    if(d == 0.0)    u1 = R3();  // zero direction vector if no translation
    else            u1 = t1/d;  // direction vector for the translation
    linprof->init( 0.0, d );    // initialize the linear profile
    
    so3 r12 = (so3)E12;            // extract the rotation relative to frame 1
    w1 = r12.w();                  // extract the rotation vector
    rotprof->init( 0.0, r12.t() ); // initialize the rotation profile
    stop();
  }

  // word of caution! the trajectory DELETE the profiles
  ~SE3Traj(){delete linprof; delete rotprof;}

  void run (){linprof->run();  rotprof->run();}
  void stop(){linprof->stop(); rotprof->stop();}
  int state(){
    if(linprof->state() == Profile::RUN) return OWD::Trajectory::RUN;
    if(rotprof->state() == Profile::RUN) return OWD::Trajectory::RUN;
    return OWD::Trajectory::STOP;
  }

  /*
   * Return the desired forward kinematics, the desire linear velocity and
   * acceleration, as well as the desired rotational velocity and acceleration
   */
  void evaluate(SE3& E0ns, R3& pd, R3& pdd, R3& wd, R3& wdd, double dt){
    if(state() == OWD::Trajectory::RUN){
      double x, xd, xdd, theta, thetad, thetadd;
  
      linprof->evaluate(x,     xd,     xdd,     dt);
      rotprof->evaluate(theta, thetad, thetadd, dt);

      SE3 E1ns( so3(w1, theta), u1*x );//desired transformation rel to frame 1
      E0ns = E01*E1ns;                 //desired FK relative to base frame
    
      pd = R01*(u1*xd);                //desired linear velocity (frame 0)
      pdd = R01*(u1*xdd);              //desired linear acceleration (frame 0)
    
      wd = R01*(w1*thetad);            //desired angular velocity (frame 0)
      wdd = R01*(w1*thetadd);          //desired angular acceleration(frame 0)
    }
  }

  void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {};
};

#endif
