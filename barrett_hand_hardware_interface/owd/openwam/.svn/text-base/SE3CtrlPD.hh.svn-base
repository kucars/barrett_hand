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

#include "SE3Ctrl.hh"

#include "../openmath/R6.hh"

//#include <mkl_lapack.h>
//#include <mkl_blas.h>

#ifndef __SE3CtrlPD_HH__
#define __SE3CtrlPD_HH__

class SE3CtrlPD : public SE3Ctrl{
private:
  double Kp[6][6], Kv[6][6];
  R6 laste;

public:

  SE3CtrlPD() : SE3Ctrl() {
    bzero((void*)(&Kp[0][0]), 6*6*sizeof(double));
    bzero((void*)(&Kv[0][0]), 6*6*sizeof(double));
    Kp[0][0] = Kp[1][1] = Kp[2][2] =  -1000;  // ballpark values
    Kp[3][3] = Kp[4][4] = Kp[5][5] =  -1000;
    Kv[0][0] = Kv[1][1] = Kv[2][2] =    -10;
    Kv[3][3] = Kv[4][4] = Kv[5][5] =    -10;
    reset();
  }

  R6 evaluate(const SE3& E0ns, const SE3& E0n, double dt){
    if(state() == Controller::RUN){
      set(E0ns);  // backup the command
      lock();

      R3 p = (R3)E0n;     // current translation
      R3 ps = (R3)E0ns;   // desired translation
    
      R3 n(E0n[SE3::NX], E0n[SE3::NY], E0n[SE3::NZ]); // current orientation
      R3 o(E0n[SE3::OX], E0n[SE3::OY], E0n[SE3::OZ]);
      R3 a(E0n[SE3::AX], E0n[SE3::AY], E0n[SE3::AZ]);
      
      R3 ns(E0ns[SE3::NX], E0ns[SE3::NY], E0ns[SE3::NZ]); // desire orientation
      R3 os(E0ns[SE3::OX], E0ns[SE3::OY], E0ns[SE3::OZ]);
      R3 as(E0ns[SE3::AX], E0ns[SE3::AY], E0ns[SE3::AZ]);

      SO3 Rn0 = !((SO3)E0n); 
      // compute the error in the base frame (E0ns, E0n are in the base frame)
      // translation error: t - td
      // rotation error: 1/2*(nxnd + oxod + axad) (Luh, Walker, Paul 1980)
      // rotate the error in the frame of the end effector  with Rn0
      R6 e( Rn0*(p-ps), Rn0*(0.5*( (ns^n) + (os^o) + (as^a) )) );
      // error time derivative
      R6 ed( (e-laste)/dt );
      
      R6 Kpe(Kp[0][0]*e.v[0], Kp[1][1]*e.v[1], Kp[2][2]*e.v[2],
	     Kp[3][3]*e.w[0], Kp[4][4]*e.w[1], Kp[5][5]*e.w[2]);
      R6 Kved(Kv[0][0]*ed.v[0], Kv[1][1]*ed.v[1], Kv[2][2]*ed.v[2],
	      Kv[3][3]*ed.w[0], Kv[4][4]*ed.w[1], Kv[5][5]*ed.w[2]);
      /*
      int stride1 =   sizeof(double);
      int stride6 = 6*sizeof(double);
      ippmMul_mv_64f(&Kp[0][0],        stride6, stride1, 6, 6,
		     (const double*)e,          stride1, 6,
		     (double*)Kpe,              stride1);
      ippmMul_mv_64f(&Kv[0][0],         stride6, stride1, 6, 6,
		     (const double*)ed,          stride1, 6,
		     (double*)Kved,              stride1);
      */

      laste = e;
      unlock();
      
      return R6(Kpe + Kved);
    }
    return R6();
  }

  void reset(){
    lock();
    laste.clear();
    unlock();
  }

};
#endif
