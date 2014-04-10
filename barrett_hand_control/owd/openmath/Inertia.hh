/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SO3.hh"

#ifndef __INERTIA_HH__
#define __INERTIA_HH__

class Inertia{
  double I[3][3];

public:

  static const int Ixx = 0; static const int Ixy = 1; static const int Ixz = 2;
  static const int Iyx = 3; static const int Iyy = 4; static const int Iyz = 5;
  static const int Izx = 6; static const int Izy = 7; static const int Izz = 8;
  
  Inertia(){bzero(&I[0][0], 3*3*sizeof(double));}

  Inertia(double xx,double xy,double xz,double yy,double yz, double zz){
    I[0][0] = xx;  I[0][1] = xy;  I[0][2] = xz;
    I[1][0] = xy;  I[1][1] = yy;  I[1][2] = yz;
    I[2][0] = xz;  I[2][1] = yz;  I[2][2] = zz;
  }
  Inertia(double xx, double xy, double xz, double yy, double yz, double zz,
	  double s){
    I[0][0] = xx*s;  I[0][1] = xy*s;  I[0][2] = xz*s;
    I[1][0] = xy*s;  I[1][1] = yy*s;  I[1][2] = yz*s;
    I[2][0] = xz*s;  I[2][1] = yz*s;  I[2][2] = zz*s;
  }

  operator       double* ()       {return &I[0][0];}
  operator const double* () const {return &I[0][0];}

  double operator [] (int i) const {return (&I[0][0])[i];}

  friend Inertia operator + (const Inertia& I1, const Inertia& I2){
    Inertia I3;
    const double *i1=(const double*)I1, *i2=(const double*)I2;
    double *i3=(double*)I3; 
    for(int i=0; i<9; i++)
      *i3++ = *i1++ + *i2++;
    return I3;
  }

  friend Inertia operator - (const Inertia& I1, const Inertia& I2){
    Inertia I3;
    const double *i1=(const double*)I1, *i2=(const double*)I2;
    double *i3=(double*)I3; 
    for(int i=0; i<9; i++)
      *i3++ = *i1++ - *i2++;
    return I3;
  }

  friend R3      operator * (const Inertia& I, const R3& p){
    return R3( I.I[0][0]*p[0] + I.I[0][1]*p[1] + I.I[0][2]*p[2],
	       I.I[1][0]*p[0] + I.I[1][1]*p[1] + I.I[1][2]*p[2],
	       I.I[2][0]*p[0] + I.I[2][1]*p[1] + I.I[2][2]*p[2]);
  }

  friend Inertia operator * (const Inertia& I1, double s){
    Inertia I2;
    const double *i1=(const double*)I1;
    double *i2=(double*)I2; 
    for(int i=0; i<9; i++)  *i2++ = *i1++ * s;
    return I2;
  }

  friend Inertia operator * (double s, const Inertia& I) {return I*s;}

  // That's kind of wrong, a rotated symmetric matrix isn't symmetric. So
  // rotating an Inertia matrix isn't an inertia matrix (which are symmetric)
  // ... but it makes my life so much easier...
  friend Inertia operator * (const Inertia& I, const SO3& R);
  friend Inertia operator * (const SO3& R, const Inertia& I);

  friend ostream& operator <<  (ostream& s, const Inertia& i);
};

#endif
