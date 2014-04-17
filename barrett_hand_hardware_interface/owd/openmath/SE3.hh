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
#include <iostream>
#include <iomanip>
#include <math.h>

#include "SO3.hh"
#include "ZYX.hh"
#include "R4.hh"

#ifndef __SE3_HH__
#define __SE3_HH__

class SE3{
private:
  double H[4][4];

  void setR3(const R3& t);
  void setSO3(const SO3& R);

public:

  static const int R11 = 0; static const int R12 = 1; static const int R13 = 2;
  static const int R21 = 4; static const int R22 = 5; static const int R23 = 6;
  static const int R31 = 8; static const int R32 = 9; static const int R33 =10;

  static const int TX =  3;  
  static const int TY =  7;
  static const int TZ = 11;

  static const int NX =  0; static const int OX =  1; static const int AX =  2;
  static const int NY =  4; static const int OY =  5; static const int AY =  6;
  static const int NZ =  8; static const int OZ =  9; static const int AZ = 10;

  SE3(){eye();}
  SE3(const SO3 R, const R3 t){  eye();  setSO3(      R );  setR3(t);}
  SE3(const so3 R, const R3 t){  eye();  setSO3( (SO3)R );  setR3(t);}
  SE3(const ZYX R, const R3 t){  eye();  setSO3( (SO3)R );  setR3(t);}

  operator       double* ()        {return &H[0][0];}
  operator const double* () const  {return &H[0][0];}

  double operator [] (int i) const {return (&H[0][0])[i];}
  
  operator so3() const;
  operator SO3() const 
  { return SO3( R3(H[0][0], H[1][0], H[2][0]),
		R3(H[0][1], H[1][1], H[2][1]),
		R3(H[0][2], H[1][2], H[2][2]) ); }
  
  operator R3()  const { return R3( H[0][3], H[1][3], H[2][3] ); }
  
  friend R4  operator * (const SE3& h, const R4& p){
  return 
  R4(h[SE3::R11]*p[R4::X]+h[SE3::R12]*p[R4::Y]+h[SE3::R13]*p[R4::Z]+h[SE3::TX],
     h[SE3::R21]*p[R4::X]+h[SE3::R22]*p[R4::Y]+h[SE3::R23]*p[R4::Z]+h[SE3::TY],
     h[SE3::R31]*p[R4::X]+h[SE3::R32]*p[R4::Y]+h[SE3::R33]*p[R4::Z]+h[SE3::TZ],
     1.0);
  }

  friend SE3 operator * (const SE3& E1, const SE3& E2);
  friend SE3 operator ^ (const SE3& E,  int i);

  // multiplication by a double will scale the translation and rotation.
  // it's intended for scaling a difference between two SE3s.  For instance,
  // you can find the "midpoint" between two SE3s with 0.5 * (SE3_second - SE3_first)
  SE3 operator*(double a) const;
  friend SE3 operator * (double a, const SE3 &s);

  // differences between two SE3s can be computed by subtracting them.  The
  // resulting SE3 will be the relative transform from one to the other.
  // differences can be added back with the + operator.
  SE3 operator +(const SE3 &s);
  SE3 operator -(const SE3 &s);
  
  friend ostream& operator <<  (ostream& s, const SE3& r);
  
  void eye(){
    bzero(&H[0][0], 4*4*sizeof(double)); 
    H[0][0] = H[1][1] = H[2][2] = H[3][3] = 1.0;
  }
};

#endif
