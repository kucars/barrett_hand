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
#include <string.h>
#include "R3.hh"

#ifndef __SO3_HH__
#define __SO3_HH__

class SO3;

class so3{
public:
  R3 omega;
  double theta;

  // this is used to avoid singularities in so3. It's the minimum rotation.
  static const double EPSILON = 0.0000000000001;

  so3(){}
  so3(const R3& w, double t){omega = w; theta = t;}

  R3     w() const {return omega;}
  double t() const {return theta;}

  operator SO3() const;

  void normalize() {omega.normalize();}

  friend so3 operator * (double s, const so3& r){return so3(r.w(), r.t()*s);}
  friend so3 operator * (const so3& r, double s){return so3(r.w(), r.t()*s);}

  friend ostream& operator <<  (ostream& s, const so3& r){
    s << r.omega << " " << r.theta;
    return s;
  }
};

using namespace std;

class SO3{
private:

  double R[3][3];

public:

  static const int R11 = 0; static const int R12 = 1; static const int R13 = 2;
  static const int R21 = 3; static const int R22 = 4; static const int R23 = 5;
  static const int R31 = 6; static const int R32 = 7; static const int R33 = 8;

  SO3();

  SO3(const R3 rx, const R3 ry, const R3 rz);
  
  operator       double* ()        {return &R[0][0];}
  operator const double* () const  {return &R[0][0];}
  
  double operator [] (int i) const {return (&R[0][0])[i];}
  
  operator so3() const;  // convert to so3
  
  void normalize();

  friend SO3 operator * (const SO3& R1, const SO3& R2);
  friend R3  operator * (const SO3& R,  const R3& p);
  friend SO3 operator ! (const SO3& R);
  friend ostream& operator <<  (ostream& s, const SO3& r);
 
  void eye();
};

#endif
