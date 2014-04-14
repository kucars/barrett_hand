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
#include <strings.h>

#ifndef __R3_HH__
#define __R3_HH__

using namespace std;

class R3{
public:
  double x[3];

  static const int X = 0;  static const int Y = 1;  static const int Z = 2;

  R3(){x[0]=x[1]=x[2]=0.0;}
  R3(double x0, double x1, double x2){x[0]=x0;  x[1]=x1;  x[2]=x2;}
  R3(const R3 &r3) {
    x[0]=r3.x[0]; x[1]=r3.x[1]; x[2]=r3.x[2];
  }
  
  double norm() const {return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);}

  double normalize() {
    double d=norm();
    if (d > 0) {
      x[0] /= d;
      x[1] /= d;
      x[2] /= d;
    } else {
      // just make a unit vector; direction doesn't matter since original
      // vector was all zero
      x[0]=1;
      x[1]=x[2]=0;
    }
    return d; // in case someone wants our original magnitude
  }

  void clear() {
    bzero(x, 3*sizeof(double));
  }

  operator       double* ()        {return x;}
  operator const double* ()  const {return x;}
  
  //  double &operator [] (int i) {return x[i];}
  //  const double &operator [] (int i) const {return x[i];}

  /// \brief Scale by a double
  inline R3 &operator *= (const double d) {
    x[0]*=d; x[1]*=d; x[2]*=d;
    return *this;
  }

  /// \brief Scale by a double
  inline R3 &operator /= (const double d) {
    x[0]/=d; x[1]/=d; x[2]/=d;
    return *this;
  }

  /// \brief Add another R3 in place
  inline R3 &operator += (const R3 &rhs) {
    x[0] += rhs.x[0];
    x[1] += rhs.x[1];
    x[2] += rhs.x[2];
    return *this;
  }

  /// \brief Subtract another R3 in place
  inline R3 &operator -=(const R3 &rhs) {
    x[0] -= rhs.x[0];
    x[1] -= rhs.x[1];
    x[2] -= rhs.x[2];
    return *this;
  }

  friend R3     operator * (double s, const R3& r)
  {return R3(r.x[0]*s, r.x[1]*s, r.x[2]*s);}
  friend R3     operator * (const R3& r, double s) {return s*r;}
  friend R3     operator / (const R3& r, double s) {return (1.0/s)*r;}
  
  friend R3     operator + (const R3& r1, const R3& r2)
  {return R3(r1.x[0]+r2.x[0], r1.x[1]+r2.x[1], r1.x[2]+r2.x[2]);}
  
  friend R3     operator - (const R3& r1, const R3& r2)
  {return R3(r1.x[0]-r2.x[0], r1.x[1]-r2.x[1], r1.x[2]-r2.x[2]);}
  // cross product
  friend R3     operator ^ (const R3& r1, const R3& r2)
  {return R3(r1.x[1]*r2.x[2] - r1.x[2]*r2.x[1], 
	     r1.x[2]*r2.x[0] - r1.x[0]*r2.x[2], 
	     r1.x[0]*r2.x[1] - r1.x[1]*r2.x[0]);}
  
  friend double operator * (const R3& r1, const R3& r2)
  {return r1.x[0]*r2.x[0] + r1.x[1]*r2.x[1] + r1.x[2]*r2.x[2];}

  friend ostream& operator <<  (ostream& s, const R3& r){
    int p;
    p = s.precision();
    s.precision(12);
    s.setf(ios::fixed, ios::floatfield);
    s << r.x[0] << " " << r.x[1] << " " << r.x[2];
    s.precision(p);
    return s;
  }
};


#endif
