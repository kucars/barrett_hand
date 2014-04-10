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
#include <string.h>

#include "R3.hh"

#ifndef __R6_HH__
#define __R6_HH__

using namespace std;

class R6{
public:
  R3 v;
  R3 w;

  static const int VX =  0;
  static const int VY =  1;
  static const int VZ =  2;
  static const int WX =  3;
  static const int WY =  4;
  static const int WZ =  5;

  R6(){clear();}
  R6(R3 v1, R3 w1) : v(v1), w(w1) {}
  R6(double x0, double x1, double x2, double x3, double x4, double x5)
    : v(x0, x1, x2), w(x3, x4, x5) {}

  double norm() const {return sqrt(v*v + w*w); }

  double normalize() {
    double d = norm();
    if (d>0) {
      v /= d;
      w /= d;
    } else {
      v=R3(1,0,0);
      w=R3(1,0,0);
    }
    return d;
  }

  void  clear(){v.clear(); w.clear();}
  
  friend R6 operator * (double s, const R6& x) {return R6(s*x.v,  s*x.w);}
  friend R6 operator * (const R6& r, double s) {return s*r;}
  friend R6 operator / (const R6& r, double s) {return (1.0/s)*r;}

  friend R6 operator + (const R6& r1, const R6& r2)
  {return R6(r1.v + r2.v, r1.w + r2.w); }

  friend R6 operator - (const R6& r1, const R6& r2)
  {return R6(r1.v - r2.v, r1.w - r2.w); }

  friend double operator * (const R6& r1, const R6& r2)
  {return r1.v*r2.v + r1.w*r2.w;}

  R6 &operator += (const R6 &rhs) {
    v += rhs.v;
    w += rhs.w;
    return *this;
  }

  R6 &operator -= (const R6 &rhs) {
    v -= rhs.v;
    w -= rhs.w;
    return *this;
  }

  friend ostream& operator <<  (ostream& s, const R6& r){
    int p;
    p = s.precision();
    s.precision(12);
    s.setf(ios::fixed, ios::floatfield);
    s.setf(ios::right, ios::adjustfield);
    
    s << setw(22) << r.v[0] 
	 << setw(22) << r.v[1] 
	 << setw(22) << r.v[2]
	 << setw(22) << r.w[0]
	 << setw(22) << r.w[1]
	 << setw(22) << r.w[2];
    
    s.setf(ios::left, ios::adjustfield);
    s.precision(p);
    return s;
  }
};

#endif
