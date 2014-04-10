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

#ifndef __R4_HH__
#define __R4_HH__

using namespace std;

class R4{
private:
  double x[4];

public:

  static const int X = 0;
  static const int Y = 1;
  static const int Z = 2;

  R4(){bzero(x, 4*sizeof(double));}
  R4(double x0, double x1, double x2, double x3)
  {x[0]=x0;      x[1]=x1;      x[2]=x2;      x[3]=x3;}
  R4(const R3& r3)
  {x[0]=r3[0];   x[1]=r3[1];   x[2]=r3[2];   x[3]=1.0;}

  double norm() const 
  {return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);}

  operator       double* ()        {return x;}
  operator const double* () const  {return x;}

  double operator [] (int i) const {return x[i];}
    
  friend R4     operator * (double s, const R4& x)
  {return R4(s*x[0], s*x[1], s*x[2], s*x[3]);}
  friend R4     operator * (const R4& r, double s) {return s*r;}
  friend R4     operator / (const R4& r, double s) {return (1.0/s)*r;}

  friend R4     operator + (const R4& x1, const R4& x2)
  {return R4(x1[0]+x2[0], x1[1]+x2[1], x1[2]+x2[2], x1[3]+x2[3]);}

  friend R4     operator - (const R4& x1, const R4& x2)
  {return R4(x1[0]-x2[0], x1[1]-x2[1], x1[2]-x2[2], x1[3]-x2[3]);}

  friend double operator * (const R4& x1, const R4& x2)
  {return x1[0]*x2[0] + x1[1]*x2[1] + x1[2]*x2[2] + x1[3]*x2[3];}

  friend ostream& operator <<  (ostream& s, const R4& r){
    int p;
    p = cout.precision();
    cout.precision(12);
    cout.setf(ios::fixed, ios::floatfield);
    cout << r[0] << " " << r[1] << " " << r[2] << " " << r[3];
    cout.precision(p);
    return s;
  }

};
#endif
