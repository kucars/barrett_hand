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

#include "../openmath/SE3.hh"

#ifndef __DH_HH__
#define __DH_HH__

class DH{
  double alpha, a, d, theta_naught, theta;
  double ca, sa;
  R3 pstar;
  SE3 H;
public:

  DH(){}
  DH(double alpha, double a, double d, double _theta_naught)
    : theta_naught(_theta_naught)
  {
    this->alpha = alpha;    this->a = a;  this->d = d;
    ca = cos(alpha), sa = sin(alpha);
    pstar = R3(a, d*sa, d*ca);
    
    t(0);
  }

  operator SO3() const {return (SO3)H;}
  operator SE3() const {return H;}

  R3 ps() const {return pstar;}

  void t(double t){
    theta = t;
    double ct = cos(theta_naught+theta), st = sin(theta_naught+theta);
    H = SE3( SO3( R3(ct, st, 0), R3(-st*ca, ct*ca, sa), R3(st*sa, -ct*sa, ca)),
	      R3(a*ct, a*st, d) );
  }
};

#endif
