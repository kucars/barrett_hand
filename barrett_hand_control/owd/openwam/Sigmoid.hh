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

#include "Profile.hh"

#include <iostream>
#include <math.h>

#ifndef __SIGMOID_HH__
#define __SIGMOID_HH__

using namespace std;
class Sigmoid : public Profile{
private:

  double x1, x2, x3;   // sigmoid parameters
  double ts;           // time shift
  double ydmax;
  
public:

  static const double VMAX = 0.1;
  static const double WMAX = 0.2;

  Sigmoid(double y) : Profile() {ydmax = y;}
  ~Sigmoid(){}

  void init(double y1, double y2);
  void evaluate(double& y, double& yd, double& ydd, double dt);
};

#endif
