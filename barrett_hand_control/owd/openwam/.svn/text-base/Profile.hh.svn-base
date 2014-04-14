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


#ifndef __PROFILE_HH__
#define __PROFILE_HH__

using namespace std;

class Profile{
private:
  int s;

protected:
  double t;            // current time
  double t1;           // start time
  double t2;           // stop time

public:

  static const int STOP = 0;
  static const int RUN = 1;

  Profile(){stop(); t=0.0; t1=0.0; t2=0.0;}
  virtual ~Profile(){}

  void run(){s = Profile::RUN;}
  void stop(){s = Profile::STOP;}
  int  state(){return s;}

  virtual void init(double y1, double y2) = 0;
  virtual void evaluate(double& y, double& yd, double& ydd, double dt)=0;
};

#endif
