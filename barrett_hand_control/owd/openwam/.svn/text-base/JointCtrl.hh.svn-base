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

#include <iostream>
#include <iomanip>

#include "Controller.hh"

#ifndef __JOINTCTRL_HH__
#define __JOINTCTRL_HH__

using namespace std;

/*
 * This is just an interface
 * Derive this class (PD, PID joint controllers) and you can point to it to 
 * control the WAM
 */

class JointCtrl : public Controller{
public:

  JointCtrl() : Controller() {}
  virtual ~JointCtrl(){}

  virtual double evaluate(double qs, double q, double dt) = 0;
  virtual void reset() = 0;

  // lil hack to read the subclass
  virtual ostream& put(ostream& s){return s;}
  virtual istream& get(istream& s){return s;}

  friend istream& operator >> (istream& s, JointCtrl& jctrl){
    return jctrl.get(s);
  }
  friend ostream& operator << (ostream& s, JointCtrl& jctrl){
    return jctrl.put(s);
  }

};

#endif
