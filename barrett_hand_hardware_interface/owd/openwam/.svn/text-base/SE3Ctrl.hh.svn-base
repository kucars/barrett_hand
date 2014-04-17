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

#include "Controller.hh"
#include "../openmath/SE3.hh"

#ifndef __SE3CTRL_HH__
#define __SE3CTRL_HH__

/*
 * This is just an interface that explain how a SE3 controller must work
 * Derive this class and you can point to it to control the WAM
 */

class SE3Ctrl : public Controller{
protected:
  SE3 se3ref;    // not really used during a motion
                 // mostly used during when idle
public:

  SE3Ctrl() : Controller() {}
  virtual ~SE3Ctrl(){}

  void set(const SE3& se3){  lock();  se3ref=se3;  unlock();  }

  virtual R6 evaluate(const SE3& E0s, const SE3& E0n, double dt) = 0;
  virtual void reset() = 0;

};

#endif
