/***********************************************************************

  Copyright 2012 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#ifndef JSCONTROLLER_HH
#define JSCONTROLLER_HH

#include <vector>
#include "JointCtrlPID.hh"
#include "Plugin.hh"

namespace OWD {

  class DefaultJSController : public JSController {
  public:
    
    DefaultJSController(std::string myname);
    ~DefaultJSController();
    
    virtual std::vector<double> evaluate(const std::vector<double> q_target,
					 const std::vector<double> q,
					 const double dt);

    virtual bool set_gains(unsigned int joint, std::vector<double> gains);
    virtual std::vector<double> get_gains(unsigned int joint);
    virtual int DOF();

    virtual void reset(unsigned int j);
    virtual bool run(unsigned int j) throw (const char *);
    virtual void stop(unsigned int j);
    virtual bool active(unsigned int j);

  protected:
    std::vector<JointCtrlPID> jcontrollers;

  };

};

  
#endif // JSCONTROLLER_HH
