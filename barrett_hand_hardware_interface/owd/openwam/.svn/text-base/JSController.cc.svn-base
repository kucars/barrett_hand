#include "JSController.hh"
#include "Joint.hh"

using namespace OWD;

DefaultJSController::DefaultJSController(std::string myname) : 
  JSController(myname)
{
  // create the individual joint controllers
  //                                  Kp,  Kd,   Ki
  jcontrollers.push_back(JointCtrlPID(900, 10.0, 2.5));  // J1
  jcontrollers.push_back(JointCtrlPID(2500, 20.0, 5.0)); // J2
  jcontrollers.push_back(JointCtrlPID(600, 10.0, 2.5));  // J3
  jcontrollers.push_back(JointCtrlPID(500,  2.5, 0.5));  // J4
  if (Joint::Jn > 4) {
    jcontrollers.push_back(JointCtrlPID( 40,  0.5, 0.5));  // J5
    jcontrollers.push_back(JointCtrlPID( 40,  0.5, 0.5));  // J6
    jcontrollers.push_back(JointCtrlPID( 12,  0.05, 0.1)); // J7
  }
}

DefaultJSController::~DefaultJSController() {
}

std::vector<double> DefaultJSController::evaluate(const std::vector<double> q_target,
					   const std::vector<double> q,
					   const double dt) {
  if (q_target.size() != jcontrollers.size()) {
    throw "q_target array size does not match number of controllers";
  }
  if (q.size() != jcontrollers.size()) {
    throw "q array size does not match number of controllers";
  }
  std::vector<double> torques(jcontrollers.size());
  for (unsigned int i=0; i<jcontrollers.size(); ++i) {
    try {
      torques[i]=jcontrollers[i].evaluate(q_target[i], q[i], dt);
    } catch (const char *err) {
      static char errmsg[200];
      snprintf(errmsg,200,"Problem computing torque for joint %d: %s",
	       i+1, err);
      throw errmsg;
    }
  }
  return torques;
}

bool DefaultJSController::set_gains(unsigned int joint, std::vector<double> gains) {
  if (joint < jcontrollers.size()) {
    // might throw an error which will have to be caught by the
    // next level up.
    jcontrollers[joint].set_gains(gains);
    return true;
  }
  return false;
}

std::vector<double> DefaultJSController::get_gains(unsigned int joint) {
  if (joint < jcontrollers.size()) {
    return jcontrollers[joint].get_gains();
  } else {
    return std::vector<double>();
  }
}

void DefaultJSController::reset(unsigned int j) {
  if (j<jcontrollers.size()) {
    jcontrollers[j].reset();
  } else {
    throw "Index out of range";
  }
}

bool DefaultJSController::run(unsigned int j) throw (const char *) {
  if (j<jcontrollers.size()) {
    return jcontrollers[j].run();
  } else {
    throw "Index out of range";
  }
}

void DefaultJSController::stop(unsigned int j) {
  if (j<jcontrollers.size()) {
    jcontrollers[j].stop();
  } else {
    throw "Index out of range";
  }
}

bool DefaultJSController::active(unsigned int j) {
  if (j<jcontrollers.size()) {
    return jcontrollers[j].state() == Controller::RUN;
  } else {
    throw "Index out of range";
  }
}

int DefaultJSController::DOF() {
  return jcontrollers.size();
}

