/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
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

#include "Plugin.hh"
#include "WAM.hh"
#include "../openwamdriver.h"

namespace OWD {

  Plugin::Plugin() {
    // register this instance
    children.push_back(this);
  }

  Plugin::~Plugin() {
    for (std::vector<Plugin *>::iterator it=children.begin(); it!=children.end(); ++it) {
      if (*it == this) {
	children.erase(it);
	return;
      }
    }
  }

  void Plugin::Publish() {
  }

  void Plugin::PublishAll() {
    for (std::vector<Plugin *>::iterator it=children.begin(); it!=children.end(); ++it) {
      (*it)->Publish();
    }
  }


  bool Plugin::AddTrajectory(Trajectory *traj,
				 std::string &failure_reason) {
    if (!wamdriver) {
      failure_reason="invalid WamDriver pointer; should have been set by openwamdriver.cpp";
      return 0;
    }
    return wamdriver->AddTrajectory(traj, failure_reason);
  }

  bool Plugin::hand_move(std::vector<double> p) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_move(p) == OW_SUCCESS);
  }

  bool Plugin::hand_velocity(std::vector<double> v) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_velocity(v) == OW_SUCCESS);
  }

  bool Plugin::hand_torque(std::vector<double> t) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_torque(t) == OW_SUCCESS);
  }

  bool Plugin::hand_get_state(int state[4]) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_get_state((int32_t *) state) == OW_SUCCESS);
  }

  bool Plugin::hand_set_speed(const double v) {
    std::vector<double> velocities(4);
    velocities[0] = velocities[1] = velocities[2] = velocities[3] = v;
    return hand_set_speed(velocities);
  }

  bool Plugin::hand_set_speed(const std::vector<double> &v) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_set_speed(v) == OW_SUCCESS);
  }    

  bool Plugin::ft_tare() {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->ft_tare() == OW_SUCCESS);
  }

  //  const double* Plugin::Jacobian0() {
  //    return (double *) OWD::Kinematics::Jacobian0;
  //  }

  //  const double* Plugin::JacobianEE() {
  //    return (double *) OWD::Kinematics::JacobianEE;
  //  }

  const R6 Plugin::Jacobian_times_vector(JointPos v) {
    double result[6];
    OWD::Kinematics::Jacobian0_times_vector(&v[0], result);
    return R6(result[0], result[1], result[2], result[3], result[4], result[5]);
  }

  const JointPos Plugin::JacobianPseudoInverse_times_vector(R6 &v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::JacobianPseudoInverse_times_vector(v,result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS,result);
    return jp;
  }

  const JointPos Plugin::JacobianTranspose_times_vector(R6 &v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::Jacobian0Transpose_times_vector(v,result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS,result);
    return jp;
  }

  const JointPos Plugin::JacobianEETranspose_times_vector(R6 &v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::JacobianEETranspose_times_vector(v,result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS,result);
    return jp;
  }

  const JointPos Plugin::Nullspace_projection(JointPos v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::Nullspace_projection(&v[0], result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS, result);
    return jp;
  }

  TrajType Plugin::ros2owd_traj (owd_msgs::JointTraj &jt) {
    TrajType traj;
    if (jt.positions.size() != jt.blend_radius.size()) {
      throw "Bad ROS trajectory: mismatched number of points";
    }
    ROS_DEBUG_NAMED("trajectory","Converting trajectory of %zd points",jt.positions.size());
    for (unsigned int i=0; i<jt.positions.size(); ++i) {
      JointPos jp = jt.positions[i].j;
      TrajPoint tp(jp,jt.blend_radius[i]);
      traj.push_back(tp);
    }
    return traj;
  }

  bool Plugin::set_links(OWD::Link* links)
  {
    for(int l=Link::L0; l<=Link::Ln; l++)
      m_links[l] = links[l];
    return true;
  }

  SE3 Plugin::get_endeffector_pose(JointPos config)
  {
      // Update the links jont values first
      for(int j=Joint::J1; j<=Joint::Jn; j++)
      {
          m_links[j].theta(config[j-1]);
      }
      return OWD::Kinematics::forward_kinematics(m_links);
  }

  std::vector<double> Plugin::_arm_position;
  std::vector<double> Plugin::_target_arm_position;
  std::vector<double> Plugin::_pid_torque;
  std::vector<double> Plugin::_dynamic_torque;
  std::vector<double> Plugin::_trajectory_torque;
  std::vector<double> Plugin::_ft_force;
  std::vector<double> Plugin::_ft_torque;
  std::vector<double> Plugin::_filtered_ft_force;
  std::vector<double> Plugin::_filtered_ft_torque;
  int                 Plugin::_ft_saturation_state;
  std::vector<double> Plugin::_hand_position;
  std::vector<double> Plugin::_target_hand_position;
  std::vector<double> Plugin::_strain;
  std::vector<float> Plugin::_tactile_f1;
  std::vector<float> Plugin::_tactile_f2;
  std::vector<float> Plugin::_tactile_f3;
  std::vector<float> Plugin::_tactile_palm;
  std::vector<double> Plugin::_lower_jlimit;
  std::vector<double> Plugin::_upper_jlimit;
  std::vector<double> Plugin::_max_joint_vel;
  std::vector<double> Plugin::_joint_vel;
  std::vector<double> Plugin::_joint_accel;
  double Plugin::_max_jerk;
  double Plugin::_max_cartesian_velocity;
  std::vector<double> Plugin::_arm_velocity;
  SE3 Plugin::_endpoint;
  bool Plugin::_holdpos;
  OWD::Trajectory *Plugin::jointstraj;

  double Plugin::gravity;
#ifdef OWDSIM
  const bool Plugin::simulation=true;
#else
  const bool Plugin::simulation=false;
#endif // OWDSIM
  std::vector<Plugin *> Plugin::children;

  OWD::Link OWD::Plugin::m_links[OWD::Link::Ln];

  const std::vector<double> &Plugin::arm_position=Plugin::_arm_position;
  const std::vector<double> &Plugin::target_arm_position=Plugin::_target_arm_position;
  const std::vector<double> &Plugin::pid_torque=Plugin::_pid_torque;
  const std::vector<double> &Plugin::dynamic_torque=Plugin::_dynamic_torque;
  const std::vector<double> &Plugin::trajectory_torque=Plugin::_trajectory_torque;
  const std::vector<double> &Plugin::ft_force=Plugin::_ft_force;
  const std::vector<double> &Plugin::ft_torque=Plugin::_ft_torque;
  const std::vector<double> &Plugin::filtered_ft_force=Plugin::_filtered_ft_force;
  const std::vector<double> &Plugin::filtered_ft_torque=Plugin::_filtered_ft_torque;
  const int                 &Plugin::ft_saturation_state=Plugin::_ft_saturation_state;
  const std::vector<double> &Plugin::hand_position=Plugin::_hand_position;
  const std::vector<double> &Plugin::target_hand_position=Plugin::_target_hand_position;
  const std::vector<double> &Plugin::strain=Plugin::_strain;
  const std::vector<float> &Plugin::tactile_f1=Plugin::_tactile_f1;
  const std::vector<float> &Plugin::tactile_f2=Plugin::_tactile_f2;
  const std::vector<float> &Plugin::tactile_f3=Plugin::_tactile_f3;
  const std::vector<float> &Plugin::tactile_palm=Plugin::_tactile_palm;
  const std::vector<double> &Plugin::lower_jlimit=Plugin::_lower_jlimit;
  const std::vector<double> &Plugin::upper_jlimit=Plugin::_upper_jlimit;
  const std::vector<double> &Plugin::max_joint_vel=Plugin::_max_joint_vel;
  const std::vector<double> &Plugin::joint_vel=Plugin::_joint_vel;
  const std::vector<double> &Plugin::joint_accel=Plugin::_joint_accel;
  const double              &Plugin::max_jerk=Plugin::_max_jerk;
  const double              &Plugin::max_cartesian_velocity=Plugin::_max_cartesian_velocity;
  const std::vector<double> &Plugin::arm_velocity=Plugin::_arm_velocity;
  const SE3 &Plugin::endpoint=Plugin::_endpoint;
  const bool &Plugin::holdpos = Plugin::_holdpos;

  
  OWD::Link links[Link::Ln+1];

  WAM *Plugin::wam = NULL;
  WamDriver *Plugin::wamdriver = NULL;

  JSController::JSController(std::string controller_name) : 
    name(_name),
    _name(controller_name) {
    // make sure this name isn't already used
    if (find_controller(controller_name)) {
      static char errmsg[200];
      snprintf(errmsg,200,"Duplicate instance of controller %s",
	       controller_name.c_str());
      throw (const char *)errmsg;
    }

    // register this instance
    children.push_back(this);
  }

  JSController::~JSController() {
    for (std::vector<JSController *>::iterator it=children.begin(); it!=children.end(); ++it) {
      if (*it == this) {
	children.erase(it);
	return;
      }
    }
  }

  JSController *JSController::find_controller(std::string controller_name) {
    for (std::vector<JSController *>::iterator it=children.begin(); it!=children.end(); ++it) {
      if ((*it)->_name == controller_name) {
	return *it;
      }
    }
    return NULL;
  }

  std::vector<JSController *> JSController::children;

}; // namespace OWD
