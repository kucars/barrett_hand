/***********************************************************************

  Copyright 2007-2011 Carnegie Mellon University and Intel Corporation
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

/*
 *  openwamdriver.cpp
 *  A ROS driver for the Barrett WAM arm.
 *
 */

// #define CALIBRATE_ON_STARTUP
// to do:
// test velocity limits

// clear the traj when done
// decide which value to use for queued trajectories; last point of previous
// traj or current point.  i guess wait to decide until you're ready to execute.//
// good scenario:  multiple trajs come in, each one matches previous, ok to
// chain together, doesn't hurt to add current position right before executing
// each one.
//
// bad scenario: multiple trajs, one is aborted.  don't want to continue with
// future trajs.  could just clear queue when you abort one.


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <dlfcn.h>
#include <string>
#include <iostream>

#ifdef BUILD_FOR_SEA
#include <owd_msgs/IndexedJointValues.h>
#include <owd_msgs/WamSetupSeaCtrl.h>
#include <positionInterface/SmoothArm.h>
#endif // BUILD_FOR_SEA

#define SAFETY_MODULE 10

extern int MECH,AP,ZERO,IFAULT;
extern int ADDR, VALUE, MODE;
extern int dyn_active_link;

extern double fv[8];  // viscous friction from Dynamics.cc
extern double fs[8];  // static friction from Dynamics.cc

#include <term.h>

#include "openwamdriver.h"
#include "openwam/Trajectory.hh"
#include "openwam/ParabolicSegment.hh"
#include "openwam/PulseTraj.hh"
#include "openwam/ServoTraj.hh"
#include "openwam/StepTraj.hh"
#include "openwam/CANbus.hh"
#include "openwam/WAM.hh"
#include "openwam/Plugin.hh"

namespace OWD {

WamDriver::WamDriver(int canbus_number, int bh_model, bool forcetorque, bool tactile) :
  nJoints(Joint::Jn),
  BH_model(bh_model), ForceTorque(forcetorque), Tactile(tactile),
  running(true),
  modified_j1(false)
{
  ros::NodeHandle n("~");
  bool log_canbus_data;
  n.param("log_canbus_data",log_canbus_data,false);

#ifndef BH280_ONLY
  bus=new CANbus(canbus_number, Joint::Jn, bh_model==280, forcetorque, tactile, log_canbus_data);
#else
  bus=new CANbus(canbus_number, 0, bh_model==280, forcetorque, tactile, log_canbus_data);
#endif // BH280_ONLY

  

  // motion limits
  gravity_comp_value=1.0;
  min_accel_time=0.75;
  Plugin::_max_joint_vel.resize(7);
  Plugin::_joint_vel.resize(7);
  Plugin::_joint_accel.resize(7);
  Plugin::_max_joint_vel[0]=2.5; // J1
  Plugin::_max_joint_vel[1]=2.5; // J2
  Plugin::_max_joint_vel[2]=2.5; // J3
  Plugin::_max_joint_vel[3]=2.5; // J4
  Plugin::_max_joint_vel[4]=2.5; // J5
  Plugin::_max_joint_vel[5]=2.5; // J6
  Plugin::_max_joint_vel[6]=2.5; // J7
  Plugin::_max_jerk = 10.0 * 3.141592654;
  n.param("max_cartesian_velocity",Plugin::_max_cartesian_velocity,0.7);
  bus->max_cartesian_velocity = Plugin::_max_cartesian_velocity;

  // working velocity limits
  Plugin::_joint_vel[0]=0.75; // J1
  Plugin::_joint_vel[1]=0.75; // J2
  Plugin::_joint_vel[2]=2;    // J3
  Plugin::_joint_vel[3]=2;    // J4
  Plugin::_joint_vel[4]=2.5;  // J5
  Plugin::_joint_vel[5]=2.5;  // J6
  Plugin::_joint_vel[6]=2.5;  // J7

  for (unsigned int j=0; j<nJoints; ++j) {
    Plugin::_joint_accel[j] = Plugin::_joint_vel[j]/min_accel_time;
  }
  last_trajectory_error[0]=0;

#ifdef FAKE7
  wamstate.positions.resize(7,0.0f);
  wamstate.jpositions.resize(4,0.0f);
  wamstate.velocities.resize(7,0.0f);
  wamstate.torques.resize(7,0.0f);
  waminternals.positions.resize(7,0.0f);
  waminternals.total_torque.resize(7,0.0f);
  waminternals.dynamic_torque.resize(7,0.0f);
  waminternals.trajectory_torque.resize(7,0.0f);
  waminternals.sim_torque.resize(7,0.0f);
  waminternals.joint_offsets.resize(7,0.0f);
  owd_msgs::PIDgains empty_gains;
  waminternals.gains.resize(7);

  // Plugin structures
  Plugin::_arm_position.resize(7);
  Plugin::_target_arm_position.resize(7);
  Plugin::_pid_torque.resize(7);
  Plugin::_dynamic_torque.resize(7);
  Plugin::_trajectory_torque.resize(7);
  Plugin::_arm_velocity.resize(7);
  
#else
  wamstate.positions.resize(nJoints,0.0f);
  wamstate.jpositions.resize(4,0.0f);
  wamstate.velocities.resize(nJoints,0.0f);
  wamstate.torques.resize(nJoints,0.0f);
  waminternals.positions.resize(nJoints,0.0f);
  waminternals.total_torque.resize(nJoints,0.0f);
  waminternals.dynamic_torque.resize(nJoints,0.0f);
  waminternals.trajectory_torque.resize(nJoints,0.0f);
  waminternals.sim_torque.resize(nJoints,0.0f);
  waminternals.joint_offsets.resize(nJoints,0.0f);
  owd_msgs::PIDgains empty_gains;
  waminternals.gains.resize(nJoints,empty_gains);

  // Plugin structures
  Plugin::_arm_position.resize(nJoints);
  Plugin::_target_arm_position.resize(nJoints);
  Plugin::_pid_torque.resize(nJoints);
  Plugin::_dynamic_torque.resize(nJoints);
  Plugin::_trajectory_torque.resize(nJoints);
  Plugin::_arm_velocity.resize(nJoints);

#endif // FAKE7

  if (BH_model == 280) {
    Plugin::_hand_position.resize(4);
    Plugin::_target_hand_position.resize(4);
    Plugin::_strain.resize(3);
  }
  if (Tactile) {
    Plugin::_tactile_f1.resize(24);
    Plugin::_tactile_f2.resize(24);
    Plugin::_tactile_f3.resize(24);
    Plugin::_tactile_palm.resize(24);
  }
  if (ForceTorque) {
    Plugin::_ft_force.resize(3);
    Plugin::_ft_torque.resize(3);
    Plugin::_filtered_ft_force.resize(3);
    Plugin::_filtered_ft_torque.resize(3);
  }

  // create a dummy previous trajectory
  owd_msgs::TrajInfo ti;
  ti.id="(none)";
  ti.type="(none)";
#ifdef FAKE7
  ti.end_position.resize(7,0.0f);
#else
  ti.end_position.resize(nJoints,0.0f);
#endif
  ti.state=owd_msgs::TrajInfo::state_done;
  wamstate.prev_trajectory = ti;

  // Construct the base transforms, based on the dimensions
  // from the WAM manual.
  static double PI=3.141592654;
  tf::Quaternion HALFPI_ROLL, NEG_HALFPI_ROLL;
  HALFPI_ROLL.setEulerZYX(0,0,PI/2.0);
  NEG_HALFPI_ROLL.setEulerZYX(0,0,-PI/2.0);
  wam_tf_base[0] = tf::Transform::getIdentity();
  wam_tf_base[1] = tf::Transform(NEG_HALFPI_ROLL);
  wam_tf_base[2] = tf::Transform(HALFPI_ROLL);
  wam_tf_base[3] = tf::Transform(NEG_HALFPI_ROLL,tf::Vector3(0.045,0,0.55));
  wam_tf_base[4] = tf::Transform(HALFPI_ROLL,tf::Vector3(-0.045,0,0));
  wam_tf_base[5] = tf::Transform(NEG_HALFPI_ROLL,tf::Vector3(0,0,0.30));
  wam_tf_base[6] = tf::Transform(HALFPI_ROLL);

}

bool WamDriver::Init(const char *joint_cal_file)
{
  ROS_INFO("Wam driver initializing");
  ROS_DEBUG("  Gravity comp value = %1.1fg",gravity_comp_value);
  joint_calibration_file = strdup(joint_cal_file);
  
  Plugin::_lower_jlimit.resize(7);
  Plugin::_upper_jlimit.resize(7);
  double lj[7]= {-2.60, -1.96, -2.73, -0.86, -4.79, -1.56, -2.99};
  memcpy(&Plugin::_lower_jlimit[0],lj,7*sizeof(double));
  double uj[7]= { 2.60,  1.96,  2.73,  3.13,  1.30,  1.56,  2.99};
  memcpy(&Plugin::_upper_jlimit[0],uj,7*sizeof(double));

  // The arms on Herb and ARM-S Phase II have different limits for J1
  if ( modified_j1) {
    Plugin::_lower_jlimit[0]=0.52;
    Plugin::_upper_jlimit[0]=5.76;
  }
    
  char speedstr[200], accelstr[200];
  strcpy(speedstr,""); strcpy(accelstr,"");
  for (unsigned int j = 0; j < nJoints; ++j) {
    snprintf(speedstr+strlen(speedstr),199-strlen(speedstr),"%3.0f ",Plugin::_joint_vel[j] * 180.0/3.141592654);
    snprintf(accelstr+strlen(accelstr),199-strlen(accelstr),"%3.0f ",Plugin::_joint_accel[j]*180.0/3.141592654);
  }
  
  ROS_DEBUG("Max motion limits: joint speed (deg/s) = [ %s]",speedstr);
  ROS_DEBUG("                 joint accel (deg/s/s) = [ %s]",accelstr);
  
  if (bus->open() == OW_FAILURE) {
    ROS_FATAL("Unable to open CANbus device");
    return false;
  }
  
  bool powerup = false;
  if (bus->check() == OW_FAILURE) {
    // Get the WAM into Idle mode
    ROS_FATAL("  Unable to communicate with the WAM.  Please do the following:");
    ROS_FATAL("    1. Turn on the WAM");
    ROS_FATAL("    2. Move it to its home position");
    ROS_FATAL("    3. Release all e-stops and press Shift-Idle on the pendant");
    powerup=true;
    do {
      usleep(100000); // wait for the user, and try again
      bus->clear();
    } while ((bus->check() == OW_FAILURE) && (ros::ok()));
  }

  if (!ros::ok()) {
    return false;
  }

  ros::NodeHandle n("~");

  std::string wamhome_list, plugin_list, gravity_vector;
  // default value is the one used for Herb
  n.param("home_position",
	  wamhome_list,
	  std::string ("3.14, -1.97, 0, -0.83, 1.309, 0, 3.02"));
  // if true will only report the top 10 values in each array
  n.param("tactile_top10",bus->tactile_top10,false);
  n.param("owd_plugins",plugin_list,std::string());
  n.param("log_controller_data",log_controller_data,false);
  n.param("gravity_vector",gravity_vector,std::string("-1,0,0"));
  n.param("ignore_breakaway_encoders",bus->ignore_breakaway_encoders,false);

#ifndef OWDSIM
  if (BH_model == 280) {
    if (bus->hand_reset() != OW_SUCCESS) {
      return false;
    }
  }
#endif // OWDSIM

#ifndef BH280_ONLY
  int32_t WamWasZeroed = 0;
  if (bus->get_property_rt(SAFETY_MODULE, ZERO, &WamWasZeroed,10000) == OW_FAILURE) {
    ROS_FATAL("Unable to query safety puck");
    return false;
  }
#else 
  int32_t WamWasZeroed=1;
#endif // BH280_ONLY
 
  bool FlippedHand;
  n.param("flipped_hand", FlippedHand, false); 

  owam = new WAM(bus, BH_model, ForceTorque, FlippedHand, Tactile, log_controller_data);
  n.param("slip_joints_on_high_torque",owam->slip_joints_on_high_torque,false);
  n.param("controller_blend_period",owam->jscontroller_blend_period,1.0);

  // Read the transmission ratios from ROS parameters
  get_transmission_ratios();

  // Set our transmission ratio values on the parameter server so that
  // others know what we're using in case we are using the compiled-in
  // defaults
  set_transmission_ratios();

  if (owam->init() == OW_FAILURE) {
    ROS_FATAL("Failed to initialize WAM instance");
    return false;
  }
  
  // initialize the Plugin pointers
  Plugin::wam = owam;
  Plugin::wamdriver = this;

  std::stringstream ss(wamhome_list);
  std::string item;
  unsigned int j=0;
  while (std::getline(ss,item,',')) {
    wamhome[++j]=strtod(item.c_str(),NULL);
  }
  if (j < nJoints) {
    ROS_FATAL("Could not extract %d joint values from ROS parameter \"home_position\"",nJoints);
    ROS_FATAL("home_position value is \"%s\"",wamhome_list.c_str());
    throw -1;
  }
  if (j > nJoints) {
    ROS_WARN("Warning: OWD configured for only %d joints; extra values in parameter \"home_position\" ignored",nJoints);
  }
  
  std::vector<double> elements;
  std::stringstream ss2(gravity_vector);
  while (std::getline(ss2,item,',')) {
    // The "up" direction that we maintain is the opposite of the
    // gravity direction that the user specifies
    elements.push_back(- strtod(item.c_str(),NULL));
  }
  if (elements.size() != 3) {
    ROS_ERROR("Could not parse value of ROS parameter \"gravity_vector\" into three components; ignoring");
  } else {
    Dynamics::up = R3(elements[0],elements[1],elements[2]);
    if (Dynamics::up.norm() != 1.0) {
      Dynamics::up.normalize();
    }
  }

  bool synchronize;
  n.param("synchronize_with_other_controllers",synchronize,false);
  if (synchronize) {
    int sync_id;
    n.param("synchronization_id",sync_id,-1);
    if (sync_id == -1) {
      ROS_FATAL("Must specify unique synchronization_id values for each synchronized controller.");
      return false;
    }
    int sync_count;
    n.param("synchronization_controller_count",sync_count,0);
    if (sync_count == 0) {
      ROS_FATAL("Must specify synchronization_controller_count");
      return false;
    }
    std::string sync_name("owd_");
    std::string tf_prefix;
    n.param("tf_prefix",tf_prefix,std::string("controller"));
    sync_name += tf_prefix;
    if (sync_id == 0) {
      ROS_INFO("Creating master synchronization controller");
      try {
	owam->ms = new MultiSyncMaster(sync_name, sync_count);
      } catch (const char *errmsg) {
	std::string e("Error while creating master synchronization controller:");
	e += errmsg;
	ROS_ERROR("%s",e.c_str());
	throw e.c_str();
      }
    } else {
      ROS_INFO("Attempting to contact master synchronization controller");
      try {
	owam->ms = new MultiSync(sync_name, sync_id);
      } catch (const char *errmsg) {
	std::string e("Error while contacting master synchronization controller:");
	e += errmsg;
	ROS_ERROR("%s",e.c_str());
	throw e.c_str();
      }
    }
    ROS_INFO("Synchronization established");
  }

  // Read our motor offsets from file (if found) and apply them
  // to the encoder values
  if (bus->simulation) {
    
    if (owam->set_jpos(wamhome) == OW_FAILURE) {
      ROS_FATAL("Unable to define WAM home position");
      throw -1;
    }

    start_control_loop();
    Plugin::gravity=9.81; // turn on Gravity
    owam->jsdynamics() = true; // turn on feed-forward dynamics

  } else if (WamWasZeroed) {
#ifndef BH280_ONLY
    ROS_DEBUG("Wam was already zeroed from a previous run; motor offsets unchanged.");
#endif // BH280_ONLY
    start_control_loop();
    Plugin::gravity=9.81; // turn on Gravity
    owam->jsdynamics() = true; // turn on feed-forward dynamics
    
  } else {
    if (!powerup) {
      // still need to tell the user what to do
      ROS_FATAL("\007WAM did not retain its encoder values\007.");
      ROS_FATAL("Please move the WAM to its home position.");
    }
    bool good_home;
    do {
      set_home_position();
      start_control_loop();
      Plugin::gravity=0.0; // turn off Gravity
      owam->jsdynamics() = true; // turn on feed-forward dynamics
      
      ROS_INFO("Robot will make small movements to verify home position, and");
      ROS_INFO("then will turn on gravity compensation.");
      //      good_home = verify_home_position();
      good_home=true;
      if (!good_home) {
        // wait until Shift-Idle is pressed
        while (bus->get_puck_state()== 2) {
          usleep(50000);
        }
        owam->jsdynamics()=false;
        stop_control_loop();
      }
    } while (!good_home);
    Plugin::gravity=9.81; // turn on Gravity
  }

  bool hold_starting_position;
  n.param("hold_starting_position",hold_starting_position,false);
  if (hold_starting_position) {
    owam->hold_position();
  }
  
  owam->exit_on_pendant_press=true; // from now on, exit if the user hits e-stop or idle
  
  load_plugins(plugin_list);

  return true;
}

  void WamDriver::get_transmission_ratios() {
    static ros::NodeHandle n("~");
    n.param("motor1_transmission_ratio",owam->mN1, owam->mN1);
    n.param("motor2_transmission_ratio",owam->mN2, owam->mN2);
    n.param("motor3_transmission_ratio",owam->mN3, owam->mN3);
    n.param("motor4_transmission_ratio",owam->mN4, owam->mN4);
    n.param("motor5_transmission_ratio",owam->mN5, owam->mN5);
    n.param("motor6_transmission_ratio",owam->mN6, owam->mN6);
    n.param("motor7_transmission_ratio",owam->mN7, owam->mN7);
    n.param("differential3_ratio",      owam->mn3, owam->mn3);
    n.param("differential6_ratio",      owam->mn6, owam->mn6);
    ROS_INFO("motor1_transmission_ratio=%f",owam->mN1);
    ROS_INFO("motor2_transmission_ratio=%f",owam->mN2);
    ROS_INFO("motor3_transmission_ratio=%f",owam->mN3);
    ROS_INFO("motor4_transmission_ratio=%f",owam->mN4);
    ROS_INFO("motor5_transmission_ratio=%f",owam->mN5);
    ROS_INFO("motor6_transmission_ratio=%f",owam->mN6);
    ROS_INFO("motor7_transmission_ratio=%f",owam->mN7);
    ROS_INFO("differential3_ratio=%f",owam->mn3);
    ROS_INFO("differential6_ratio=%f",owam->mn6);
  }

  void WamDriver::set_transmission_ratios() {
    static ros::NodeHandle n("~");
    n.setParam("motor1_transmission_ratio",owam->mN1);
    n.setParam("motor2_transmission_ratio",owam->mN2);
    n.setParam("motor3_transmission_ratio",owam->mN3);
    n.setParam("motor4_transmission_ratio",owam->mN4);
    n.setParam("motor5_transmission_ratio",owam->mN5);
    n.setParam("motor6_transmission_ratio",owam->mN6);
    n.setParam("motor7_transmission_ratio",owam->mN7);
    n.setParam("differential3_ratio",      owam->mn3);
    n.setParam("differential6_ratio",      owam->mn6);
  }

void WamDriver::load_plugins(std::string plugin_list) {
  // make sure we lock out any calls to the Publish functions
  boost::mutex::scoped_lock lock(plugin_mutex);

  // try to load any specified plugins
  std::stringstream pp(plugin_list);
  std::string pname;
  while (std::getline(pp,pname,','))
  {
    // open the plugin from disk
    void *plib;
    try {
      plib = dlopen(pname.c_str(), RTLD_NOW);
    } catch (char const *error) {
      ROS_WARN("Plugin %s threw error message \"%s\"",
               pname.c_str(), error);
      continue;
    }
    if (!plib)
    {
      ROS_WARN("Could not load plugin %s: %s",
               pname.c_str(), dlerror());
      continue;
    }
    // functions to find
    bool (*pl_register)();
    bool (*pl_register_path)(const char *);
    bool (*pl_unregister)();
    // the recommendation of casting our function pointer
    // to a void * comes from the dlsym man page
    *(void **)(&pl_register_path) = dlsym(plib,"_Z19register_owd_pluginPKc");
    char * perr1 = pl_register_path ? 0 : strdup(dlerror());
    *(void **)(&pl_register) = dlsym(plib,"_Z19register_owd_pluginv");
    char * perr2 = pl_register ? 0 : strdup(dlerror());
    if (pl_register_path && pl_register) {
      ROS_WARN("Plugin %s provides both register_owd_plugin() and register_owd_plugin(path)!",
               pname.c_str());
      dlclose(plib);
      free(perr1);
      free(perr2);
      continue;
    }
    if (!(pl_register_path || pl_register)) {
      ROS_WARN("Could not find register_owd_plugin function in library %s: %s (no path) %s (with path)",
               pname.c_str(),
               perr1 ? perr1 : "(none)",
               perr2 ? perr2 : "(none)");
      dlclose(plib);
      free(perr1);
      free(perr2);
      continue;
    }
    free(perr1);
    free(perr2);
    // find unregister function           
    *(void **)(&pl_unregister) = dlsym(plib,"_Z21unregister_owd_pluginv");
    if (!pl_unregister) {
      ROS_WARN("Could not find unregister_owd_plugin function in library %s: %s",
               pname.c_str(), dlerror());
      dlclose(plib);
      continue;
    }
    // call the correct register function
    try {
      bool success;
      if (pl_register_path)
        success = (*pl_register_path)(pname.c_str());
      else
        success = (*pl_register)();
      if (!success)
      {
        ROS_WARN("Could not call register_owd_plugin function in plugin %s",
                 pname.c_str());
        dlclose(plib);
        continue;
      }
    } catch (char const* error) {
      ROS_ERROR("Plugin %s threw error string \"%s\"",
                pname.c_str(),error);
      dlclose(plib);
      continue;
    }
    ROS_INFO("Loaded and registered plugin %s", pname.c_str());
    PluginPointers thisplugin(plib,pl_unregister);
    loaded_plugins.push_back(thisplugin);
  }
}

void WamDriver::unload_plugins() {
  // shut down and unload all of the plugins

  // make sure we lock out any calls to the Publish functions
  boost::mutex::scoped_lock lock(plugin_mutex);

  for (unsigned int i=0; i<loaded_plugins.size(); ++i) {
    void *plib = loaded_plugins[i].first;
    bool (*pl_unregister)() = loaded_plugins[i].second;
    // call the plugin's unregister_owd_plugin() function
    (*pl_unregister)();
    // unload the plugin
    dlerror(); // clear any previous error
    if (dlclose(plib)) {
      ROS_ERROR("Error while unloading user plugin");
      char *perr = dlerror();
      if (perr) {
	ROS_ERROR("Last system error: %s",perr);
      }
    }
    ROS_INFO("Unloaded plugin at %p",plib);
  }
  loaded_plugins.clear();
}

void WamDriver::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_wamstate = 
    n.advertise<owd_msgs::WAMState>("wamstate", 1);
  pub_waminternals = 
    n.advertise<owd_msgs::WAMInternals>("waminternals", 1);
  ss_AddTrajectory = 
    n.advertiseService("AddTrajectory",&WamDriver::AddTrajectory,this);
  ss_AddTimedTrajectory = 
    n.advertiseService("AddTimedTrajectory",&WamDriver::AddTimedTrajectory,this);
  ss_GetStiffness =
    n.advertiseService("GetStiffness",&WamDriver::GetStiffness,this);
  ss_SetStiffness =
    n.advertiseService("SetStiffness",&WamDriver::SetStiffness,this);
  ss_SetJointStiffness =
    n.advertiseService("SetJointStiffness",&WamDriver::SetJointStiffness,this);
  ss_SetJointOffsets =
    n.advertiseService("SetJointOffsets",&WamDriver::SetJointOffsets,this);
  ss_DeleteTrajectory = 
    n.advertiseService("DeleteTrajectory",&WamDriver::DeleteTrajectory,this);
  ss_CancelAllTrajectories = 
    n.advertiseService("CancelAllTrajectories",&WamDriver::CancelAllTrajectories,this);
  ss_PauseTrajectory = 
    n.advertiseService("PauseTrajectory",&WamDriver::PauseTrajectory,this);
  ss_ReplaceTrajectory =
    n.advertiseService("ReplaceTrajectory",&WamDriver::ReplaceTrajectory,this);
  ss_SetSpeed =
    n.advertiseService("SetSpeed",&WamDriver::SetSpeed,this);
  ss_GetSpeed =
    n.advertiseService("GetSpeed",&WamDriver::GetSpeed,this);
  ss_SetExtraMass = 
    n.advertiseService("SetExtraMass",&WamDriver::SetExtraMass,this);
  ss_SetStallSensitivity = 
    n.advertiseService("SetStallSensitivity",&WamDriver::SetStallSensitivity,this);
  ss_GetStallSensitivity = 
    n.advertiseService("GetStallSensitivity",&WamDriver::GetStallSensitivity,this);
  ss_GetArmDOF =
    n.advertiseService("GetArmDOF",&WamDriver::GetDOF,this);
  ss_CalibrateJoints =
    n.advertiseService("CalibrateJoints", &WamDriver::CalibrateJoints, this);
  sub_wamservo =
    n.subscribe("wamservo", 1, &WamDriver::wamservo_callback,this);
  sub_MassProperties =
    n.subscribe("wam_mass", 1, &WamDriver::MassProperties_callback,this);
  ss_StepJoint =
    n.advertiseService("StepJoint", &WamDriver::StepJoint, this);
  ss_SetGains =
    n.advertiseService("SetGains", &WamDriver::SetGains, this);
  ss_ReloadPlugins = 
    n.advertiseService("ReloadPlugins", &WamDriver::ReloadPlugins, this);
  ss_SetForceInputThreshold =
    n.advertiseService("SetForceInputThreshold", &WamDriver::SetForceInputThreshold, this);
  ss_SetTactileInputThreshold =
    n.advertiseService("SetTactileInputThreshold", &WamDriver::SetTactileInputThreshold, this);
  ss_SetTorqueLimits =
    n.advertiseService("SetTorqueLimits", &WamDriver::SetTorqueLimits, this);
  ss_SetController =
    n.advertiseService("SetController", &WamDriver::SetController, this);

#ifdef BUILD_FOR_SEA
  sub_wam_joint_targets = 
    n.subscribe("wam_joint_targets", 10, &WamDriver::wamjointtargets_callback,this);
 
  sub_wam_seactrl_settl =
    n.subscribe("wam_seactrl_settl", 10, &WamDriver::wam_seactrl_settl_callback, this);
  pub_wam_seactrl_curtl = 
    n.advertise<owd_msgs::IndexedJointValues>("wam_seactrl_curtl", 1);
  ss_WamRequestSeaCtrlTorqLimit =
    n.advertiseService("WamRequestSeaCtrlTorqLimit",&WamDriver::WamRequestSeaCtrlTorqLimit,this);

  sub_wam_seactrl_setkp =
    n.subscribe("wam_seactrl_setkp", 10, &WamDriver::wam_seactrl_setkp_callback, this); 
  pub_wam_seactrl_curkp =
    n.advertise<owd_msgs::IndexedJointValues>("wam_seactrl_curkp", 1);
  ss_WamRequestSeaCtrlKp =
    n.advertiseService("WamRequestSeaCtrlKp",&WamDriver::WamRequestSeaCtrlKp,this);

  sub_wam_seactrl_setkd =
    n.subscribe("wam_seactrl_setkd", 10, &WamDriver::wam_seactrl_setkd_callback, this); 
  pub_wam_seactrl_curkd =
    n.advertise<owd_msgs::IndexedJointValues>("wam_seactrl_curkd", 1);
  ss_WamRequestSeaCtrlKd =
    n.advertiseService("WamRequestSeaCtrlKd",&WamDriver::WamRequestSeaCtrlKd,this);

  sub_wam_seactrl_setki =
    n.subscribe("wam_seactrl_setki", 10, &WamDriver::wam_seactrl_setki_callback, this); 
  pub_wam_seactrl_curki =
    n.advertise<owd_msgs::IndexedJointValues>("wam_seactrl_curki", 1);
  ss_WamRequestSeaCtrlKi =
    n.advertiseService("WamRequestSeaCtrlKi",&WamDriver::WamRequestSeaCtrlKi,this);
#endif // BUILD_FOR_SEA

}


WamDriver::~WamDriver() {
  ROS_FATAL("Destroying class WamDriver");
  if (owam) {
    owam->jsdynamics() = false;
  }
  
#ifdef AUTO_SHUTDOWN // (not quite working)
  for (int p=1; p<8; p++) {
    if (bus->set_property(p,MODE,0,false) == OW_FAILURE) {
      ROS_FATAL("Error changing mode of puck %d",p);
      throw -1;
    }
  }
  if (bus->set_property(10,MODE,0,false) == OW_FAILURE) {
    ROS_FATAL("Error changing mode of puck 10");
    throw -1;
  }
#endif  // AUTO_SHUTDOWN

  ROS_FATAL("~WamDriver: shutting down plugins");

  unload_plugins();

  if (owam) {
    // owam isn't created until the ::Init function, so we need to check
    // that we have one before trying to delete it.
    ROS_FATAL("~WamDriver: stopping class WAM control loop");
    owam->stop();
    ROS_FATAL("~WamDriver: deleting owam");
    delete owam;
  }
  // make sure the bus is still around before we delete it (depending on
  // how we started shutting down, another function may have deleted it
  // first)
  if (bus) {
    ROS_FATAL("~WamDriver: deleting bus");
    delete bus;
  }

  ROS_INFO("~WamDriver: WAM driver exiting normally");
  running=false;
}

OWD::Trajectory *WamDriver::BuildTrajectory(owd_msgs::JointTraj &jt) {

  // check to see if we should use a MacJointTraj based on whether
  // any of the corners have non-zero blend radii
  bool blended_traj = false;
  for (unsigned int i=1; i<jt.blend_radius.size()-1; ++i) {
    if (jt.blend_radius[i] > 0.0) {
      blended_traj=true;
      break;
    }
  }
  
  TrajType traj;
  try {
    traj=Plugin::ros2owd_traj(jt);
  } catch (const char *error) {
    snprintf(last_trajectory_error,200,"Could not extract valid trajectory: %s",error);
    ROS_ERROR_NAMED("BuildTrajectory","%s",last_trajectory_error);
    return NULL;
  }
  bool bWaitForStart=(jt.options & jt.opt_WaitForStart);
  bool bCancelOnStall=(jt.options & jt.opt_CancelOnStall);
  bool bCancelOnForceInput=(jt.options & jt.opt_CancelOnForceInput);
  bool bCancelOnTactileInput(jt.options & jt.opt_CancelOnTactileInput);
  ROS_DEBUG_NAMED("BuildTrajectory","Building trajectory with options WaitForStart=%d CancelOnStall=%d CancelOnForceInput=%d CancelOnTactileInput=%d",int(bWaitForStart), int(bCancelOnStall), int(bCancelOnForceInput), int(bCancelOnTactileInput));

  if (!blended_traj) {
    ROS_WARN_NAMED("BuildTrajectory","No blends found; using ParaJointTraj");

    try {
      ParaJointTraj *paratraj = new ParaJointTraj(traj,Plugin::_joint_vel,Plugin::_joint_accel,bWaitForStart,bCancelOnStall,bCancelOnForceInput,bCancelOnTactileInput);
      ROS_DEBUG_NAMED("BuildTrajectory","parabolic trajectory built");
      ROS_DEBUG_NAMED("BuildTrajectory","Segments=%zd, total time=%3.3f",paratraj->parsegs[0].size(),paratraj->parsegs[0].back().end_time);
      
      if (jt.options & jt.opt_Synchronize) {
	paratraj->Synchronize=true;
      }
      if (jt.id == "") {
	paratraj->id = Trajectory::random_id();
      } else {
	paratraj->id = jt.id;
      }
      return paratraj;
    } catch (const char * error) {
      snprintf(last_trajectory_error,200,"Error building parabolic traj: %s",error);
      ROS_ERROR_NAMED("BuildTrajectory","%s",last_trajectory_error);
      return NULL;
    }
  } else {
    // use a MacJointTraj (blends at points)
    try {
      MacJointTraj *mactraj = new MacJointTraj(traj,
					       Plugin::_joint_vel,
					       Plugin::_joint_accel,
					       Plugin::max_jerk,
					       bWaitForStart,
					       bCancelOnStall,
					       bCancelOnForceInput,
					       bCancelOnTactileInput);
      if (jt.options & jt.opt_Synchronize) {
	mactraj->Synchronize=true;
      }
      if (jt.id == "") {
	mactraj->id = Trajectory::random_id();
      } else {
	mactraj->id = jt.id;
      }
      
      ROS_DEBUG_NAMED("BuildTrajectory","MacFarlane blended trajectory %s built", mactraj->id.c_str());
      return mactraj;
    } catch (const char *error) {
      snprintf(last_trajectory_error,200,"Error building blended traj: %s",error);
      ROS_ERROR_NAMED("BuildTrajectory","%s",last_trajectory_error);
      for (unsigned int tt=0; tt<traj.size(); ++tt) {
	ROS_ERROR_NAMED("BuildTrajectory","  Traj point %d: %s",tt,
			traj[tt].sdump());
      }
      // this error is often happening when trying to build a blended traj
      // from just half of a 2-arm traj when one arm doesn't move much.  we
      // can usually still succeed with a non-blended traj
      ROS_ERROR_NAMED("BuildTrajectory","Trying to use a ParaJointTraj instead");
      try {
	ParaJointTraj *paratraj = new ParaJointTraj(traj,Plugin::_joint_vel,Plugin::_joint_accel,bWaitForStart,bCancelOnStall,bCancelOnForceInput,bCancelOnTactileInput);
	ROS_DEBUG_NAMED("BuildTrajectory","parabolic trajectory built");
	ROS_DEBUG_NAMED("BuildTrajectory","Segments=%zd, total time=%3.3f",paratraj->parsegs[0].size(),paratraj->parsegs[0].back().end_time);
	if (jt.options & jt.opt_Synchronize) {
	  paratraj->Synchronize=true;
	}
	if (jt.id == "") {
	  paratraj->id = Trajectory::random_id();
	} else {
	  paratraj->id = jt.id;
	}
	return paratraj;
      } catch (const char * error) {
	snprintf(last_trajectory_error,200,"Error building parabolic traj: %s",error);
	ROS_ERROR_NAMED("BuildTrajectory","%s",last_trajectory_error);
	return NULL;
      }
    }
  }
}

void WamDriver::save_joint_offset(double jointval, double *offset) {

    // get the nearest parallel/square angle
    double newval = get_nearest_joint_value(jointval,40.0);
    ROS_WARN("\nSet joint to %3f degrees (y/n)?",
           newval*180/3.141592654);
    while (char c = getchar()) {
        if (c == 'y') {
            *offset = jointval - newval;
            return;
        } else if (c == 'n') {
            return;
        } else {
            usleep(10000);
        }
    }
    return;
}

int WamDriver::get_joint_num() {
    while (char c = getchar()) {
        if ((c >= '0') && (c <= '9')) {
            return (int)(c - '0');
        } else if ((c == 'q') || (c == 3) || (c == 27)) {
            // allow abort with q, ctrl-c, or ESC
            return -1;
        } else {
            usleep(10000);
        }
    }
    return 0;
}

double WamDriver::get_nearest_joint_value(double jointval, double tolerance) {
    double jointval_deg = jointval * 180.0 / 3.141592654;
    // check for a variety of multiples of 90-deg, from
    // -270 to 270, to infer the position that the user
    // intends to set
    for (int a = -3; a < 4; a++) {
        double newval = (double)a * 90.0;
        if (abs((int)(newval - jointval_deg))  < tolerance) {
            return newval * 3.141592654/180.0;
        }
    }
    // no match found, so return unchanged value
    return jointval;
}

void WamDriver::apply_joint_offsets(double *joint_offsets) {
    ROS_INFO("New offsets recorded for the following joints:");
    for (unsigned int j = 1; j <= nJoints; ++j) {
        if (joint_offsets[j] != 0.0f) {
            ROS_INFO("  %d = %1.8f radians (%3.6f degrees)",j,joint_offsets[j],joint_offsets[j]*180.0/3.141592654);
        }
    }

    ROS_ERROR("\n  Return the WAM to the initial position.");
    ROS_ERROR("  Press RETURN when you are ready to disable the motors.");
    ROS_ERROR("  WARNING: the arm will lose all power when you hit RETURN,");
    ROS_ERROR("  so be sure it is well-supported or it will drop quickly.");
    char *line=NULL;
    size_t linelen = 0;
    linelen = getline(&line,&linelen,stdin);
    free(line);
    owam->jsdynamics() = false;
    ROS_ERROR("  Now verify that the WAM is still in the intended home position.");
    ROS_ERROR("  Press shift-idle on the pendant when ready to save the calibration values.");
    owam->exit_on_pendant_press=false;

    // wait for shift-idle to be pressed
    while (bus->get_puck_state()== 2) {
        usleep(50000);
    }
    
    // turn off gravity and idle the controller
    owam->jsdynamics()=false;
    // wait 3sec for the arm to physically stabilize
    sleep(3);
    // get the current position
    double jointpos[nJoints+1];
    owam->get_current_data(jointpos,NULL,NULL);

    stop_control_loop();

    // tweak the joint angles to apply the offsets
    double new_joint_angles[Joint::Jn+1];
    // subtract each offset from the current value
    // that will force the WAM to have to move that joint
    // physically further in the offset direction to reach the
    // same value.  thus if someone provides a positive offset,
    // the arm will be shifted in the positive direction.
    for (unsigned int j = 1; j <= nJoints; ++j) {
        new_joint_angles[j]=jointpos[j]-joint_offsets[j];
    }

    unsigned int retries=4;
    int result = owam->set_jpos(new_joint_angles);
    while (result == OW_FAILURE) {
      if (retries-- > 0) {
        ROS_WARN("Failed to set new joint angles; trying again");
	result = owam->set_jpos(new_joint_angles);
      } else {
        ROS_ERROR("Unable to set new joint angles.");
        throw -1;
      }
    }

    // record the AP to MECH offsets for each motor
    FILE *moff_out = fopen(joint_calibration_file,"w");
    if (moff_out) {
        fprintf(moff_out,"WAM encoder offset values\n");
        for (unsigned int puck_id = 1; puck_id <= nJoints; puck_id++) {
            int32_t mech=0;
            int32_t moff = get_puck_offset(puck_id,&mech);
            fprintf(moff_out,"%d = %d %d\n",puck_id,moff,mech);
        }
        fclose(moff_out);
        ROS_DEBUG("New offsets written to \"%s\"",joint_calibration_file);
    } else {
        ROS_ERROR("Error writting to offset file \"%s\"",joint_calibration_file);
    }

    start_control_loop();
    owam->jsdynamics()=true;

    owam->exit_on_pendant_press=true; // from now on, exit if the user hits e-stop or idle
    ROS_DEBUG("New offsets recorded; server ready.");
}        

int WamDriver::get_puck_offset(int puckid, int32_t *mechout, int32_t *apout) {
    int32_t p,mech;
    int retry=6;
    while (bus->get_property_rt(bus->pucks[puckid].id(), AP, &p, 10000,4) == OW_FAILURE) {
      if (retry-- > 0) {
	ROS_WARN("Failed to get AP from puck %d, retrying",puckid);
      } else {
	ROS_FATAL("Unable to get AP from puck %d; cannot proceed", puckid);
	throw -1;
      }
    }

    if (bus->fw_vers < 119) {
      // prior to puck firmware version 119 we have to get MECH by
      // setting an address and then querying the VALUE property.
      retry=6;
      while (bus->set_property_rt(bus->pucks[puckid].id(), ADDR, 0x84D7, false) == OW_FAILURE) {
	if (retry-- > 0) {
	  ROS_WARN("Failed to set address of MECH on puck %d, retrying",puckid);
	} else {
	  ROS_FATAL("Failed to set address of MECH on puck %d; cannot proceed",puckid);
	  throw -1;
	}
      }
      
      retry=6;
      while (bus->get_property_rt(bus->pucks[puckid].id(), VALUE, &mech, 10000,4) == OW_FAILURE) {
	if (retry-- > 0) {
	  ROS_WARN("Failed to get MECH from puck %d, retrying",puckid);
	} else {
	  ROS_FATAL("Failed to get MECH from puck %d; cannot proceed",puckid);
	  throw -1;
	}
      }
    } else {
      // starting with puck firmware version 119 we can just ask for
      // MECH by name
      retry=6;
      while (bus->get_property_rt(bus->pucks[puckid].id(), MECH, &mech, 10000,4) == OW_FAILURE) {
	if (retry-- > 0) {
	  ROS_WARN("Failed to get MECH from puck %d, retrying",puckid);
	} else {
	  ROS_FATAL("Failed to get MECH from puck %d; cannot proceed",puckid);
	  throw -1;
	}
      }
    }
    ROS_DEBUG("Puck %d  AP=%d  OFFSET=%d  MECH=%d",puckid,p,p-mech,mech);
    if (mechout) {
        // return the value of MECH, too
        *mechout = mech;
    }
    if (apout) {
        // return the value of AP, too
        *apout = p;
    }
    return p-mech;
}

void WamDriver::calibrate_joint_angles() {
/*
 * Calibration procedure:
 *   0. At initialization, driver reads AP-MECH offsets from file
 *   1. Idle the controller, leave gravity comp enabled.
 *   2. Move joints to known positions, and hit a key to save each joint
 *      error
 * OR 2. Use laser pointer and record a bunch of matching joint angles,
 *      then solve for joint error
 *   3. Move robot to home position, idle robot, and correct joint angles
 *      based on errors from #2
 *   4. Record AP-MECH offsets for each joint and save to file for next time
 *   5. Switch back to position mode if controller was idled (#1)
 */

  if (bus->simulation) {
    return;
  }

  // Set up stdin to work char-by-char instead of line-by-line
  struct termios previous_termattr;
  struct termios new_termattr;
  tcgetattr(fileno(stdin), &previous_termattr);
  tcgetattr(fileno(stdin), &new_termattr);
  new_termattr.c_lflag &= ~ICANON;
  new_termattr.c_cc[VMIN] =0;
  new_termattr.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &new_termattr);
  
  // cancel any pending trajectories and ignore future ones
  discard_movements=true;
  wamstate.state = owd_msgs::WAMState::state_inactive;
  if (owam->jointstraj) {
    owam->cancel_trajectory();
  }

  // put in hold mode, but suppress each of the controllers
  owam->check_safety_torques=false; // hold positions stiffly
  owam->hold_position();
  for (unsigned int j=0; j<nJoints; ++j) {
    owam->jscontroller->stop(j);
  }

  ROS_ERROR("Robot is in calibration mode.  For each joint,");
  ROS_ERROR("move it to be parallel or square to the previous");
  ROS_ERROR("joint, then press key 1-7 corresponding to the joint");
  ROS_ERROR("number.");
  ROS_ERROR("'h' will hold a joint angle, 'u' will unhold it.");
  ROS_ERROR("Hit 'd' when done, or 'q' to quit.");
  double joint_offsets[nJoints+1];
  for (unsigned int j = 1; j <=nJoints; ++j) {
    joint_offsets[j] = 0.0f;
  }
  bool done = false;
  bool save = true;
  char cmd;
  unsigned int jnum;
  double jointpos[nJoints+1];
  while (!done) {
    owam->get_current_data(jointpos,NULL,NULL); // update the joint positions
    if (read(fileno(stdin),&cmd,1)) {
      switch(cmd) {
      case 'd' :
      case 'D':
	done = true;
      break;
      case '1' :
	save_joint_offset(jointpos[1],joint_offsets + 1);
	break;
      case '2' :
	save_joint_offset(jointpos[2],joint_offsets + 2);
	break;
      case '3' :
	save_joint_offset(jointpos[3],joint_offsets + 3);
	break;
      case '4' :
	save_joint_offset(jointpos[4],joint_offsets + 4);
	break;
      case '5' :
	if (nJoints>4) {
	  save_joint_offset(jointpos[5],joint_offsets + 5);
	}
	break;
      case '6' :
	if (nJoints>4) {
	  save_joint_offset(jointpos[6],joint_offsets + 6);
	}
	break;
      case '7' :
	if (nJoints>4) {
	  save_joint_offset(jointpos[7],joint_offsets + 7);
	}
	break;
      case 'h' :
	// hold a joint angle
	ROS_ERROR("\nHold joint number 1-%d: holdpos: %d\n",nJoints, owam->holdpos);
	jnum = get_joint_num();
	if ((jnum > 0) && (jnum <= nJoints)) {

	  // get current held positions
#ifdef BUILD_FOR_SEA
	  owam->posSmoother.getSmoothedPVA(owam->heldPositions);
#endif // BUILD_FOR_SEA

	  // if it's within 2 deg of parallel or square,
	  // set it to parallel or square
	  double holdval = get_nearest_joint_value(jointpos[jnum] - joint_offsets[jnum],2);
	  owam->heldPositions[jnum] = holdval+joint_offsets[jnum];
	  owam->jscontroller->reset(jnum-1);
#ifdef BUILD_FOR_SEA
	  owam->heldPositions[jnum] = holdval+joint_offsets[jnum];
	  owam->posSmoother.reset(owam->heldPositions, Joint::Jn+1);   
#endif // BUILD_FOR_SEA
	  owam->jscontroller->run(jnum-1);
	}
	break;
      case 'u' :
	// unhold a joint angle
	ROS_ERROR("\nUnhold joint number 1-%d: holdpos: %d\n",nJoints, owam->holdpos);
	jnum = get_joint_num();
	if ((jnum > 0) && (jnum <= nJoints)) {
	  owam->jscontroller->stop(jnum-1);
	}
	break;
      case 'q' :
      case 'Q' :
	save = false;
      done = true;
      }
    }
    for (unsigned int j=1; j <= nJoints; j++) {
      printf("%d=% 3.2f  ",j,(jointpos[j] - joint_offsets[j]) * 180.0 / 3.141592654);
    }
    printf("    \r");
    usleep(10000);
  }
  // restore terminal settings
  tcsetattr(fileno(stdin), TCSAFLUSH, &previous_termattr);

  owam->release_position();
  owam->check_safety_torques = true;
  for (unsigned int j=1; j<=nJoints; ++j) {
    owam->jscontroller->run(j-1);
  }

  if (save) {
    apply_joint_offsets(joint_offsets);
  }

  // start accepting trajectories again
  discard_movements=false;
  wamstate.state = owd_msgs::WAMState::state_free;
}


void WamDriver::calibrate_wam_mass_model() {
#ifdef USE_OPENWAM
    ROS_ERROR("Robot is in mass calibration mode.  Select a link\n");
    ROS_ERROR("using keys 1-7, then press + or - to increase or\n");
    ROS_ERROR("decrease the link mass, or x/X, y/Y, z/Z to decrease/\n");
    ROS_ERROR("increase the COG distance.\n");
    ROS_ERROR("Hit 'd' when done, or 'q' to quit.\n");
    char cmd;
    int active_link = 1;
    bool done=false;
    double pulsemag = 4.0;
    int pulsedur = 200;
    bool pulsepair = true;
    static double friction_pulse_lowpos[8]= {0.0,-2.1,-1.5,-2.3, -.4,-4.0,-1.1,-2.5};
    static double friction_pulse_highpos[8]={0.0, 2.1, 1.5, 2.0, 2.6, 0.8, 1.1, 2.5};

    TrajPoint tp(nJoints);
    TrajType vtraj;
    while (!done) {
        if (read(fileno(stdin),&cmd,1)) {
            // get the current position
          //            for (int j1=1; j1<8; j1++) {
          //                tp[j1-1]=ScreenBuf::joint_pos[j1];
          //            }
            if ((cmd >= '1') && (cmd <= '0'+nJoints)) {
                active_link = cmd - '0';
                dyn_active_link = active_link;
            } else {
                switch(cmd) {
                case 'q' :
                    ROS_ERROR("Exiting calibration mode.\n");
                    return;
                case '+':
                    owam->links[active_link].mass *= 1.05;
                    break;
                case '-':
                    owam->links[active_link].mass /= 1.05;
                    break;
                case 'x':
                    owam->links[active_link].cog.x[0] /= 1.05;
                    break;
                case 'X':
                    owam->links[active_link].cog.x[0] *= 1.05;
                    break;
                case 'y':
                    owam->links[active_link].cog.x[1] /= 1.05;
                    break;
                case 'Y':
                    owam->links[active_link].cog.x[1] *= 1.05;
                    break;
                case 'z':
                    owam->links[active_link].cog.x[2] /= 1.05;
                    break;
                case 'Z':
                    owam->links[active_link].cog.x[2] *= 1.05;
                    break;
                case 'D':
                  //                    if (ScreenBuf::mode == ScreenBuf::ACCEL_PULSE) {
                        pulsedur += 100;
                        //                    } else if (ScreenBuf::mode == ScreenBuf::FRICTION_CALIB) {
                        fv[active_link]+=0.1;
                        //                    }
                    break;
                case 'd':
                  //                    if (ScreenBuf::mode == ScreenBuf::ACCEL_PULSE) {
                        pulsedur -= 100;
                        //                    } else if (ScreenBuf::mode == ScreenBuf::FRICTION_CALIB) {
                        fv[active_link]-=0.1;
                        //                    }
                    break;
                case 'M':
                    pulsemag += 1.0;
                    break;
                case 'm':
                    pulsemag -= 1.0;
                    break;
                case '/':
                    pulsepair = !pulsepair;
                    break;
                case 'H':
                    owam->hold_position();
                    break;
                case 'h':
                    owam->release_position();
                    break;
                case '\n':
#ifdef PULSE
                  if (ScreenBuf::mode == ScreenBuf::ACCEL_PULSE) {
                        //apply an acceleration pulse at the current joint
                        owam->lock("OWD: calib mass model");
                        //delete previous pulse if there is one
                        if(owam->pulsetraj != NULL)
                            delete owam->pulsetraj;
                        if (pulsepair) {
                            owam->pulsetraj = new PulseTraj(pulsemag,pulsedur,pulsedur/2,active_link);
                        } else {
                            owam->pulsetraj = new PulseTraj(pulsemag,pulsedur,0,active_link);
                        }
                        owam->unlock("OWD: calib mass model");
                } else if (ScreenBuf::mode == ScreenBuf::FRICTION_CALIB) {
                        // run a velocity traj on the current joint
		    Plugin::_joint_accel[active_link-1]=Plugin::_joint_vel[active_link-1]=ScreenBuf::friction_pulse_velocity;
                        vtraj.clear();
                        // the first point in the trajectory will have most joint values the
                        // same as the current position, but the active joint set to the low
                        // pulse position
                        tp[active_link-1]=friction_pulse_lowpos[active_link];
                        vtraj.push_back(TrajPoint(tp));
                        // the second trajectory point will have the active joint set to the
                        // high pulse position
                        tp[active_link-1]=friction_pulse_highpos[active_link];
                        vtraj.push_back(TrajPoint(tp));
                        //                        ExecuteTrajectory(vtraj,false,false,false,false,0);
                        // restore the joint velocities to max
			Plugin::_joint_vel[active_link-1]=Plugin::_max_joint_vel[active_link-1];
			Plugin::_joint_accel[active_link-1]=Plugin::_joint_vel[active_link-1]/min_accel_time;
                    } else if (ScreenBuf::mode == ScreenBuf::TRAJ_RECORD) {
#endif // PULSE
                        // add current position to point list
                        vtraj.push_back(TrajPoint(tp));
#ifdef PULSE
                    }
#endif // PULSE
                    break;
                case 'w':
                  //                    if (ScreenBuf::mode == ScreenBuf::ACCEL_PULSE) {
                        //write pulse trajectory positions to file
                        write_pulse_data();
#ifdef DIAG
                        //                    } else if (ScreenBuf::mode == ScreenBuf::TRAJ_RECORD) {
                        // write list of points to file
                        char point_fname[100];
                        snprintf(point_fname,100,"collected_points_%d",point_filenum++);
                        log_rave_trajectory(point_fname,vtraj);
                        vtraj.clear();
                        ScreenBuf::savedpoints=0;
                        ROS_ERROR("Saved points written to %s",point_fname);
#endif // DIAG
                        //                    }
                    break;
                case 'S':
                    fs[active_link]+=0.1;
                    break;
                case 's':
                    fs[active_link]-=0.1;
                    break;
                case 'V':
                  //                    ScreenBuf::friction_pulse_velocity+= 0.05f;
                    break;
                case 'v':
                  //                    ScreenBuf::friction_pulse_velocity-= 0.05f;
                    break;
                case 'A':
                  //                    ScreenBuf::mode = ScreenBuf::ACCEL_PULSE;
                    break;
                case 'F':
                  //                    ScreenBuf::mode = ScreenBuf::FRICTION_CALIB;
                    break;
                case 'T':
                  //                    ScreenBuf::mode = ScreenBuf::TRAJ_RECORD;
                    vtraj.clear();
                    break;
                case 'N':
                  //                    ScreenBuf::mode = ScreenBuf::NONE;
                    break;
                }
            }
        }
        usleep(20000);
    }
#endif // USE_OPENWAM
}


void WamDriver::write_pulse_data() {
    if(owam->pulsetraj != NULL && owam->pulsetraj->done && !owam->pulsetraj->record) {
        fstream filestream;
        filestream.open("pulsetraj.txt", ios::out);
        for(unsigned int i = 0; i < owam->pulsetraj->posvals.size(); i++) {
            for(unsigned int j = 0; j < nJoints;j++){
                filestream << owam->pulsetraj->posvals[i][j]; 
                filestream <<  ", ";
            }
            for(unsigned int j = 0; j < nJoints;j++) {
                filestream << owam->pulsetraj->velvals[i][j]; 
                filestream <<  ", ";
            }
            for(unsigned int j = 0; j < nJoints;j++) {
                filestream << owam->pulsetraj->accelvals[i][j]; 
                filestream <<  ", ";
            }
            for(unsigned int j = 0; j < nJoints;j++) {
                filestream << owam->pulsetraj->trqvals[i][j]; 
                if(j != nJoints -1) {
                    filestream <<  ", ";
                }
            }
            filestream << endl;
        }
        filestream.close();
    }
    ROS_DEBUG("Pulse data written to pulsetraj.txt");
}




void WamDriver::set_home_position() {

  ROS_ERROR("\007When ready, type HOME<return>\007");
  char *line=NULL;
  size_t linelen = 0;
  linelen = getline(&line,&linelen,stdin);
  while ((linelen < 4) || strncmp(line,"HOME",4)) {
    ROS_ERROR("\007\nYou must type the word HOME and press <return>\007");
    linelen = getline(&line,&linelen,stdin);
  }
  free(line);
    
    if (FILE *moff_file = fopen(joint_calibration_file,"r")) {
      ROS_INFO("Applying encoder offsets from file %s",joint_calibration_file);
        int32_t initial_mech[nJoints+1];
        for (unsigned int m =1; m <= nJoints; ++m) {
            puck_offsets[m] = initial_mech[m]=0;
        }
        char *jvalstr = NULL;
        size_t jsize = 0;
        linelen = getline(&jvalstr,&jsize,moff_file); // getline will do the malloc
        if (strncmp(jvalstr,"WAM encoder offset values",strlen("WAM encoder offset values"))) {
            ROS_ERROR("Unrecognized format in joint calibration file \"%s\"",joint_calibration_file);
            goto NOCALIB;
        } else {
            int puck_id;
            int32_t offset, initial;
            unsigned int puckcount=0;
            while (fscanf(moff_file,"%d = %d %d\n",
                          &puck_id,&offset,&initial) ==3 ) {
                puck_offsets[puck_id] = offset;
                initial_mech[puck_id] = initial;
                puckcount++;
            }
            if (puckcount != nJoints) {
                ROS_ERROR("Unrecognized format in joint calibration file \"%s\"",joint_calibration_file);
                goto NOCALIB;
            }
            int32_t apvals[bus->n_arm_pucks+1];
            for (puck_id = 1; puck_id<=bus->n_arm_pucks; ++puck_id) {
	      int32_t mech;
	      int32_t old_AP;
                
                get_puck_offset(bus->pucks[puck_id].id(),&mech,&old_AP);

                // Check to make sure we're mapping the offset to
                // the nearest half-rev of the motor.  If it looks like
                // we're more than half a rev away for when we saved
                // the offset, add or subtract a full revolution.
                int32_t puckdiff = mech - initial_mech[puck_id];
                if (puckdiff < -2048) {
                    apvals[puck_id] = mech + puck_offsets[puck_id] + 4096;
                } else if (puckdiff > 2048) {
                    apvals[puck_id] = mech + puck_offsets[puck_id] - 4096;
                } else {
                    apvals[puck_id] = mech + puck_offsets[puck_id];
                }
                
                ROS_DEBUG("Changing puck %d from AP=%d to %d",puck_id,old_AP,apvals[puck_id]);
            }
            ROS_DEBUG("Setting new positions");
            if (bus->send_AP(apvals) == OW_FAILURE) {
                ROS_FATAL("Unable to update pucks with calibrated home position");
                throw -1;
            }
            ROS_DEBUG("Positions set");
            for (puck_id = 1; puck_id<=bus->n_arm_pucks; ++puck_id) {
                int32_t mech;
                int32_t old_AP;
                get_puck_offset(bus->pucks[puck_id].id(),&mech,&old_AP);
        }
        }
        free(jvalstr);
        fclose(moff_file);
    } else {
    NOCALIB:
        ROS_ERROR("Could not read WAM calibration file \"%s\";",joint_calibration_file);
        ROS_ERROR("using home position values instead.");
        
        if (owam->set_jpos(wamhome) == OW_FAILURE) {
            ROS_FATAL("Unable to define WAM home position");
            throw -1;
        }
        
        // record the offsets in case we go into calibration later
        for (unsigned int i=1; i<=nJoints; ++i) {
            puck_offsets[i]=get_puck_offset(i);
        }
    }
}

void WamDriver::start_control_loop() {
    // before starting the control thread, suppress faults and
    // get the position from the WAM, so that the safety module knows
    // where it's starting from
#ifndef BH280_ONLY
  if(bus->set_property_rt(SAFETY_MODULE, IFAULT, 4, false, 10000) == OW_FAILURE){
        ROS_FATAL("Unable to suppress safety module faults.");
        throw -1;
    }
    double jointpos[nJoints+1];
    owam->get_current_data(jointpos,NULL,NULL); // update the joint positions
    // Re-activate the tip velocity-limit checking.
    if(bus->set_property_rt(SAFETY_MODULE, ZERO, 1, false, 10000) == OW_FAILURE){
        ROS_FATAL("Unable to re-activate safety module.");
        throw -1;
    }
#endif // BH280_ONLY
    ROS_INFO("Starting control thread");

    if (owam->start() == OW_FAILURE) {
        ROS_FATAL("Failed to start OpenWAM control thread");
        throw -1;
    }

#ifndef BH280_ONLY
    // confirm with user
    ROS_ERROR("Verify the following:");
    ROS_ERROR("  1. WAM has %d joints (%s)",nJoints,nJoints==4?"wrist NOT INSTALLED":nJoints==7?"wrist INSTALLED":"WARNING: UNFAMILIAR CONFIG");
    ROS_ERROR("  2. Tool mass is %fkg (%s)",owam->links[nJoints].mass,owam->links[nJoints].mass==1.3?"Barrett Hand with cameras":owam->links[nJoints].mass==0.0?"no tool":"custom tool");
    
    ROS_ERROR("Press Shift-Activate to activate motors and start system.");

#ifdef AUTO_ACTIVATE
    if (bus->set_property_rt(10,MODE,2,false) == OW_FAILURE) {
        ROS_FATAL("Error changing mode of puck 10");
        throw -1;
    }
    for (int p=1; p<8; p++) {
        if (bus->set_property_rt(p,MODE,2,false) == OW_FAILURE) {
            ROS_FATAL("Error changing mode of puck %d",p);
            throw -1;
        }
    }
#endif // AUTO_ACTIVATE
    
    // check the mode of puck 1 to detect when activate is pressed
    while ((bus->get_puck_state() != 2) && (ros::ok())) {
      usleep(50000);
      static int statcount=0;
      if (++statcount == 20) {
	owam->rosprint_stats();
	statcount=0;
      }
    }
    if (!ros::ok()) {
      throw -1;
    }
#endif // not BH280_ONLY    
}

void WamDriver::stop_control_loop() {
    owam->stop();
}

bool WamDriver::move_until_stop(int joint, double stop, double limit, double velocity, double &original_joint_pos) {
    double jointpos[nJoints+1];
    owam->hold_position(jointpos);
    TrajPoint curpoint(nJoints);
    curpoint.absolute_time=0;
    for (size_t i=0; i<curpoint.size(); ++i) {
        curpoint[i]=jointpos[i+1];
    }
    TrajType traj;
    traj.push_back(curpoint);
    original_joint_pos=curpoint[joint-1];
    curpoint[joint-1]=limit;
    traj.push_back(curpoint);
    std::vector<double> modified_joint_vel(Plugin::_joint_vel);
    // override the default joint velocity to match what was specified
    modified_joint_vel[joint-1]=velocity;
    std::vector<double> modified_joint_accel(Plugin::_joint_accel);
    // rescale the acceleration to match the new velocity
    modified_joint_accel[joint-1]=Plugin::_joint_accel[joint-1]*velocity/Plugin::_joint_vel[joint-1];
    try {
      ParaJointTraj *paratraj = new ParaJointTraj(traj,modified_joint_vel,modified_joint_accel,false,true,false,false);
      ROS_DEBUG_NAMED("calibration",
		      "Moving joint %d from %2.2f to %2.2f",
		      joint,original_joint_pos,limit);
      if (owam->run_trajectory(paratraj) == OW_FAILURE) {
	ROS_ERROR_NAMED("calibration",
			"Error starting joint %d trajectory",joint);
        return false;
      }
    } catch (const char *msg) {
      ROS_ERROR_NAMED("calibration","Could not create trajectory to move joint: %s",msg);
      return false;
    }


    // loop until the trajectory finishes or gets stuck
    owam->lock(); // lock before checking jointstraj
    while (owam->jointstraj) {
      if (owam->jointstraj->state() == OWD::Trajectory::STOP) {
            owam->unlock();
            // joint is stuck, so stop the trajectory
            ROS_DEBUG_NAMED("calibration","Joint hit torque limit.");
            owam->cancel_trajectory();
            owam->lock(); // just so that we can unlock again outside the loop
            break;
        }
        owam->unlock();
        usleep(50000); // 50ms delay
        owam->lock(); // lock before rechecking jointstraj
    }
    owam->unlock();
    // check our resting position
    owam->get_current_data(jointpos,NULL,NULL);
    ROS_DEBUG_NAMED("calibration",
		    "Joint stopped at angle %2.2f",jointpos[joint]);
    
    float max_error = 0.12; //normal error
    
    if ((jointpos[joint] > stop-max_error) && (jointpos[joint] < stop+max_error) ) {
      // we stopped within .12 radians of the expected position: good!
      // (the worst-case for being off by one complete motor rev is 
      // .15 radians (J1), so if we're within .12 we're fine.
      return true;
    } else {
      // we stopped somewhere else
      return false;
    }
    return false;
}

bool WamDriver::move_joint(int joint, double newpos, double velocity) {
    double jointpos[nJoints+1];
    owam->hold_position(jointpos);
    TrajPoint curpoint(nJoints);
    curpoint.absolute_time=0;
    for (size_t i=0; i<curpoint.size(); ++i) {
        curpoint[i]=jointpos[i+1];
    }
    TrajType traj;
    traj.push_back(curpoint);
    curpoint[joint-1]=newpos;
    traj.push_back(curpoint);
    std::vector<double> modified_joint_vel(Plugin::_joint_vel);
    // override the default joint velocity to match what was specified
    modified_joint_vel[joint-1]=velocity;
    std::vector<double> modified_joint_accel(Plugin::_joint_accel);
    // rescale the acceleration to match the new velocity
    modified_joint_accel[joint-1]=Plugin::_joint_accel[joint-1]*velocity/Plugin::_joint_vel[joint-1];
    try {
      ParaJointTraj *paratraj = new ParaJointTraj(traj,modified_joint_vel,modified_joint_accel,false,false,false,false);
      if (owam->run_trajectory(paratraj) == OW_FAILURE) {
	ROS_ERROR_NAMED("calibration",
			"Error starting joint %d trajectory",joint);
        return false;
      }
    } catch (const char *msg) {
      ROS_ERROR_NAMED("calibration","Could not create trajectory to move joint: %s",msg);
      return false;
    }

    // loop until the trajectory finishes or gets stuck
    owam->lock(); // lock before checking jointstraj
    while (owam->jointstraj) {
      if (owam->jointstraj->state()==OWD::Trajectory::STOP) {
            owam->unlock();
            // joint got stuck
            owam->cancel_trajectory();
            return false;
        }
        owam->unlock();
        usleep(50000); // 50ms delay
        owam->lock(); // lock before rechecking jointstraj
    }
    owam->unlock();
    // we finished the trajectory ok
    return true;
}

bool WamDriver::verify_home_position() {
  ROS_INFO_NAMED("calibration","Checking joints: ");
  fflush(stdout);
  double orig_jpos;
  if (nJoints==7) {
    ROS_INFO_NAMED("calibration"," 7");
    // J7 check: make sure we hit the stop around 3.02 when moving to 3.2
    if (!move_until_stop(7,3.02,3.2,1.0,orig_jpos)) {
      ROS_ERROR_NAMED("calibration","\nERROR: Joint 7 was not turned to its positive limit.\n");
      ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
      ROS_ERROR_NAMED("calibration","then turn the joint fully CCW as seen from the hand.");
      owam->release_position();
      return false;
    }
    // return to initial
    move_joint(7,orig_jpos,1.0);
    
    ROS_INFO_NAMED("calibration"," 6"); fflush(stdout);
    // J6 check: make sure we hit the stop around 1.57 when moving to 1.75
    if (!move_until_stop(6,1.57,1.75,1.0,orig_jpos)) {
      ROS_ERROR_NAMED("calibration","\nERROR: Joint 6 was not started in the center of its range.\n");
      ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
      ROS_ERROR_NAMED("calibration","then adjust the joint.");
      owam->release_position();
      return false;
    }
    // return to initial
    move_joint(6,orig_jpos,1.0);
    
    ROS_INFO_NAMED("calibration"," 5"); fflush(stdout);
    // J5 check: make sure we hit the stop around 1.31 when moving to 1.5
    if (!move_until_stop(5,1.31,1.5,1.0,orig_jpos)) {
      ROS_ERROR_NAMED("calibration","\nERROR: Joint 5 was not turned to its positive limit.\n");
      ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
      ROS_ERROR_NAMED("calibration","then turn the joint fully CCW as seen from the hand.");
      owam->release_position();
      return false;
    }
    // return to initial
    move_joint(5,orig_jpos,1.0);
  }

  ROS_INFO_NAMED("calibration"," 2"); fflush(stdout);
  double orig_j2pos;
  // J2 check: make sure we hit the stop around 1.97 when moving to 2.2
  if (!move_until_stop(2,1.97,2.2,1.0,orig_j2pos)) {
    ROS_ERROR_NAMED("calibration","\nERROR: Joint 2 was not turned to its positive limit.\n");
    ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
    ROS_ERROR_NAMED("calibration","then swing the joint all the way to its positive stop.");
    owam->release_position();
    return false;
  }
  // move to 1.9 to allow testing of J3
  ROS_INFO_NAMED("calibration","Moving J2 to 1.94 to allow space for J3");
  move_joint(2,1.94,1.0);
  
  ROS_INFO_NAMED("calibration"," 3"); fflush(stdout);
  // J3 check: make sure we hit the stop around -2.8 when moving to -3.0
  if (!move_until_stop(3,-2.8,-3.0,1.0,orig_jpos)) {
    ROS_ERROR_NAMED("calibration","\nERROR: Joint 3 was not turned to its negative limit.\n");
    ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
    ROS_ERROR_NAMED("calibration","then turn the joint fully CW as seen from the hand.");
    owam->release_position();
    return false;
    }
  // return to initial
  move_joint(3,orig_jpos,1.0);

  // return J2 to initial
  move_joint(2,orig_j2pos,1.0);
    
  ROS_INFO_NAMED("calibration"," 4"); fflush(stdout);
  // J4 check: make sure we hit the stop around -.83 when moving to -1.05
  if (!move_until_stop(4,-0.83,-1.05,1.0,orig_jpos)) {
    ROS_ERROR_NAMED("calibration","\nERROR: Joint 4 was not turned to its negative limit.\n");
    ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
    ROS_ERROR_NAMED("calibration","then bend the joint fully backwards.");
    owam->release_position();
    return false;
  }
  // return to initial
  move_joint(4,orig_jpos,1.0);

  ROS_INFO_NAMED("calibration"," 1"); fflush(stdout);
  // J1 check: make sure we hit the stop around -2.66 when moving to -2.85
  if (!move_until_stop(1,-2.66,-2.85,1.0,orig_jpos)) {
    ROS_ERROR_NAMED("calibration","\nERROR: Joint 1 was not turned to its negative limit.\n");
    ROS_ERROR_NAMED("calibration","Please press Shift-Idle to turn off the motors,");
    ROS_ERROR_NAMED("calibration","then turn the joint fully CW as seen from above.");
    owam->release_position();
    return false;
  }
  // return to initial
  move_joint(1,orig_jpos,1.0);
  owam->release_position();
  
  ROS_INFO_NAMED("calibration"," done.");
  return true;
}

bool WamDriver::Publish() {
  double jointpos[nJoints+1];
  double abs_jointpos[4+1];
  double totaltorqs[nJoints+1]; // total actual torq
  double jointtorqs[nJoints+1];  // PID error-correction torques,
  //                                       after removing dynamic torques
  double trajtorqs[nJoints+1]; // torques output by the current traj
  double simtorqs[nJoints+1];  // Torques calculated from simulated links
                               // (used for experiental mass properties)
  boost::mutex::scoped_lock lock(wamstate_mutex);
  
  wamstate.header.stamp = ros::Time::now();

  if (! ros::ok()) {
    wamstate.state = owd_msgs::WAMState::state_inactive;
    pub_wamstate.publish(wamstate);  // no need to fill in anything else
    return true;
  }

  owam->get_current_data(jointpos,totaltorqs,jointtorqs,simtorqs,trajtorqs);
  owam->get_abs_positions(abs_jointpos);
  owam->get_gains(waminternals.gains);

  for (unsigned int i=0; i<nJoints; ++i) {
    wamstate.positions[i] = owam->last_control_position[i];
    waminternals.positions[i] = jointpos[i+1];
    if (i<4) {
      wamstate.jpositions[i]=abs_jointpos[i+1];
    }
    wamstate.velocities[i] = owam->arm_velocity[i+1];
    wamstate.torques[i] = jointtorqs[i+1];
    waminternals.total_torque[i] 
      = waminternals.dynamic_torque[i]
      = totaltorqs[i+1];
    waminternals.dynamic_torque[i] -= jointtorqs[i+1] + trajtorqs[i+1];
    waminternals.trajectory_torque[i] = trajtorqs[i+1];
    waminternals.sim_torque[i] = simtorqs[i+1];
    waminternals.joint_offsets[i] = owam->joints[i+1].offset;
    waminternals.elbow_velocity = owam->elbow_vel.norm();
    waminternals.endpoint_velocity = owam->endpoint_vel.norm();
    waminternals.barrett_endpoint_velocity = owam->barrett_endpoint_vel;
    // publish as transforms, too
    char jref[50], jname[50];
    snprintf(jref,50,"wam%d",i);
    snprintf(jname,50,"wam%d",i+1);
    std::string jrefstring(jref);
    std::string jnamestring(jname);
    tf::Quaternion YAW;
    YAW.setEulerZYX(jointpos[i+1],0,0);
    tf::Transform wam_tf = wam_tf_base[i] *  tf::Transform(YAW);
    tf::StampedTransform st(wam_tf,ros::Time::now(),jrefstring,jnamestring);
    tf_broadcaster.sendTransform(st);
  }

  owam->lock();
  // Get the data we might need and then unlock as quickly as we can.
  // Whatever you do, DO NOT call any ROS functions in this block, or
  // you run the risk of slowing down the control loop enough to cause a
  // heartbeat fault.
  bool safety_hold = owam->safety_hold;
  bool holdpos = owam->holdpos;
  bool jointstraj(false);
  std::string trajid;
  if (owam->jointstraj) {
    jointstraj=true;
    trajid = owam->jointstraj->id;
  }
  char jointstr[200];
  strcpy(jointstr,"");
  for(int i = Joint::J1; i<=Joint::Jn; i++) {
    snprintf(jointstr+strlen(jointstr),199-strlen(jointstr)," %1.4f",owam->last_control_position[i]);
  }
  wamstate.stiffness = owam->stiffness;
  wamstate.controller = owam->jscontroller->name;
  if (owam->new_jscontroller) {
    // we're in the process of switching controllers, so publish
    // both controller names
    wamstate.controller += std::string(" + ");
    wamstate.controller += owam->new_jscontroller->name;
  }
  owam->unlock();

  // Now that we've unlocked the controller again, we can figure out what's going
  // on the with trajectory and log any messages as necessary.
  if (jointstraj) {
    static bool stall_reported(false);
    // trajectory is still running, but we still might be hitting something
    if (safety_hold) {
      wamstate.state=owd_msgs::WAMState::state_traj_stalled;
      if (!stall_reported) {
	ROS_INFO_NAMED("AddTrajectory","Trajectory %s has stalled at position[%s ]; trying to continue...",
		       trajid.c_str(),jointstr);
	stall_reported=true;
      }
    } else {
      wamstate.state=owd_msgs::WAMState::state_traj_active;
      if (stall_reported) {
	ROS_INFO_NAMED("AddTrajectory","Stalled trajectory %s has resumed at position[%s ]",
		       trajid.c_str(),jointstr);
	stall_reported=false;
      }
    }
  } else if (holdpos) {
    wamstate.state = owd_msgs::WAMState::state_fixed;
  } else {
    wamstate.state = owd_msgs::WAMState::state_free;
  }

  pub_wamstate.publish(wamstate);
  pub_waminternals.publish(waminternals);
  
  boost::mutex::scoped_lock plock(plugin_mutex);
  OWD::Plugin::PublishAll();

  return true;
}

// Handle requests for DOF information
bool WamDriver::GetDOF(owd_msgs::GetDOF::Request &req,
                       owd_msgs::GetDOF::Response &res) {
#ifdef FAKE7
  res.nDOF = 4;
#else
  res.nDOF =nJoints;
#endif // FAKE7

  return true;
}

bool WamDriver::CalibrateJoints(owd_msgs::CalibrateJoints::Request &req,
    owd_msgs::CalibrateJoints::Response &res) {
    calibrate_joint_angles();
    return true;
}

bool WamDriver::AddTrajectory(owd_msgs::AddTrajectory::Request &req,
                              owd_msgs::AddTrajectory::Response &res) {

  // this function takes a sequence of jointspace points and builds
  // a trajectory object.  the request will be rejected if:
  //  1) there are not at least 2 points
  //  2) the first point is too far from the arm's current position (or
  //     the ending position of the last queued trajectory)
  //  3) if there's a problem with the trajectory geometry and
  //     the timing cannot be computed
  // if the first point is a close match, it will be adjusted to an
  // exact match.

  res.ok=true;
  res.reason="";

  // check number of points
  if (req.traj.positions.size() < 2) {
    ROS_ERROR_NAMED("AddTrajectory","Minimum of 2 traj points required");
    res.id = req.traj.id;
    res.ok=false;
    res.reason="Minimum of 2 traj points required";
    return true;
  }

#ifdef FAKE7
  if (nJoints !=4) {
    ROS_ERROR("FAKE7 was defined but openwam was compiled with 7 DOF");
    ROS_ERROR("Remove -DWRIST from openwam and relink");
    res.id = req.traj.id;
    res.ok=false;
    res.reason="FAKE7 was defined but openwam was compiled with 7 DOF";
    return true;
  }
    
  // we're only running a 4DOF wam, so strip the extra DOFS from the
  // trajectory points
  for (unsigned int i=0; i<req.traj.positions.size(); ++i) {
    req.traj.positions[i].j.resize(4);
  }
#endif // FAKE7

  // check all joint positions for bad values
  for (unsigned int i=0; i<req.traj.positions.size(); ++i) {
    if (req.traj.positions[i].j.size() != nJoints) {
      ROS_ERROR("Wrong number of joint values at point %d in trajectory (expected %d but got %zd",
		i, nJoints, req.traj.positions[i].j.size());
      res.ok=false;
      res.reason="Invalid trajectory: wrong number of joint values";
      return true;
    }
    for (unsigned int j=0; j<req.traj.positions[i].j.size(); ++j) {
      try {
	if (!is_in_range(req.traj.positions[i].j[j],
			 Plugin::lower_jlimit[j] - 0.2,
			 Plugin::upper_jlimit[j] + 0.2)) {
	  ROS_ERROR("At requested trajectory point %d: value %2.2f for joint %d is not in the range of %2.2f to %2.2f; trajectory rejected",
		    i, req.traj.positions[i].j[j], j,
		    Plugin::lower_jlimit[j] - 0.2,
		    Plugin::upper_jlimit[j] + 0.2);
	  res.ok=false;
	  res.reason="One or more joint values were outside the allowable range (see OWD log for details)";
	  return true;
	}
      } catch (const char *err) {
	ROS_ERROR("One or more values in the AddTrajectory request were INF or NAN");
	res.ok=false;
	res.reason="One or more joint values were INF or NAN";
	return true;
      }
    }
  }

  if (req.traj.blend_radius.size() == 0) {
    req.traj.blend_radius.resize(req.traj.positions.size(),0);
  } else if (req.traj.blend_radius.size() != req.traj.positions.size()) {
    res.reason="Size of blend radius array does not match size of position array";
    ROS_ERROR_NAMED("AddTrajectory","%s",res.reason.c_str());
    res.id = req.traj.id;
    res.ok=false;
    return true;
  }

  // get trajectory start point
  JointPos firstpoint = JointPos(req.traj.positions[0].j);

  // figure out where robot will be at start of trajectory
  JointPos curpoint(nJoints);
  double wampos[nJoints+1];
  if (! owam->jointstraj) {

    // if we weren't already holding position, then start now
    if (!owam->holdpos) {
      owam->set_stiffness(1.0);
    }

    // get position from WAM
    owam->hold_position(wampos);
    curpoint.SetFromArray(nJoints,wampos+1);

    // make sure first point is close to current pos
    if (! firstpoint.closeto(curpoint)) {
      ROS_ERROR_NAMED("AddTrajectory","First traj point is too far from current position");
      char firststr[200], curstr[200];
      strcpy(firststr,""); strcpy(curstr,"");
      for (unsigned int j=0; j<nJoints; ++j) {
        snprintf(firststr+strlen(firststr),199-strlen(firststr)," %1.4f",firstpoint[j]);
        snprintf(curstr+strlen(curstr),199-strlen(curstr)," %1.4f",curpoint[j]);
      }
      ROS_ERROR_NAMED("AddTrajectory","Current point: %s",curstr);

      ROS_ERROR_NAMED("AddTrajectory","First point: %s",firststr);
      res.reason="First traj point is too far from current position";
      res.id = req.traj.id;
      res.ok=false;
      return true;
    }

    // if points don't exactly agree, then substitute the current point
    // for the first trajectory point (we already checked to make sure
    // it was close enough)

    if (firstpoint != curpoint) {
      req.traj.positions[0].j = curpoint;
      firstpoint=curpoint;

      // must make sure that the blend radius of the next point
      // isn't too big (the new first point might be closer to the
      // second point than the original)
      JointPos first_segment(JointPos(req.traj.positions[1].j) - firstpoint);
      double segment_length=first_segment.length();

      // just realized on 4/7/2011 that this isn't a very good check;
      // the math only works if the segments are 90 degrees apart.
      // otherwise, the backoff distance that the blend occupies is
      // not equal to the blend radius.  this issue has been ticketed
      // as Trac ticket #97.
      if (req.traj.blend_radius[1] > 0.5*segment_length) {
	req.traj.blend_radius[1]=0.5*segment_length;
      }
    }
  } else {
    // compare against the last queued point
    if (trajectory_list.size() > 0) {
      curpoint = trajectory_list.back()->end_position;
#ifdef FAKE7
      curpoint.resize(nJoints);  // strip off the fake joints
#endif
    } else {
      owam->lock();
      if (! owam->jointstraj) {
        // the trajectory finished while we were monkeying around.
        // go back and try again.
        owam->unlock();
        return AddTrajectory(req,res);
      }
      curpoint = owam->jointstraj->end_position;
      owam->unlock();
    }
    // make sure first point matches current pos or last queued pos.
    if (firstpoint !=  curpoint) {
      ROS_ERROR_NAMED("AddTrajectory","First traj point doesn't match last queued point");
      res.reason="First traj point doesn't match last queued point";
      res.id = req.traj.id;
      res.ok=false;
      return true;
    }
  }

  // check for trajectories that don't actually go anywhere
  bool zerodist = true;
  for (unsigned int k=1; k < req.traj.positions.size(); ++k) {
    if (firstpoint != JointPos(req.traj.positions[k].j)) {
      zerodist=false;
      break;
    }
  }

  if (zerodist) {
    // ParaJointTraj handles the zero-distance case better than
    // MacJointTraj, so wipe out all the blend radii to force a
    // ParaJointTraj to be used.
    for (unsigned int k=0; k < req.traj.blend_radius.size(); ++k) {
	req.traj.blend_radius[k]=0.0;
    }
  }

  // build trajectory
  OWD::Trajectory *t = BuildTrajectory(req.traj);
  if (!t) {
    res.reason="BuildTrajectory failed: ";
    res.reason += std::string(last_trajectory_error);
    res.id = req.traj.id;
    res.ok=false;
    return true;
  }

  // Return success/failure and the traj id
  res.id = t->id;
  res.ok = AddTrajectory(t,res.reason);
  
  // This should be after it's been added, for synchronization
  res.time_added = ros::Time::now();

  return true;
}

bool WamDriver::AddTimedTrajectory(owd_msgs::AddTimedTrajectory::Request &req,
				   owd_msgs::AddTimedTrajectory::Response &res) {
  
  ROS_WARN("About to deserialize a trajectory from a string of len %zd",
	   req.SerializedTrajectory.size());
  try {
    BinaryData bd(req.SerializedTrajectory);
    int trajtype = bd.GetInt();
    BinaryData bd2(bd.GetString());
    OWD::Trajectory *t;
    if (trajtype == Trajectory::TRAJTYPE_MACJOINTTRAJ) {
      t = new MacJointTraj(bd2);
    } else {
      ROS_ERROR("Could not deserialize trajectory: unknown type");
      res.ok=false;
      res.reason = std::string("Did not recognize serialized trajectory type: was it created by an OWD class?");
      res.id = std::string("");
    return true;
    }
    if (req.options & owd_msgs::JointTraj::opt_WaitForStart) {
      t->WaitForStart=true;
    }
    if (req.options & owd_msgs::JointTraj::opt_CancelOnStall) {
      t->CancelOnStall=true;
    }
    if (req.options & owd_msgs::JointTraj::opt_CancelOnForceInput) {
      t->CancelOnForceInput=true;
    }
    if (req.options & owd_msgs::JointTraj::opt_CancelOnTactileInput) {
      t->CancelOnTactileInput=true;
    }
    if (req.options & owd_msgs::JointTraj::opt_Synchronize) {
      t->Synchronize=true;
    }
    
    if (req.id == "") {
      t->id = Trajectory::random_id();
    } else {
      t->id = req.id;
    }
    
    res.id = t->id;
    res.ok = AddTrajectory(t,res.reason);

  } catch (const char *errmsg) {
    ROS_ERROR("Could not deserialize trajectory: %s",errmsg);
    res.ok=false;
    res.reason = std::string("Could not deserialize trajectory: ");
    res.reason += std::string(errmsg);
    res.id = std::string("");
    return true;
  }
    

  // This should be after it's been added, for synchronization
  res.time_added = ros::Time::now();

  return true;
}

bool WamDriver::AddTrajectory(OWD::Trajectory *traj, std::string &failure_reason) {
  // this function takes a precomputed trajectory object and either runs it
  // (if no other traj is running) or queues it for later execution.
  // the first point of the trajectory must exactly match either the current
  // position or the final position of the last queued trajectory.

  failure_reason="";
  // figure out where robot will be at start of trajectory
  JointPos curpoint(nJoints);
  double wampos[nJoints+1];
  if (traj->start_position.size() != nJoints) {
    failure_reason="start_position field is the wrong size";
    return false;
  }
  if (traj->end_position.size() != nJoints) {
    failure_reason="end_position field is the wrong size";
    return false;
  }
  if (!owam->jointstraj) {

    // if we weren't already holding position, then abort, because
    // it's extremely unlikely that the start position of the new trajectory
    // will exactly match
    if (!owam->holdpos) {
      failure_reason="Must hold position before calling AddTrajectory";
      return false;
    }

    // get position from WAM
    owam->hold_position(wampos);
    curpoint.SetFromArray(nJoints,wampos+1);

    // make sure first point matches the current pos
    if (!traj->start_position.verycloseto(curpoint)) {
      ROS_ERROR_NAMED("AddTrajectory","First traj point does not match current position");
      char firststr[200], curstr[200];
      strcpy(firststr,""); strcpy(curstr,"");
      for (unsigned int j=0; j<nJoints; ++j) {
        snprintf(firststr+strlen(firststr),199-strlen(firststr)," %1.4f",traj->start_position[j]);
        snprintf(curstr+strlen(curstr),199-strlen(curstr)," %1.4f",curpoint[j]);
      }
      ROS_ERROR_NAMED("AddTrajectory","Current point: %s",curstr);
      ROS_ERROR_NAMED("AddTrajectory","First point: %s",firststr);
      failure_reason="First traj point is too far from current position";
      return false;
    }

  } else {
    // compare start against the last queued point
    if (trajectory_list.size() > 0) {
      curpoint = trajectory_list.back()->end_position;
#ifdef FAKE7
      curpoint.resize(nJoints);  // strip off the fake joints
#endif
    } else {
      owam->lock();
      if (! owam->jointstraj) {
        // the trajectory finished while we were monkeying around.
        // go back and try again.
        owam->unlock();
        return AddTrajectory(traj,failure_reason);
      }
      curpoint = owam->jointstraj->end_position;
      owam->unlock();
    }
    // make sure first point matches current pos or last queued pos.
    if (traj->start_position !=  curpoint) {
      ROS_ERROR_NAMED("AddTrajectory","First traj point doesn't match last queued point");
      failure_reason="First traj point doesn't match last queued point";
      return false;
    }
  }

  // Everything looks ok at this point, so add the traj info to 
  // the list we publish
  owd_msgs::TrajInfo ti;
  ti.id = traj->id;
  ti.type = traj->type;
  ti.end_position = traj->end_position;
#ifdef FAKE7
  ti.end_position.resize(7);
#endif
  ti.state = owd_msgs::TrajInfo::state_pending;

  boost::mutex::scoped_lock lock(wamstate_mutex);
  // if there are already running / queued trajectories, just add ours.
  // note: the order of these conditional checks is important.  only want to
  // use the owam->jointtraj check if the queue is empty; otherwise you might
  // enter a race condition with the Update loop (both of you will notice
  // jointtraj is null, and both will start changing it).  by making sure
  // the queue is empty first, we know the Update loop will have nothing to do.
  // Synchronized trajectories always get queued, so that the service
  // call can return right away without waiting for the sync start
  if ((trajectory_list.size() > 0) || owam->jointstraj || traj->Synchronize) {
    ROS_INFO("Trajectory %s queued for execution", traj->id.c_str());
    trajectory_list.push_back(traj);
    wamstate.trajectory_queue.push_back(ti);
    return true;
  }

  // run the trajectory
  if (owam->run_trajectory(traj) == OW_FAILURE) {
    wamstate.trajectory_queue.push_back(ti);
    return false;
  }
  wamstate.trajectory_queue.push_back(ti);
  wamstate.trajectory_queue[0].state=owd_msgs::TrajInfo::state_active;
  ROS_INFO("Added trajectory %s",ti.id.c_str());
  char endstr[200];
  strcpy(endstr,"");
  for (unsigned int j=0; j<nJoints; ++j) {
    snprintf(endstr+strlen(endstr),199-strlen(endstr)," %1.4f",ti.end_position[j]);
  }
  ROS_INFO_NAMED("AddTrajectory","Trajectory %s will stop at [%s ]",
		 ti.id.c_str(),endstr);
  return true;
}

bool WamDriver::DeleteTrajectory(owd_msgs::DeleteTrajectory::Request &req,
                                 owd_msgs::DeleteTrajectory::Response &res) {
#ifdef FAKE7
  res.ok=false;
  res.reason="Not implemented for FAKE7 compilation option";
  return true;
#endif

  std::vector<OWD::Trajectory *>::iterator tl_it;
  std::vector<owd_msgs::TrajInfo>::iterator ws_it;
  // lock against re-queuing by Update() function
  boost::mutex::scoped_lock lock(wamstate_mutex);
  for (unsigned int x=0; x<req.ids.size(); ++x) {
    std::string delete_id = req.ids[x];
    owam->lock();
    if (owam->jointstraj && (owam->jointstraj->id == delete_id)) {
      owam->unlock();
      owam->cancel_trajectory();
      if (wamstate.trajectory_queue.size() > 0) {
	wamstate.prev_trajectory = wamstate.trajectory_queue.front();
	wamstate.prev_trajectory.state = owd_msgs::TrajInfo::state_aborted;
	wamstate.trajectory_queue.erase(wamstate.trajectory_queue.begin());
      }
    } else {
      owam->unlock();
      for (tl_it = trajectory_list.begin(),
             ws_it=wamstate.trajectory_queue.begin()+1;
           tl_it != trajectory_list.end(); ++tl_it,++ws_it) {
	if (tl_it != trajectory_list.end()) {
	  if ((*tl_it)->id == delete_id) {
	    trajectory_list.erase(tl_it);
	    if (ws_it != wamstate.trajectory_queue.end()) {
	      wamstate.trajectory_queue.erase(ws_it);
	    }
	    break;
	  }
        }
      }
    }
  }
  // now look for discontinuities in position
  owam->lock();
  JointPos p;
  unsigned int tl_start(0), ws_start(1); // start checking first queued traj
  if (owam->jointstraj) {
    p=owam->jointstraj->end_position;
  } else if (trajectory_list.size() > 0) {
    p=(*tl_it)->end_position;
    tl_start=1; // start checking second queued traj
    ws_start=2;
  } else {
    // no more trajectories
    owam->unlock();
    ROS_INFO("DeleteTrajectory done; no more trajectories");
    return true;
  }
  owam->unlock();
  for (unsigned int tl_check=tl_start,ws_check=ws_start; tl_check < trajectory_list.size(); ++tl_check,++ws_check) {
    // make sure previous endpoint matches beginning of next traj
    while ((tl_check < trajectory_list.size()) && (p != trajectory_list[tl_check]->start_position)) {
      // as long as the points don't match, delete the queued traj
      trajectory_list.erase(trajectory_list.begin()+tl_check);
      if (ws_check < wamstate.trajectory_queue.size()) {
	wamstate.trajectory_queue.erase(wamstate.trajectory_queue.begin()+ws_check);
      }
    }
    if (tl_check < trajectory_list.size()) {
      p=trajectory_list[tl_check]->end_position;
    }
  }
  ROS_INFO("DeleteTrajectory processed: new queue size %zd",
           trajectory_list.size());
  res.ok=true;
  return true;
}

bool WamDriver::SetJointStiffness(owd_msgs::SetJointStiffness::Request &req,
                             owd_msgs::SetJointStiffness::Response &res) {
  if (req.stiffness.size() != nJoints) {
    char errmsg[200];
    snprintf(errmsg,200,"SetJointStiffness expects a vector of %d stiffness values, but %zd were sent",nJoints,req.stiffness.size());
    ROS_WARN("%s",errmsg);
    res.reason=errmsg;
    res.ok=false;
    return true;
  }
  if (owam->jointstraj) {
    owd_msgs::CancelAllTrajectories::Request cancel_req;
    owd_msgs::CancelAllTrajectories::Response cancel_res;
    CancelAllTrajectories(cancel_req,cancel_res);
    ROS_WARN("Trajectory cancelled by SetJointStiffness command");
  }
  double current_pos[8];
  owam->hold_position(current_pos);
  for (unsigned int i=0; i<nJoints; ++i) {
    if (req.stiffness[i] == 0) {
      owam->jscontroller->stop(i);
    } else if (!owam->jscontroller->active(i)) {
      // since the joint was not previously active, move
      // the setpoint to the current position
      owam->heldPositions[i+1]=owam->joints[i+1].q;
      owam->jscontroller->reset(i);
      owam->jscontroller->run(i);
    }
  }
  res.ok=true;
  return true;
}

bool WamDriver::GetStiffness(owd_msgs::GetStiffness::Request &req,
                             owd_msgs::GetStiffness::Response &res) {
  res.stiffness = owam->stiffness;
  res.ok=true;
  return true;
}

bool WamDriver::SetStiffness(owd_msgs::SetStiffness::Request &req,
                             owd_msgs::SetStiffness::Response &res) {
  
  try {
    if (!is_in_range(req.stiffness,0,1)) {
      ROS_ERROR("SetStiffness value of %2.2f ignored",req.stiffness);
      res.ok=false;
      res.reason="Stiffness value not in range of 0 to 1";
      return true;
    }
  } catch (const char *err) {
    ROS_ERROR("Invalid value passed to SetStiffness: %s", err);
    res.ok=false;
    res.reason="Stiffness value was INF or NAN";
    return true;
  }
  if (req.stiffness > 0.0) {
    if (!owam->jointstraj) {
      // if we're not running a trajectory, then hold the current position
      if (req.stiffness > 1.0) {
	owam->set_stiffness(1.0);
      } else {
	owam->set_stiffness(req.stiffness);
      }
      owam->hold_position();
      ROS_INFO("Position held by SetStiffness command");
    }
  } else {
    if (owam->jointstraj) {
      owd_msgs::CancelAllTrajectories::Request cancel_req;
      owd_msgs::CancelAllTrajectories::Response cancel_res;
      CancelAllTrajectories(cancel_req,cancel_res);
    }

    // Release position
    
    owam->release_position();
    owam->set_stiffness(0.0);
    ROS_INFO("Position released by SetStiffness command");
  }
  res.ok=true;
  return true;
}

bool WamDriver::SetJointOffsets(owd_msgs::SetJointOffsets::Request &req,
                             owd_msgs::SetJointOffsets::Response &res) {
  if (req.offset.size() != nJoints) {
    char errmsg[200];
    snprintf(errmsg,200,"SetJointOffsets expects a vector of %d offset values, but %zd were sent",nJoints,req.offset.size());
    ROS_WARN("%s",errmsg);
    res.reason=errmsg;
    res.ok=false;
    return true;
  }
  // Check to make sure the offset values are reasonable (and not NAN!)
  for (unsigned int i=0; i<req.offset.size(); ++i) {
    try {
      if (!is_in_range(req.offset[i],-30.0/360.0*TWOPI,30.0/360.0*TWOPI)) {
	char errmsg[200];
	snprintf(errmsg,200,"Requested offset of %3.2f radians for joint %d was not in the range of +/- 30 degrees",
		 req.offset[i],i+1);
	res.ok=false;
	res.reason=errmsg;
	return true;
      }
    } catch (const char *err) {
      char errmsg[200];
      snprintf(errmsg,200,"Error while checking joint %d offset: %s",
	       i+1, err);
      res.ok=false;
      res.reason=errmsg;
      return true;
    }
  }
  double *j_offsets = (double *)malloc((nJoints+1) * sizeof(double));
  if (j_offsets == NULL) {
    res.ok=false;
    res.reason="out of memory";
    return true;
  }
  memcpy(j_offsets+1, &req.offset[0], nJoints*sizeof(double));
  res.reason="";
  res.ok=true;
  if (owam->set_joint_offsets(j_offsets) != OW_SUCCESS) {
    res.ok=false;
    res.reason="Cannot change joint offsets while a trajectory is active";
  }
  free(j_offsets);
  return true;
}

bool WamDriver::PauseTrajectory(owd_msgs::PauseTrajectory::Request &req,
                                owd_msgs::PauseTrajectory::Response &res) {
  if (owam->jointstraj) {
    if (req.pause) {
      owam->pause_trajectory();
      ROS_INFO("Trajectory paused");
    } else {
      owam->resume_trajectory();
      ROS_INFO("Trajectory resumed");
    }
    res.ok=true;
  } else {
    res.ok=false;
    res.reason="PauseTrajectory called while no trajectory was running";
    ROS_ERROR("%s",res.reason.c_str());
  }
  return true;
}

bool WamDriver::ReplaceTrajectory(owd_msgs::ReplaceTrajectory::Request &req,
                                  owd_msgs::ReplaceTrajectory::Response &res) {
  // not implemented yet
  return false;


  /* old code that might help:
     if (trajectory && trajectory->id == pCurTraj->id) {
     delete trajectory; // wipe out the current one
     listCommands.push_front(pCurTraj); // add this to the front
     ROS_WARN("Added trajectory #%d to the front of the command queue\n",cmdnum);
     syslog(LOG_ERR,"Added trajectory #%d to the front of the command queue",cmdnum);
     log_rave_trajectory(trajname,pCurTraj->vtraj);
     pCurTraj=NULL;
     return 0;
     }
     // check to see if the trajectory replaces a queued one
     for (std::list<Command *>::iterator it= listCommands.begin();
     it != listCommands.end(); ++it) {
     if (((TrajectoryCommand *)(*it))->id == pCurTraj->id) {
     delete *it; // remove the old one
     *it = pCurTraj; // repoint to the new one
     ROS_WARN("Added trajectory #%d to the command queue, replacing previous one\n",cmdnum);
     syslog(LOG_ERR,"Added trajectory #%d to the command queue, replacing previous one",cmdnum);
     log_rave_trajectory(trajname,pCurTraj->vtraj);
     pCurTraj=NULL;
     return 0;
  */
}

bool WamDriver::CancelAllTrajectories(
     owd_msgs::CancelAllTrajectories::Request &req,
     owd_msgs::CancelAllTrajectories::Response &res) {
  while (trajectory_list.size() > 0) {
    // remove each one from the list first, then delete it.
    // that way we won't screw up other threads
    std::vector<OWD::Trajectory *>::iterator it=trajectory_list.begin();
    OWD::Trajectory *deleteme = *it;  // get the pointer value
    trajectory_list.erase(it); // remove from the vector
    delete deleteme;  // delete the actual traj
  }
  if (owam->jointstraj) {
    owam->cancel_trajectory();
  }
  boost::mutex::scoped_lock lock(wamstate_mutex);
  if (wamstate.trajectory_queue.size() > 0) {
    wamstate.prev_trajectory = wamstate.trajectory_queue.front();
    wamstate.prev_trajectory.state = owd_msgs::TrajInfo::state_aborted;
    wamstate.trajectory_queue.clear();
  }
  res.ok=true;
  return true;
}


bool WamDriver::SetSpeed(owd_msgs::SetSpeed::Request &req,
                         owd_msgs::SetSpeed::Response &res) {
  if (req.velocities.size() != nJoints) {
    res.ok=false;
    res.reason="Received SetSpeed command with wrong array size";
    ROS_ERROR("%s",res.reason.c_str());
    return true;
  }
  if (isinf(req.min_accel_time)) {
    ROS_ERROR("Infinite min_accel_time passed to SetSpeed service call; entire call rejected.");
      res.ok=false;
      res.reason="min_accel_time was infinite.";
      return true;
  }
  if (isnan(req.min_accel_time)) {
    ROS_ERROR("NAN min_accel_time passed to SetSpeed service call; entire call rejected.");
    res.ok=false;
    res.reason="min_accel_time was NAN.";
    return true;
  }
  for (unsigned int i=0; i<nJoints; ++i) {
    if (isinf(req.velocities[i])) {
      ROS_ERROR("Infinite velocity passed to SetSpeed service call; entire call rejected.");
      res.ok=false;
      res.reason="One or more joint speeds were infinite.";
      return true;
    }
    if (isnan(req.velocities[i])) {
      ROS_ERROR("NAN velocity passed to SetSpeed service call; entire call rejected.");
      res.ok=false;
      res.reason="One or more joint speeds were NAN.";
      return true;
    }
    if (req.velocities[i] > Plugin::_max_joint_vel[i]) {
      // limit to max
      Plugin::_joint_vel[i] = Plugin::_max_joint_vel[i];
      ROS_WARN("Limited joint %d velocity to max %2.2f radians/s",i,Plugin::_joint_vel[i]);
    } else if (req.velocities[i] < 0.05 * Plugin::_max_joint_vel[i]) {
      // limit to no less than 5% of max
      Plugin::_joint_vel[i] = 0.05 * Plugin::_max_joint_vel[i];
      ROS_WARN("Limited joint %d velocity to min %2.2f radians/s",i,Plugin::_joint_vel[i]);
    } else {
      Plugin::_joint_vel[i] = req.velocities[i];
      ROS_INFO("Set joint %d velocity limit to %2.2f radians/s",i,Plugin::_joint_vel[i]);
    }
    Plugin::_joint_accel[i] = Plugin::_joint_vel[i] / req.min_accel_time;
  }
  res.ok=true;
  return true;
}

bool WamDriver::GetSpeed(owd_msgs::GetSpeed::Request &req,
                         owd_msgs::GetSpeed::Response &res) {
  res.max_velocity = Plugin::_joint_vel;
  res.max_acceleration = Plugin::_joint_accel;
  res.max_jerk = Plugin::_max_jerk;
  return true;
}
    
bool WamDriver::SetExtraMass(owd_msgs::SetExtraMass::Request &req,
			     owd_msgs::SetExtraMass::Response &res) {
  
  if (isinf(req.m.mass) || isnan(req.m.mass) ||
      isinf(req.m.cog_x) || isnan(req.m.cog_x) ||
      isinf(req.m.cog_y) || isnan(req.m.cog_y) ||
      isinf(req.m.cog_z) || isnan(req.m.cog_z) ||
      isinf(req.m.inertia_xx) || isnan(req.m.inertia_xx) ||
      isinf(req.m.inertia_xy) || isnan(req.m.inertia_xy) ||
      isinf(req.m.inertia_xz) || isnan(req.m.inertia_xz) ||
      isinf(req.m.inertia_yy) || isnan(req.m.inertia_yy) ||
      isinf(req.m.inertia_yz) || isnan(req.m.inertia_yz) ||
      isinf(req.m.inertia_zz) || isnan(req.m.inertia_zz)) {
    res.ok=false;
    res.reason="One or more values are INF or NAN";
    return true;
  }
     
  owam->lock("SetExtraMass");
  if ((req.m.link < Link::L1) ||
      (req.m.link > Link::Ln)) {
    owam->unlock();
    res.ok=false;
    res.reason="Specified link is out of range";
    return true; // always return true if we received the request
  }
  
  // add the masses
  owam->links[req.m.link].mass = 
    owam->original_links[req.m.link].mass
    + req.m.mass;

  // Weight the new CG by the individual masses
  owam->links[req.m.link].cog.x[0] = 
    (owam->original_links[req.m.link].cog.x[0] * owam->original_links[req.m.link].mass
     + req.m.cog_x * req.m.mass)
    / owam->links[req.m.link].mass;

  owam->links[req.m.link].cog.x[1] =
    (owam->original_links[req.m.link].cog.x[1] * owam->original_links[req.m.link].mass
     + req.m.cog_y * req.m.mass) 
    / owam->links[req.m.link].mass;

  owam->links[req.m.link].cog.x[2] = 
    (owam->original_links[req.m.link].cog.x[2] * owam->original_links[req.m.link].mass
     + req.m.cog_z * req.m.mass)
    / owam->links[req.m.link].mass;

  // add the inertias
  owam->links[req.m.link].inertia =
    owam->original_links[req.m.link].inertia 
    + Inertia(  req.m.inertia_xx, req.m.inertia_xy, req.m.inertia_xz,
                req.m.inertia_yy, req.m.inertia_yz, req.m.inertia_zz);

  owam->unlock("SetExtraMass");
  res.ok=true;
  return true;
}

bool WamDriver::SetStallSensitivity(owd_msgs::SetStallSensitivity::Request &req,
				    owd_msgs::SetStallSensitivity::Response &res) {

  res.ok = true;
  if (isnan(req.level) || isinf(req.level)) {
    res.ok=false;
    res.reason="Requested value was INF or NAN";
    return true;
  }
  if (req.level > 1.0) {
    req.level = 1.0;
    res.reason = "Warning: sensitivity limited to 1.0";
  } else  if (req.level < 0) {
    req.level = 0;
    res.reason = "Warning: sensitivity cannot be negative; changed to zero";
  }
  owam->stall_sensitivity = req.level;
  return true;
}

bool WamDriver::GetStallSensitivity(owd_msgs::GetStallSensitivity::Request &req,
				    owd_msgs::GetStallSensitivity::Response &res)
{
  res.level = owam->stall_sensitivity;
  return true;
}

 
void WamDriver::MassProperties_callback(const boost::shared_ptr<const owd_msgs::MassProperties> &mass) {
  if ((mass->link < Link::L1) || (mass->link > Link::Ln)) {
    ROS_ERROR_NAMED("mass","Received MassProperties message with link out of range");
    return;
  }
  owam->lock("OWD MassProperties cb");
  owam->sim_links[mass->link].mass = mass->mass;

  owam->sim_links[mass->link].cog.x[0] = mass->cog_x;
  owam->sim_links[mass->link].cog.x[1] = mass->cog_y;
  owam->sim_links[mass->link].cog.x[2] = mass->cog_z;
  
  owam->sim_links[mass->link].inertia 
    = Inertia(  mass->inertia_xx, mass->inertia_xy, mass->inertia_xz,
                mass->inertia_yy, mass->inertia_yz, mass->inertia_zz);

  owam->unlock("OWD MassProperties cb");
  return;
}

void WamDriver::Pump(const ros::TimerEvent& e) {
    // Perform periodic tasks

    // let the driver update
    this->Update();

    // publish our state info
    this->Publish();
}

void WamDriver::Update() {
  
#ifndef OWDSIM // don't log data if running simulation
  static int statcount = 0;
  if (++statcount == 20) {
    owam->rosprint_stats();
    if (owam->recorder.count > 1000) {
      static int dumpnum=0;
      char filename[200];
      snprintf(filename,200,"/tmp/wamstats%02d-%04d.csv",bus->id,dumpnum++);
      ROS_INFO_NAMED("controller","Dumping data to %s",filename);
      if (!owam->recorder.dump(filename)) {
	ROS_INFO("FAILED!!!");
      }
      owam->recorder.reset();
  }
    statcount=0;
  }
#endif // ! OWDSIM

  // Look for any joints that have suddenly changed in value due to
  // a suspected puck firmware bug
  for (int i=1; i<Joint::Jn; ++i) {
    if (bus->jumptime[i] > 0) {
      char timestring[100];
      time_t seconds = bus->jumptime[i] / 1e6;
      int useconds = bus->jumptime[i] - seconds * 1e6;
      strftime(timestring,100,"%T",localtime(&seconds));
	
      ROS_WARN("Motor %d jumped by more than 1000 encoder ticks at time [%s.%06d]",i,timestring,useconds);
      bus->jumptime[i]=0;
    }
  }

  if (owam->jointstraj) {
    return; // still running a trajectory
  }
  
  // look for a recently-completed trajectory
  boost::mutex::scoped_lock lock(wamstate_mutex);
  if ((wamstate.trajectory_queue.size() > 0) &&
      (wamstate.trajectory_queue.front().state !=
       owd_msgs::TrajInfo::state_pending)) {
    // we are not currently executing a trajectory, but if there's one in the
    // queue that's not still pending, then it must have just finished, so
    // move it from the queue to the previous field
    
    wamstate.prev_trajectory = wamstate.trajectory_queue.front();
    char endstr[200];
    strcpy(endstr,"");
    for(int i = Joint::J1; i<=Joint::Jn; i++) {
      snprintf(endstr+strlen(endstr),199-strlen(endstr)," %1.4f",owam->heldPositions[i]);
    }
    if (owam->last_traj_state == Trajectory::DONE) {
      wamstate.prev_trajectory.state = owd_msgs::TrajInfo::state_done;
      ROS_INFO_NAMED("AddTrajectory","Trajectory %s has finished; new reference position is [%s ]",
		     wamstate.prev_trajectory.id.c_str(),endstr);
    } else if (owam->last_traj_state == Trajectory::ABORT) {
      wamstate.prev_trajectory.state = owd_msgs::TrajInfo::state_aborted;
      ROS_WARN_NAMED("AddTrajectory","Trajectory %s has been cancelled/aborted (%s); new reference position is [%s ]",
		     wamstate.prev_trajectory.id.c_str(),
		     owam->last_traj_error,
		     endstr);
    } else {
      ROS_ERROR_NAMED("AddTrajectory","Trajectory %s is done but the last_traj_state was not set; Error may be %s; new reference position is [%s ]",
		     wamstate.prev_trajectory.id.c_str(),
		     owam->last_traj_error,
		     endstr);
    }
    wamstate.trajectory_queue.erase(wamstate.trajectory_queue.begin());
  }

  // now look for any new trajectories to start
  if (trajectory_list.size()==0) {
    // If there are no queued trajectories, then take advantage of this opportunity
    // to look for updated ROS parameters that can't be changed while running
    // trajectories.
    update_xmission_ratio("motor1_transmission_ratio",owam->mN1,42);
    update_xmission_ratio("motor2_transmission_ratio",owam->mN2,28);
    update_xmission_ratio("motor3_transmission_ratio",owam->mN3,28);
    update_xmission_ratio("motor4_transmission_ratio",owam->mN4,18);
    update_xmission_ratio("motor5_transmission_ratio",owam->mN5,10);
    update_xmission_ratio("motor6_transmission_ratio",owam->mN6,10);
    update_xmission_ratio("motor7_transmission_ratio",owam->mN7,15);
    update_xmission_ratio("differential3_ratio"      ,owam->mn3,1.7);
    update_xmission_ratio("differential6_ratio"      ,owam->mn6,1);

    return; // nothing else to do
  }

  // we have a trajectory ready to run
  

  // make sure already holding position
  if (!owam->holdpos || (owam->stiffness < 1.0)) {
    ROS_ERROR("WAM not holding position; cannot start trajectory");
    for (std::vector<OWD::Trajectory *>::iterator it=trajectory_list.begin();
         it != trajectory_list.end(); ++it) {
      delete *it;
    }
    trajectory_list.clear();
    wamstate.trajectory_queue.clear();
    return;
  }

  // check for current position
  JointPos curpoint(nJoints);
  double wampos[nJoints+1];
  // get the held value
  owam->hold_position(wampos,true);
  curpoint.SetFromArray(nJoints,wampos+1);
  JointPos firstpoint(trajectory_list.front()->start_position);
  if (firstpoint != curpoint) {
    ROS_ERROR("Arm position doesn't match queued trajectory start");
    ROS_ERROR("Clearing trajectory queue");
    for (std::vector<OWD::Trajectory *>::iterator it=trajectory_list.begin();
         it != trajectory_list.end(); ++it) {
      delete *it;
    }
    trajectory_list.clear();
    wamstate.trajectory_queue.clear();
    return;
  }

  // load next trajectory
  if (owam->run_trajectory(trajectory_list.front()) == OW_FAILURE) {
    ROS_ERROR("Could not start queued trajectory");
  }
  trajectory_list.erase(trajectory_list.begin());
  if (wamstate.trajectory_queue.size() > 0) {
    wamstate.trajectory_queue.front().state=owd_msgs::TrajInfo::state_active;
  }
}

void WamDriver::update_xmission_ratio(const char *param_name, double &current_value, double nominal_value) {
  static ros::NodeHandle n("~");
  double new_value;
  if (!n.getParamCached(std::string(param_name), new_value)) {
    // nothing new
    return;
  }
  if (new_value != current_value) {
    if ((new_value > nominal_value * 1.1) ||
	(new_value < nominal_value * 0.9)) {
      ROS_WARN("New value for %s=%2.3f is outside the +/- 10%% range of the nominal; ignored.", param_name, new_value);
      return;
    }
    // prevent the control loop from running
    owam->lock("update_xmission_ratio");
    // remember how far away the target position is from the current joint vals
    static double heldPositionsDelta[Joint::Jn+1];
    for (int i=1; i<=Joint::Jn; ++i) {
      heldPositionsDelta[i] = owam->heldPositions[i] - owam->joints[i].q;
    }
    // set the new transmission ratio
    current_value=new_value;
    // recalculate the joint positions using the new ratio
    owam->mpos2jpos();
    for (int i=1; i<=Joint::Jn; ++i) {
      // recalculate the held position based on the new joint positions
      owam->heldPositions[i] = owam->joints[i].q + heldPositionsDelta[i];
      // modify the last position recorded by the velocity computation
      // so that it doesn't think we moved
      owam->previous_joint_val[i] = owam->joints[i].q;
    }

    // let the control loop resume
    owam->unlock();
    ROS_INFO("Using new value for %s = %2.3f",param_name, new_value);
  }
}

void WamDriver::wamservo_callback(const boost::shared_ptr<const owd_msgs::Servo> &servo) {
  ROS_DEBUG_NAMED("servo","Received servo command:");
  if (servo->joint.size() != servo->velocity.size()) {
    ROS_ERROR("Mismatched arrays received in Servo message; ignored");
    return;
  }
  for (unsigned int j=0; j<servo->joint.size(); ++j) {
    if (isinf(servo->velocity[j]) || isnan(servo->velocity[j])) {
      ROS_ERROR("INF or NAN value published to wamservo topic; ignored");
      return;
    }
  }
  for (unsigned int j=0; j<servo->joint.size(); ++j) {
    ROS_DEBUG_NAMED("servo","  Servo joint %d velocity %2.2f",
              servo->joint[j], servo->velocity[j]);
  }
  owam->lock();
  if (owam->jointstraj) {
    ServoTraj *straj = dynamic_cast<ServoTraj *>(owam->jointstraj);
    if (straj) {
      for (unsigned int i=0; i<servo->joint.size(); ++i) {
        straj->SetVelocity(servo->joint[i],servo->velocity[i]);
      }
      std::string trajid = straj->id;
      owam->unlock();
      ROS_DEBUG_NAMED("servo","Updated servo trajectory %s",trajid.c_str());
      return;
    } else {
      owam->unlock();
      ROS_WARN("A Joint Trajectory is still running; velocity command ignored");
      return;
    }
  } else {
    owam->unlock();
#ifdef BUILD_FOR_SEA
    owam->posSmoother.getSmoothedPVA(owam->heldPositions);
#endif // BUILD_FOR_SEA
    std::string trajid = Trajectory::random_id();
    ServoTraj *straj = new ServoTraj(nJoints, trajid, owam->heldPositions+1, &Plugin::_lower_jlimit[0], &Plugin::_upper_jlimit[0]);
    for (unsigned int i=0; i<servo->joint.size(); ++i) {
      straj->SetVelocity(servo->joint[i],servo->velocity[i]);
    }
    if (owam->run_trajectory(straj) != OW_SUCCESS) {
      ROS_ERROR_NAMED("servo","Servo trajectory failed to start");
      return;
    }
    ROS_DEBUG_NAMED("servo","Starting servo trajectory %s",trajid.c_str());
    return;
  }
} 

bool WamDriver::StepJoint(owd_msgs::StepJoint::Request &req,
			  owd_msgs::StepJoint::Response &res) {
#ifdef BUILD_FOR_SEA
  owam->posSmoother.getSmoothedPVA(owam->heldPositions);
#endif // BUILD_FOR_SEA
  std::string trajid = Trajectory::random_id();
  try {
    StepTraj *straj = new StepTraj(trajid, nJoints, req.joint, owam->heldPositions+1, req.radians);
    if (owam->run_trajectory(straj) != OW_SUCCESS) {
      ROS_ERROR("Could not start Servo traj");
      return false;
    }
  } catch (const char *msg) {
    ROS_ERROR("Could not construct StepTraj: %s",msg);
    return false;
  }

  ROS_DEBUG_NAMED("servo","Starting servo trajectory %s",trajid.c_str());
  return true;
}

bool WamDriver::SetGains(owd_msgs::SetGains::Request &req,
			 owd_msgs::SetGains::Response &res) {
  if (isinf(req.gains.kp) || isnan(req.gains.kp) ||
      isinf(req.gains.kd) || isnan(req.gains.kd) ||
      isinf(req.gains.ki) || isnan(req.gains.ki)) {
    ROS_ERROR("INF or NAN passed to SetGains; ignored");
    return false;
  }
  return owam->set_gains(req.joint-1,req.gains);
}

bool WamDriver::ReloadPlugins(owd_msgs::Reset::Request &req,
			      owd_msgs::Reset::Response &res) {
  if (owam->jointstraj) {
    res.reason="Cannot reload plugins while a trajectory is active";
    res.ok=false;
    return true;
  }
  if ((owam->jscontroller != &owam->default_jscontroller) ||
      (owam->new_jscontroller)) {
    res.reason="Cannot reload plugins while using the non-default controller";
    res.ok=false;
    return true;
  }
  unload_plugins();
  std::string plugin_list;
  ros::NodeHandle n("~");
  n.param("owd_plugins",plugin_list,std::string());
  ROS_DEBUG_STREAM("Attempting to load plugins " << plugin_list);
  load_plugins(plugin_list);
  res.ok=true;
  return true;
}

bool WamDriver::SetForceInputThreshold(owd_msgs::SetForceInputThreshold::Request &req,
				       owd_msgs::SetForceInputThreshold::Response &res) {
  try {
    if (!is_in_range(req.direction.x,-1,1) ||
	!is_in_range(req.direction.x,-1,1) ||
	!is_in_range(req.direction.x,-1,1)) {
      ROS_ERROR("Invalid direction vector passed to SetForceInputThreshold; ignored");
      res.ok=false;
      res.reason="Invalid direction vector (one or more values not in the range -1 to 1";
      return true;
    }
    if (!is_in_range(req.force,-128,128)) {
      ROS_ERROR("Force level passed to SetForceInputThreshold is outside the max force sensor range of -128 to 128");
      res.ok=false;
      res.reason="Invalid force level (not in the range -128 to 128)";
      return true;
    }
    if (!is_in_range(req.torques.x,-128,128) ||
	!is_in_range(req.torques.y,-128,128) ||
	!is_in_range(req.torques.z,-128,128)) {
      ROS_ERROR("Invalid torque vector passed to SetForceInputThreshold (one or more torques outside the sensor range of -128 to 128");
      res.ok=false;
      res.reason="Invalid torque vector (at least one value not in range -128 to 128)";
      return true;
    }
  } catch (const char *err) {
    ROS_ERROR("Rejected SetForceInputThreshold call: %s",err);
    res.ok=false;
    res.reason="One or more values were INF or NAN";
    return true;
  }

  R3 direction(req.direction.x, req.direction.y, req.direction.z);
  direction.normalize();
  Trajectory::forcetorque_threshold_direction = direction;
  Trajectory::forcetorque_threshold = req.force;

  R3 torques(req.torques.x, req.torques.y, req.torques.z);
  Trajectory::forcetorque_torque_threshold = torques.normalize();
  Trajectory::forcetorque_torque_threshold_direction = torques;
  res.reason=std::string("");
  res.ok=true;
  return true;
}

bool WamDriver::SetTactileInputThreshold(owd_msgs::SetTactileInputThreshold::Request &req,
				       owd_msgs::SetTactileInputThreshold::Response &res) {
  Trajectory::tactile_pad = req.pad_number;
  Trajectory::tactile_threshold = req.threshold;
  Trajectory::tactile_minimum_readings = req.minimum_readings;
  res.reason=std::string("");
  res.ok=true;
  return true;
}

bool WamDriver::SetTorqueLimits(owd_msgs::SetTorqueLimits::Request &req,
				owd_msgs::SetTorqueLimits::Response &res) {
  if (req.limit.size() > (unsigned int) owam->bus->n_arm_pucks) {
    ROS_WARN("Ignoring SetTorqueLimits attempt: too many arguments");
    res.reason=std::string("Ignoring SetTorqueLimits attempt: too many arguments");
    res.ok=false;
    return true;
  }
  for (unsigned int p=0; p<req.limit.size(); ++p) {
    try {
      if (!is_in_range(req.limit[p],0,Joint::MAX_MECHANICAL_TORQ[p])) {
	ROS_ERROR("Requested torque limit for joint %d exceeds max mechanical limit of %2.2fNm, so all requested torque limits have been rejected",
		  p+1, Joint::MAX_MECHANICAL_TORQ[p]);
	res.ok=false;
	res.reason="One or more requested joint torques exceed max mechanical limits";
	return true;
      }
    } catch (const char *err) {
      ROS_ERROR("Invalid torque sent to SetTorqueLimits: %s",err);
      res.ok=false;
      res.reason="One or more torques were INF or NAN";
      return true;
    }
  }
  for (unsigned int p=0; p<req.limit.size(); ++p) {
    Joint::MAX_SAFE_TORQ[p] = req.limit[p];
  }
  res.reason=std::string("");
  res.ok=true;
  return true;
}

bool WamDriver::SetController(owd_msgs::SetController::Request &req,
			      owd_msgs::SetController::Response &res) {
  JSController *controller = JSController::find_controller(req.name);
  if (controller) {
    if (controller->DOF() != Joint::Jn) {
      res.reason=std::string("controller has wrong DOF");
      res.ok=false;
      return true;
    }
    if (owam->holdpos) {
      // check to make sure that the existing controller is doing a good
      // job at holding the joints close to the targets; if not we don't
      // want to switch controllers and cause the new controller to have to
      // suddenly fix a huge error.
      for (int j=0; j<Joint::Jn; ++j) {
	if (fabs(owam->last_control_position[j] - owam->joints[j+1].q)
	    > 0.05) {
	  // Sometimes the existing controller is not holding position well
	  // because of an external force, so in this case we can allow a
	  // greater error as long as the new torques would be similar to
	  // to the current torques
	  if (fabs(owam->last_control_position[j] - owam->joints[j+1].q)
	      < 0.2) {
	    for (int k=0; k<Joint::Jn; ++k) {
	      try {
		controller->reset(k);
		if (!controller->run(k)) {
		  res.reason = std::string("Could not switch new controller into run state");
		  res.ok=false;
		  return true;
		}
	      } catch (const char *err) {
		res.reason = std::string("Could not switch new controller into run state");
		res.ok=false;
		return true;
	      }
	    }
	    std::vector<double> newtorques = controller->evaluate(Plugin::target_arm_position,
								  Plugin::arm_position,
								  0.002);
	    if (fabs(newtorques[j] - Plugin::pid_torque[j]) < 
		0.30*Joint::MAX_MECHANICAL_TORQ[j]) { /* was 0.15 */
	      continue;
	    } else {
	      char errmsg[300];
	      snprintf(errmsg,300,"New controller torque for joint %d differs too much from existing controller, so rejecting switch due to safety concerns (existing torque=%2.2fNm, new=%2.2fNm)",
		       j+1,
		       Plugin::pid_torque[j],
		       newtorques[j]);
	      res.reason=std::string(errmsg);
	      res.ok=false;
	      return true;
	    }
	  }	    
	  res.reason=std::string("Existing controller is not holding position well; please SetStiffness 0 and try again");
	  res.ok=false;
	  return true;
	}
      }
    }
    for (int j=0; j<Joint::Jn; ++j) {
      controller->reset(j);
      try {
	if (!controller->run(j)) {
	  res.reason = std::string("Could not switch new controller into run state");
	  res.ok=false;
	  return true;
	}
      } catch (const char *err) {
	res.reason = std::string("Could not switch new controller into run state: ");
	res.reason += std::string(err);
	res.ok=false;
	return true;
      }
    }
    owam->lock();
    if (owam->new_jscontroller) {
      res.reason=std::string("Still not finished switching to controller ");
      res.reason += owam->new_jscontroller->name;
      owam->unlock();
      res.ok=false;
      return true;
    }
    if (owam->holdpos) {
      // controller is active, so set up for a gradual switch
      owam->new_jscontroller = controller;
      owam->jscontroller_blend_time=0;
    } else {
      // controller is inactive, so swith immediately
      owam->jscontroller = controller;
    }
    owam->unlock();
    res.reason=std::string("");
    res.ok=true;
    ROS_INFO("Switching to controller %s",controller->name.c_str());
  } else {
    res.reason=std::string("Requested controller has not been defined");
    res.ok=false;
  }
  return true;
}


// storage for static members
CANbus *WamDriver::bus = NULL;
WAM *WamDriver::owam = NULL;


#ifdef BUILD_FOR_SEA
void WamDriver::wamjointtargets_callback(const boost::shared_ptr<const owd_msgs::IndexedJointValues> &jt) {

  if (owam->holdpos == false) {
    ROS_WARN("ignoring wam_joint_targets message while in non-holding mode\n");
    return;
  }

  //ROS_INFO("Received JointTargets command:");

  if (jt->jointIndices.size() != jt->values.size()) {
    ROS_ERROR("Mismatched arrays received in wamjointtargets_callback; ignored");
    return;
  }
  
  owam->posSmoother.setCoarseTarg(jt->jointIndices, jt->values);
  return;
}

void WamDriver::resetSeaCtrl() {
  ROS_INFO("Reset SeaCtrl");

  owam->posSmoother.setReset();
}

void WamDriver::wam_seactrl_settl_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &tl) {

  ROS_INFO("Received set torq limit command:");

  if (tl->jointIndices.size() != tl->values.size()) {
    ROS_ERROR("Mismatched arrays received in wam_seactrl_settl_callback; ignored");
    return;
  }

  if (tl->retainArmState == 0) {
    resetSeaCtrl();
  }

  int n = tl->jointIndices.size();

  for (int i=0; i<n; i++) {
    int index = tl->jointIndices.at(i);
    assert (index >= 0);
    assert (index <= Joint::Jn);
    owam->jointsctrl[index].setTorqLimit(tl->values.at(i));
  }
  // a torque setting can affect all settings, so publish all
  publishAllSeaSettings();
}


void WamDriver::publishCurrentTorqLimits() {
  // build the curtl message
  ROS_INFO("publishCurrentTorqLimits");
  owd_msgs::IndexedJointValues curtl;
  for (int j=Joint::J1; j<=Joint::Jn; j++) {
    curtl.jointIndices.push_back(j);
    curtl.values.push_back( owam->jointsctrl[j].getTorqLimit() );
  }
  pub_wam_seactrl_curtl.publish(curtl);

  return;
}

bool WamDriver::WamRequestSeaCtrlTorqLimit(owd_msgs::WamRequestSeaCtrlTorqLimit::Request &req,
                                           owd_msgs::WamRequestSeaCtrlTorqLimit::Response &res) {
  ROS_INFO("WamRequestSeaCtrlTorqLimit\n");
  publishCurrentTorqLimits();
  return true;
}


void WamDriver::wam_seactrl_setkp_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &kp) {

  ROS_INFO("Received set kp command:");

  if (kp->jointIndices.size() != kp->values.size()) {
    ROS_ERROR("Mismatched arrays received in wam_seactrl_setkp_callback; ignored");
    return;
  }

  if (kp->retainArmState == 0) {
    resetSeaCtrl();
  }

  int n = kp->jointIndices.size();

  for (int i=0; i<n; i++) {
    int index = kp->jointIndices.at(i);
    assert (index >= 0);
    assert (index <= Joint::Jn);
    owam->jointsctrl[index].setKp(kp->values.at(i));
  }
  publishCurrentKp();
}

void WamDriver::publishCurrentKp() {
  ROS_INFO("publishCurrentKp");
  // build the curkp message
  owd_msgs::IndexedJointValues curkp;
  for (int j=Joint::J1; j<=Joint::Jn; j++) {
    curkp.jointIndices.push_back(j);
    curkp.values.push_back( owam->jointsctrl[j].getKp() );
  }
  pub_wam_seactrl_curkp.publish(curkp);

  return;
}

bool WamDriver::WamRequestSeaCtrlKp(owd_msgs::WamRequestSeaCtrlKp::Request &req,
                                    owd_msgs::WamRequestSeaCtrlKp::Response &res) {
  ROS_INFO("WamrequestSeaCtrlKp");
  publishCurrentKp();
  return true;
} 

void WamDriver::wam_seactrl_setkd_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &kd) {

  ROS_INFO("Received set kd command:");

  if (kd->jointIndices.size() != kd->values.size()) {
    ROS_ERROR("Mismatched arrays received in wam_seactrl_setkd_callback; ignored");
    return;
  }

  if (kd->retainArmState == 0) {
    resetSeaCtrl();
  }

  int n = kd->jointIndices.size();

  for (int i=0; i<n; i++) {
    int index = kd->jointIndices.at(i);
    assert (index >= 0);
    assert (index <= Joint::Jn);
    owam->jointsctrl[index].setKd(kd->values.at(i));
  }
  publishCurrentKd();
}

void WamDriver::publishCurrentKd() {
  ROS_INFO("publishCurrentKd");
  // build the curkd message
  owd_msgs::IndexedJointValues curkd;
  for (int j=Joint::J1; j<=Joint::Jn; j++) {
    curkd.jointIndices.push_back(j);
    curkd.values.push_back( owam->jointsctrl[j].getKd() );
  }
  pub_wam_seactrl_curkd.publish(curkd);

  return;
}

bool WamDriver::WamRequestSeaCtrlKd(owd_msgs::WamRequestSeaCtrlKd::Request &req,
                                    owd_msgs::WamRequestSeaCtrlKd::Response &res) {
  ROS_INFO("WamrequestSeaCtrlKd");
  publishCurrentKd();
  return true;
} 


void WamDriver::wam_seactrl_setki_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &ki) {

  ROS_INFO("Received set ki command:");

  if (ki->jointIndices.size() != ki->values.size()) {
    ROS_ERROR("Mismatched arrays received in wam_seactrl_setki_callback; ignored");
    return;
  }

  if (ki->retainArmState == 0) {
    resetSeaCtrl();
  }

  int n = ki->jointIndices.size();

  for (int i=0; i<n; i++) {
    int index = ki->jointIndices.at(i);
    assert (index >= 0);
    assert (index <= Joint::Jn);
    owam->jointsctrl[index].setKi(ki->values.at(i));
  }
  publishCurrentKi();
}

void WamDriver::publishCurrentKi() {
  ROS_INFO("publishCurrentKi");
  // build the curki message
  owd_msgs::IndexedJointValues curki;
  for (int j=Joint::J1; j<=Joint::Jn; j++) {
    curki.jointIndices.push_back(j);
    curki.values.push_back( owam->jointsctrl[j].getKi() );
  }
  pub_wam_seactrl_curki.publish(curki);

  return;
}

bool WamDriver::WamRequestSeaCtrlKi(owd_msgs::WamRequestSeaCtrlKi::Request &req,
                                    owd_msgs::WamRequestSeaCtrlKi::Response &res) {
  ROS_INFO("WamrequestSeaCtrlKi");
  publishCurrentKi();
  return true;
} 


void WamDriver::publishAllSeaSettings() {
  // publish sea settings
  publishCurrentTorqLimits();
  publishCurrentKp();
  publishCurrentKd();
  publishCurrentKi();

}

#endif
};

