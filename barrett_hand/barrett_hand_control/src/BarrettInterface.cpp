#include <ros/ros.h>
#include <ros/time.h>
#include <owd_msgs/WAMState.h>
#include <owd_msgs/IndexedJointValues.h>
#include <owd_msgs/WamSetupSeaCtrl.h>
#include "openwam/openwamdriver.h"
#include "bhd280.hh"
#include "ft.hh"
#include "tactile.hh"
#include <sys/mman.h>
#include <sys/resource.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, std::string("owd"));

#ifdef OWD_RT
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("owd: mlockall failed: ");
    return NULL;
  }
#else // OWD_RT
#ifndef OWDSIM
  // increase the process priority so that it can still do effective control
  // on a non-realtime system
  if (setpriority(PRIO_PROCESS,0,-5)) {
    ROS_WARN("Could not elevate process scheduling priority; will be more vulnerable to heartbeat faults on a heavily loaded system");
    ROS_WARN("Please make sure the executable is set to suid root");
  }
#endif // OWDSIM
#endif // OWD_RT


  // read parameters and set wam options

  ros::NodeHandle n("~");
  std::string calibration_filename;
  int canbus_number;
  std::string hand_type;
  bool forcetorque;
  bool modified_j1;
  int pub_freq;  // DEPRECATED
  int wam_pub_freq;
  int hand_pub_freq;
  int ft_pub_freq;
  int tactile_pub_freq;
  n.param("calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
  n.param("canbus_number",canbus_number,0);
  n.param("hand_type",hand_type,std::string("none"));
  n.param("forcetorque_sensor",forcetorque,false);
  n.param("modified_j1",modified_j1,false);
  int wam_freq_default=10;
  int hand_freq_default=10;
 if (n.getParam("publish_frequency",pub_freq)) {
    ROS_WARN("Parameter publish_frequency has been deprecated.  Please use wam_publish_frequency, hand_publish_frequency, ft_publish_frequency, and tactile_publish_frequency.");
    // we'll use the deprecated value to set the defaults for the
    // individual hand and wam frequencies, so that it will only be
    // used if the newer params are not specified
    wam_freq_default=hand_freq_default=pub_freq;
  }
  n.param("wam_publish_frequency",wam_pub_freq,wam_freq_default);
  n.param("hand_publish_frequency",hand_pub_freq,hand_freq_default);
  n.param("ft_publish_frequency",ft_pub_freq,10);
  n.param("tactile_publish_frequency",tactile_pub_freq,10);

  if (wam_pub_freq > 500) {
    ROS_WARN("value of wam_publish_frequency exceeds maximum sensor rate; capping to 500Hz");
    wam_pub_freq = 500;
  }
  if (hand_pub_freq > 40) {
    ROS_WARN("value of hand_publish_frequency exceeds maximum sensor rate; capping to 40Hz");
    hand_pub_freq = 40;
  }
  if (ft_pub_freq > 500) {
    ROS_WARN("value of ft_publish_frequency exceeds maximum sensor rate; capping to 500Hz");
    ft_pub_freq = 500;
  }
  if (tactile_pub_freq > 40) {
    ROS_WARN("value of tactile_publish_frequency exceeds maximum sensor rate; capping to 40Hz");
    tactile_pub_freq = 40;
  }
  
  ROS_DEBUG("Using CANbus number %d",canbus_number);

  int BH_model(0);
  bool tactile(false);
  if (! hand_type.compare(0,3,"280")) {
    BH_model=280;
    if (! hand_type.compare("280+TACT")) {
      ROS_DEBUG("Expecting tactile sensors on this hand");
      tactile=true;
    }
  } else if (! hand_type.compare(0,3,"260")) {
    BH_model=260;
  } else if (! hand_type.compare(0,7,"Robotiq")) {
    BH_model=998;
  } else if (! hand_type.compare(0,29,"darpa_arms_calibration_target")) {
    BH_model=999;
  } else if (hand_type.compare(0,4, "none")) { // note the absence of the !
    ROS_FATAL("Unknown hand type \"%s\"; cannot continue with unknown mass properties",
	      hand_type.c_str());
    exit(1);
  }


}
