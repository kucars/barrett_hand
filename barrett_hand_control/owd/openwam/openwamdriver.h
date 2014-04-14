/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
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

#ifndef OPENWAMDRIVER_H
#define OPENWAMDRIVER_H

#include <list>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <owd_msgs/AddTrajectory.h>
#include <owd_msgs/AddTimedTrajectory.h>
#include <owd_msgs/DeleteTrajectory.h>
#include <owd_msgs/CancelAllTrajectories.h>
#include <owd_msgs/PauseTrajectory.h>
#include <owd_msgs/ReplaceTrajectory.h>
#include <owd_msgs/MassProperties.h>
#include <owd_msgs/GetStiffness.h>
#include <owd_msgs/SetStiffness.h>
#include <owd_msgs/SetJointStiffness.h>
#include <owd_msgs/SetJointOffsets.h>
#include <owd_msgs/SetSpeed.h>
#include <owd_msgs/GetSpeed.h>
#include <owd_msgs/SetExtraMass.h>
#include <owd_msgs/SetStallSensitivity.h>
#include <owd_msgs/GetStallSensitivity.h>
#include <owd_msgs/WAMState.h>
#include <owd_msgs/WAMInternals.h>
#include <owd_msgs/GetDOF.h>
#include <owd_msgs/Servo.h>
#include <owd_msgs/Reset.h>
#include <owd_msgs/SetForceInputThreshold.h>
#include <owd_msgs/SetTactileInputThreshold.h>
#include <owd_msgs/SetTorqueLimits.h>
#include <owd_msgs/SetController.h>
#include <owd_msgs/CalibrateJoints.h>
#include <owd_msgs/StepJoint.h>
#include <owd_msgs/SetGains.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#ifdef BUILD_FOR_SEA
  #include <owd_msgs/WamRequestSeaCtrlTorqLimit.h>
  #include <owd_msgs/WamRequestSeaCtrlKp.h>
  #include <owd_msgs/WamRequestSeaCtrlKd.h>
  #include <owd_msgs/WamRequestSeaCtrlKi.h>
#endif

#include "Joint.hh"
#include "TrajType.hh"

// forward declaration of a few classes we keep pointers to, 
// so that we don't have to include their .hh files right now
#include "WAM.hh"
class CANbus;

namespace OWD {
  class Trajectory;

  class WamDriver
{
public:
  /// Constructor
  /// \param canbus_number numeric suffix for CAN bus device (/dev/can# for
  ///                      ESD cards or /dev/pcan# for PEAK cards)
  /// \param bh_model 260 for the serial hand, 280 for the CANbus hand,
  ///                 or 0 for no hand
  /// \param forcetorque true if the force/torque sensor is installed
  /// \param tactile true if the tactile sensors are installed
  ///                (BH model 280 only)
  WamDriver(int canbus_number, int bh_model, bool forcetorque, bool tactile);

    ~WamDriver();

    bool Init(const char *joint_calibration_file);

    void Pump(const ros::TimerEvent& e);
    
    bool Publish();

    void Update();

    bool AddTrajectory(Trajectory *traj, std::string &failure_reason);

    bool AddTrajectory(owd_msgs::AddTrajectory::Request &req,
                       owd_msgs::AddTrajectory::Response &res);
    bool AddTimedTrajectory(owd_msgs::AddTimedTrajectory::Request &req,
                       owd_msgs::AddTimedTrajectory::Response &res);
    bool DeleteTrajectory(owd_msgs::DeleteTrajectory::Request &req,
                          owd_msgs::DeleteTrajectory::Response &res);
    bool CancelAllTrajectories(owd_msgs::CancelAllTrajectories::Request &req,
                          owd_msgs::CancelAllTrajectories::Response &res);
    bool PauseTrajectory(owd_msgs::PauseTrajectory::Request &req,
                         owd_msgs::PauseTrajectory::Response &res);
    bool ReplaceTrajectory(owd_msgs::ReplaceTrajectory::Request &req,
                           owd_msgs::ReplaceTrajectory::Response &res);
    bool GetStiffness(owd_msgs::GetStiffness::Request &req,
                      owd_msgs::GetStiffness::Response &res);
    bool SetStiffness(owd_msgs::SetStiffness::Request &req,
                      owd_msgs::SetStiffness::Response &res);
    bool SetJointStiffness(owd_msgs::SetJointStiffness::Request &req,
                      owd_msgs::SetJointStiffness::Response &res);
    bool SetJointOffsets(owd_msgs::SetJointOffsets::Request &req,
                      owd_msgs::SetJointOffsets::Response &res);
    bool SetSpeed(owd_msgs::SetSpeed::Request &req,
                  owd_msgs::SetSpeed::Response &res);
    bool GetSpeed(owd_msgs::GetSpeed::Request &req,
		  owd_msgs::GetSpeed::Response &res);
    bool SetExtraMass(owd_msgs::SetExtraMass::Request &req,
		      owd_msgs::SetExtraMass::Response &res);
    bool SetStallSensitivity(owd_msgs::SetStallSensitivity::Request &req,
			     owd_msgs::SetStallSensitivity::Response &res);
    bool GetStallSensitivity(owd_msgs::GetStallSensitivity::Request &req,
			     owd_msgs::GetStallSensitivity::Response &res);
    bool GetDOF(owd_msgs::GetDOF::Request &req,
                owd_msgs::GetDOF::Response &res);
    bool CalibrateJoints(owd_msgs::CalibrateJoints::Request &req,
			 owd_msgs::CalibrateJoints::Response &res);
    bool StepJoint(owd_msgs::StepJoint::Request &req,
		   owd_msgs::StepJoint::Response &res);
    bool SetGains(owd_msgs::SetGains::Request &req,
		  owd_msgs::SetGains::Response &res);
    bool ReloadPlugins(owd_msgs::Reset::Request &req,
		       owd_msgs::Reset::Response &res);
    bool SetForceInputThreshold(owd_msgs::SetForceInputThreshold::Request &req,
				owd_msgs::SetForceInputThreshold::Response &res);
    bool SetTactileInputThreshold(owd_msgs::SetTactileInputThreshold::Request &req,
				owd_msgs::SetTactileInputThreshold::Response &res);
    bool SetTorqueLimits(owd_msgs::SetTorqueLimits::Request &req,
			 owd_msgs::SetTorqueLimits::Response &res);

    bool SetController(owd_msgs::SetController::Request &req,
		       owd_msgs::SetController::Response &res);

    void AdvertiseAndSubscribe(ros::NodeHandle &n);


    owd_msgs::AddTrajectory::Response AddTrajectory(
						   owd_msgs::AddTrajectory::Request *);

    void update_xmission_ratio(const char *param_name, double &current_value, double nominal_value);
    void wamservo_callback(const boost::shared_ptr<const owd_msgs::Servo> &message);
    void MassProperties_callback(const boost::shared_ptr<const owd_msgs::MassProperties> &message);

    inline void SetModifiedJ1(bool mj1) {modified_j1 = mj1;}

#ifdef BUILD_FOR_SEA
    void wamjointtargets_callback(const boost::shared_ptr<const owd_msgs::IndexedJointValues> &message);

    void resetSeaCtrl();

    void wam_seactrl_settl_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &message);

    void publishCurrentTorqLimits();
    bool WamRequestSeaCtrlTorqLimit(owd_msgs::WamRequestSeaCtrlTorqLimit::Request &req,
                                    owd_msgs::WamRequestSeaCtrlTorqLimit::Response &res);

    void wam_seactrl_setkp_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKp();
    bool WamRequestSeaCtrlKp(owd_msgs::WamRequestSeaCtrlKp::Request &req,
                             owd_msgs::WamRequestSeaCtrlKp::Response &res);

    void wam_seactrl_setkd_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKd();
    bool WamRequestSeaCtrlKd(owd_msgs::WamRequestSeaCtrlKd::Request &req,
                             owd_msgs::WamRequestSeaCtrlKd::Response &res);

    void wam_seactrl_setki_callback(const boost::shared_ptr<const owd_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKi();
    bool WamRequestSeaCtrlKi(owd_msgs::WamRequestSeaCtrlKi::Request &req,
                             owd_msgs::WamRequestSeaCtrlKi::Response &res);

    void publishAllSeaSettings();
#endif


private:
    Trajectory *BuildTrajectory(owd_msgs::JointTraj &jt);
    owd_msgs::WAMState wamstate;
    owd_msgs::WAMInternals waminternals;
    owd_msgs::Servo servocmd;
    double gravity_comp_value;
    double wamhome[8];
    double min_accel_time;

    bool discard_movements;

    // internal structures
    char *joint_calibration_file;
    unsigned int nJoints;
    int32_t puck_offsets[Joint::Jn+1];
    JointPos desiredJointPositions, vLastCommand;
    struct timeval trajstarttime;
    bool intraj;
    std::vector<Trajectory *> trajectory_list;
    char last_trajectory_error[200];
    int BH_model; /// model number of the hand, either 260, 280, or 0 (no hand)
    bool ForceTorque; /// whether the Force/Torque sensor is installed
    bool Tactile;  /// whether the Tactile sensors are installed (280 hand only)
    bool log_controller_data;
    typedef pair<void *,bool (*)()> PluginPointers;
    std::vector<PluginPointers> loaded_plugins;

    // update internal structures
    void resetDesiredJointPositions(void);
    void resetTrajectory();
    void queueJointPositions(void);

    int QueueJointPositions();
    void calibrate_wam_mass_model();
    void calibrate_joint_angles();
    void write_pulse_data();
    void set_home_position();
    void start_control_loop();
    void stop_control_loop();
    bool verify_home_position();
    bool move_joint(int joint, double newpos, double velocity);
    bool move_until_stop(int joint, double stop, double limit, double velocity,
			 double &orig_joint_pos);

    int get_puck_offset(int puckid,int32_t *mech = NULL,int32_t *apout = NULL);
    void save_joint_offset(double jointval, double *offset);
    int get_joint_num();
    double get_nearest_joint_value(double jointval, double tolerance);
    void apply_joint_offsets(double *joint_offsets);
    TrajType ros2owd_traj (owd_msgs::JointTraj &jt);
    void get_transmission_ratios();
    void set_transmission_ratios();
    void load_plugins(std::string plugin_list);
    void unload_plugins();

 public: // make this public so that it can be shared with BHD_280
    static CANbus *bus;
    static WAM *owam;
    bool running;

 private:
    boost::mutex wamstate_mutex;
    boost::mutex plugin_mutex;
    bool modified_j1;

    ros::Publisher
      pub_wamstate,
      pub_waminternals;

    ros::Subscriber
      sub_wamservo,
      sub_wam_joint_targets,
      sub_MassProperties;

    ros::ServiceServer 
      ss_AddTrajectory,
      ss_AddTimedTrajectory,
      ss_GetStiffness,
      ss_SetStiffness,
      ss_SetJointStiffness,
      ss_SetJointOffsets,
      ss_DeleteTrajectory, 
      ss_CancelAllTrajectories,
      ss_PauseTrajectory,
      ss_ReplaceTrajectory,
      ss_SetSpeed,
      ss_GetSpeed,
      ss_SetExtraMass,
      ss_SetStallSensitivity,
      ss_GetStallSensitivity,
      ss_GetArmDOF,
      ss_CalibrateJoints,
      ss_StepJoint,
      ss_SetGains,
      ss_ReloadPlugins,
      ss_SetForceInputThreshold,
      ss_SetTactileInputThreshold,
      ss_SetTorqueLimits,
      ss_SetController;

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform wam_tf_base[7];
 
#ifdef BUILD_FOR_SEA
    ros::Publisher
      pub_wam_seactrl_curtl,
      pub_wam_seactrl_curkp,
      pub_wam_seactrl_curkd,
      pub_wam_seactrl_curki;

    ros::Subscriber 
      sub_wam_seactrl_settl,
      sub_wam_seactrl_setkp,
      sub_wam_seactrl_setkd,
      sub_wam_seactrl_setki;

    ros::ServiceServer
      ss_WamRequestSeaCtrlTorqLimit,
      ss_WamRequestSeaCtrlKp,
      ss_WamRequestSeaCtrlKd,
      ss_WamRequestSeaCtrlKi;

#endif // BUILD_FOR_SEA

    friend class PositionCommand;
    friend class TrajectoryCommand;
    friend class Trajectory;
};
};
#endif //  OPENWAMDRIVER_H
