/***********************************************************************

  Copyright 2010 Carnegie Mellon University and Intel Corporation
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

#include <ros/ros.h>
#include <CANbus.hh>
#include <tf/transform_broadcaster.h>
#include <owd_msgs/BHState.h>
#include <owd_msgs/MoveHand.h>
#include <owd_msgs/ResetHand.h>
#include <owd_msgs/ResetFinger.h>
#include <owd_msgs/GetDOF.h>
#include <owd_msgs/RelaxHand.h>
#include <owd_msgs/SetHandProperty.h>
#include <owd_msgs/GetHandProperty.h>
#include <owd_msgs/SetSpeed.h>
#include <owd_msgs/SetHandTorque.h>
#include <owd_msgs/SetFingerCompliant.h>

class BHD_280 {
public:
  ros::Publisher
    pub_handstate;
  ros::ServiceServer 
    ss_gethanddof,
    ss_movehand,
    ss_resethand,
    ss_resethandquick,
    ss_resetfinger,
    ss_sethandprop,
    ss_gethandprop,
    ss_relaxhand,
    ss_setspeed,
    ss_sethandtorque,
    ss_setfingercompliant;

  ros::NodeHandle node;
  CANbus *bus;
  tf::TransformBroadcaster *tf_broadcaster;
  tf::Transform finger_link1_base[3];
  tf::Transform finger_link2_base, finger_link3_base;

  owd_msgs::BHState bhstate;
  double max_velocity;

  BHD_280(CANbus *cb);
  ~BHD_280();
  void Pump(ros::TimerEvent const& e);
  bool Publish();
  bool GetDOF(owd_msgs::GetDOF::Request &req,
	      owd_msgs::GetDOF::Response &res);
  bool RelaxHand(owd_msgs::RelaxHand::Request &req,
		 owd_msgs::RelaxHand::Response &res);
  bool ResetHand(owd_msgs::ResetHand::Request &req,
		 owd_msgs::ResetHand::Response &res);
  bool ResetHandQuick(owd_msgs::ResetHand::Request &req,
		 owd_msgs::ResetHand::Response &res);
  bool ResetFinger(owd_msgs::ResetFinger::Request &req,
		   owd_msgs::ResetFinger::Response &res);
  bool MoveHand(owd_msgs::MoveHand::Request &req,
		owd_msgs::MoveHand::Response &res);
  bool SetHandProperty(owd_msgs::SetHandProperty::Request &req,
		owd_msgs::SetHandProperty::Response &res);
  bool GetHandProperty(owd_msgs::GetHandProperty::Request &req,
		owd_msgs::GetHandProperty::Response &res);
  bool SetSpeed(owd_msgs::SetSpeed::Request &req,
		owd_msgs::SetSpeed::Response &res); 
  bool SetHandTorque(owd_msgs::SetHandTorque::Request &req,
		     owd_msgs::SetHandTorque::Response &res); 
  bool SetFingerCompliant(owd_msgs::SetFingerCompliant::Request &req,
		                  owd_msgs::SetFingerCompliant::Response &res); 

private:
  void AdvertiseAndSubscribe(ros::NodeHandle &n);
  void GetParameters(ros::NodeHandle &n);
  void SetPuckValues();
  void Unadvertise();
  void createT(double a, double alpha, double d, double theta, double result[4][4]);
};
  


