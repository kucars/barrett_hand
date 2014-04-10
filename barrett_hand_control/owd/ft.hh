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

#include <ros/ros.h>
#include <CANbus.hh>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <owd_msgs/Reset.h>
#include <owd_msgs/ForceState.h>

class FT {
public:
  ros::Publisher pub_ft, pub_filtered_ft, pub_accel, pub_ft_state;
  ros::ServiceServer ss_tare;
  CANbus *bus;

  ros::NodeHandle node;
  geometry_msgs::WrenchStamped ft_vals;
  geometry_msgs::Vector3 accel_vals;
  owd_msgs::ForceState ft_state;

  bool valid_data;

  FT(CANbus *cb);
  ~FT();
  void Pump(const ros::TimerEvent& e);
  bool Publish();
  bool Tare(owd_msgs::Reset::Request &req,
	    owd_msgs::Reset::Response &res);
  
private:
  void AdvertiseAndSubscribe(ros::NodeHandle &n);
  void Unadvertise();
};
  


