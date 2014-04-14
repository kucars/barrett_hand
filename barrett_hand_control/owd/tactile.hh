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
#include <owd_msgs/BHTactile.h>

class Tactile {
public:
  ros::Publisher pub_tactile;
  CANbus *bus;

  ros::NodeHandle node;

  Tactile(CANbus *cb);
  ~Tactile();
  void Pump(const ros::TimerEvent& e);
  bool Publish();
  
private:
  void AdvertiseAndSubscribe(ros::NodeHandle &n);
  void Unadvertise();
  owd_msgs::BHTactile tactile_msg;
};
  


