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

#include "tactile.hh"

Tactile::Tactile(CANbus *cb) : bus(cb), node("~") {
  tactile_msg.finger1.resize(24);
  tactile_msg.finger2.resize(24);
  tactile_msg.finger3.resize(24);
  tactile_msg.palm.resize(24);
  AdvertiseAndSubscribe(node);
}

Tactile::~Tactile() {
  Unadvertise();
}

void Tactile::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_tactile = n.advertise<owd_msgs::BHTactile>("tactile", 40);
}

void Tactile::Unadvertise() {
  pub_tactile.shutdown();
}

void Tactile::Pump(const ros::TimerEvent& e) {
  if (bus->tactile_get_data(&tactile_msg.finger1[0],
			    &tactile_msg.finger2[0],
			    &tactile_msg.finger3[0],
			    &tactile_msg.palm[0]) != OW_SUCCESS) {
    ROS_WARN_NAMED("tactile","Unable to get data from Tactile sensors");
    return;
  }
  this->Publish();
}

bool Tactile::Publish() {
  pub_tactile.publish(tactile_msg);
  return true;
}
  
