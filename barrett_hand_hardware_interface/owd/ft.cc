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

#include "ft.hh"

FT::FT(CANbus *cb) : bus(cb), node("~"), valid_data(false) {
  AdvertiseAndSubscribe(node);
}

FT::~FT() {
  Unadvertise();
}

void FT::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_ft = n.advertise<geometry_msgs::WrenchStamped>("forcetorque", 1);
  pub_ft_state = n.advertise<owd_msgs::ForceState>("forcetorque_state", 1);
  pub_filtered_ft = n.advertise<geometry_msgs::WrenchStamped>("filtered_forcetorque", 1);
  pub_accel = n.advertise<geometry_msgs::Vector3>("accelerometer", 1);
  ss_tare = n.advertiseService("ft_tare",&FT::Tare,this);
}

void FT::Unadvertise() {
  pub_ft.shutdown();
  ss_tare.shutdown();
}

void FT::Pump(const ros::TimerEvent& e) {
  // publish our data
  this->Publish();
}

bool FT::Publish() {
  static double ft_values[6];
  static double ft_filtered_values[6];
  int ft_get_status;
  ft_get_status = bus->ft_get_data(ft_values,ft_filtered_values);
 
  //forcetorque_state message always publishes
  //when sensor is actively saturated, saturated_axes=255
  //When sensor has no issues, saturated_axes=0
  //When sensor has been saturated since latest re-tare:
  //  saturated_axes = integer of bitmap of saturated cells, 000001-111111
  ft_state.header.stamp = ros::Time::now();
  ft_state.saturated_axes = bus->ft_get_state();
  ft_state.wrench.force.x = ft_filtered_values[0];
  ft_state.wrench.force.y = ft_filtered_values[1];
  ft_state.wrench.force.z = ft_filtered_values[2];
  ft_state.wrench.torque.x = ft_filtered_values[3];
  ft_state.wrench.torque.y = ft_filtered_values[4];
  ft_state.wrench.torque.z = ft_filtered_values[5];
  pub_ft_state.publish(ft_state);
  
  if (ft_get_status != OW_SUCCESS) {
    ROS_DEBUG_NAMED("ft","Unable to get data from Force/Torque sensor");
    return false; }

  // set the time
  ft_vals.header.stamp = ros::Time::now();
  // fill in the raw values
  ft_vals.wrench.force.x=ft_values[0];
  ft_vals.wrench.force.y=ft_values[1];
  ft_vals.wrench.force.z=ft_values[2];
  ft_vals.wrench.torque.x=ft_values[3];
  ft_vals.wrench.torque.y=ft_values[4];
  ft_vals.wrench.torque.z=ft_values[5];
  pub_ft.publish(ft_vals);
  // overwrite with the filtered values
  ft_vals.wrench.force.x= ft_filtered_values[0];
  ft_vals.wrench.force.y= ft_filtered_values[1];
  ft_vals.wrench.force.z= ft_filtered_values[2];
  ft_vals.wrench.torque.x=ft_filtered_values[3];
  ft_vals.wrench.torque.y=ft_filtered_values[4];
  ft_vals.wrench.torque.z=ft_filtered_values[5];
  pub_filtered_ft.publish(ft_vals);
  if (bus->accelerometer_data) {
    accel_vals.x = bus->accelerometer_data[0];
    accel_vals.y = bus->accelerometer_data[1];
    accel_vals.z = bus->accelerometer_data[2];
    pub_accel.publish(accel_vals);
  }
  return true;
}
  
bool FT::Tare(owd_msgs::Reset::Request &req,
	      owd_msgs::Reset::Response &res) {
  res.ok=true;
  ROS_DEBUG_NAMED("ft","Taring the F/T sensor");
  if (bus->ft_tare() != OW_SUCCESS) {
    ROS_WARN_NAMED("ft","Unable to tare the sensor");
    res.ok=false;
    res.reason="Unable to tare the sensor";
  }    
  return true;
}

