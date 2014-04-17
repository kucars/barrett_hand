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

#include "PulseTraj.hh"
#include <ros/ros.h>

PulseTraj::PulseTraj(double pulse_accel_in, int pulse_duration_ticks_in, int pulse_switch_ticks_in, int pulse_joint_i_in)
{
    _pulseok = false;
    _pulse_counter = 0;
    _record_counter = 0;

    if(pulse_joint_i_in < 0 || pulse_joint_i_in > Joint::Jn){
      ROS_WARN("PulseTraj Error - Joint index out of bounds!");
      return;
    }
    else if( pulse_switch_ticks_in > pulse_duration_ticks_in){
      ROS_WARN("PulseTraj Error - Switching time is greater than duration");
      return;
    }
    else if(pulse_duration_ticks_in == 0){
      ROS_WARN("PulseTraj Error - Pulse duration cannot be 0");
      return;
    }
    else if(fabs((float)pulse_accel_in) > _pulse_accel_limit){
        if (pulse_accel_in > 0.0) {
            pulse_accel_in = _pulse_accel_limit;
        } else {
            pulse_accel_in = 0 - _pulse_accel_limit;
        }
        ROS_WARN("PulseTraj Error - Acceleration capped to limit of %f rad/s",(float)pulse_accel_in);
    }

    pulse_accel =  pulse_accel_in;
    pulse_duration_ticks = pulse_duration_ticks_in;
    pulse_switch_ticks = pulse_switch_ticks_in;
    pulse_joint_i = pulse_joint_i_in;
    pulse_pre_record_ticks = 200;
    pulse_post_record_ticks = 200;

    int vecsize = pulse_duration_ticks + pulse_pre_record_ticks + pulse_post_record_ticks;
    posvals.resize(vecsize);
    velvals.resize(vecsize);
    accelvals.resize(vecsize);
    trqvals.resize(vecsize);
    for(unsigned int i = 0; i < posvals.size(); i++) {
        posvals[i].resize(Joint::Jn);
        velvals[i].resize(Joint::Jn);
        accelvals[i].resize(Joint::Jn);
        trqvals[i].resize(Joint::Jn);
    }
    _pulseok = true;
    if (pulse_joint_i_in == 0) {
        // joint==0 is a flag to just record positions without applying a pulse
        done=true;
    } else {
        done = false;
    }
    record = true;
}


bool PulseTraj::GetMotion(double qd[], double qdd[])
{
    if(!PulseOk()){
      ROS_WARN("PulseTraj Error - Pulse not initialized, cannot give accelerations");
        return false;
    }

    if(_pulse_counter > pulse_duration_ticks + pulse_pre_record_ticks + pulse_post_record_ticks){
      ROS_WARN("PulseTraj Error - Pulse counter running over");
        return false;
    }
    
    if ((_pulse_counter >= pulse_pre_record_ticks) &&
        (_pulse_counter < pulse_pre_record_ticks + pulse_duration_ticks)) {
        // we're in the middle region, so apply the pulse
        if(pulse_switch_ticks == 0) {
            qdd[pulse_joint_i] = pulse_accel; //apply the acceleration
            qd[pulse_joint_i] = pulse_accel *   // velocity = accel * time
                (_pulse_counter - pulse_pre_record_ticks) // time
                / 500.0;                                  // ticks per sec
        } else if(_pulse_counter < pulse_pre_record_ticks + pulse_switch_ticks) {
            qdd[pulse_joint_i] = pulse_accel;
            qd[pulse_joint_i] = pulse_accel *   // velocity = accel * time
                (_pulse_counter - pulse_pre_record_ticks) // time
                / 500.0;                                  // ticks per sec
        } else {
            qdd[pulse_joint_i] = -pulse_accel;
            qd[pulse_joint_i] = pulse_accel *   // velocity = accel * time
                (pulse_switch_ticks - // accel section minus decel time
                 (_pulse_counter -pulse_pre_record_ticks -pulse_switch_ticks))
                / 500.0;                                  // ticks per sec
        }
    }

    _pulse_counter++;

    if(_pulse_counter >= pulse_pre_record_ticks + pulse_duration_ticks){
        _pulse_counter = 0;
        done = true;
        return false;
    }

    return true;

}

bool PulseTraj::RecordPositions(double q[])
{
    static double old_vel[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    static double old_accel[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    if(!PulseOk()){
      ROS_WARN("PulseTraj Error - Pulse not initialized, cannot record positions");
        return false;
    }
    if(!record)
        return false;

    for(int i = 0; i < Joint::Jn;i++) {
        posvals[_record_counter][i] = q[i+1];
        if (_record_counter>0) {
            double vel = (q[i+1] - posvals[_record_counter-1][i] + 3.0*old_vel[i])/4.0;
            old_vel[i] = vel;
            velvals[_record_counter][i] = vel * 500.0;
        } else {
            velvals[_record_counter][i] = 0.0;
            old_vel[i] = 0.0;
            old_accel[i] = 0.0;
        }
        if (_record_counter>1) {
            double accel = (velvals[_record_counter][i] - velvals[_record_counter-1][i] + 3.0*old_accel[i])/4.0;
            old_accel[i] = accel;
            accelvals[_record_counter][i] = accel * 500.0;
        } else {
            accelvals[_record_counter][i] = 0.0;
        }
    }
    _record_counter++;
    if (_record_counter >= pulse_pre_record_ticks + pulse_duration_ticks + pulse_post_record_ticks) {
        record = false;
    }
    return true;
}

bool PulseTraj::RecordTorques(double trq[])
{
    if(!PulseOk()){
      ROS_WARN("PulseTraj Error - Pulse not initialized, cannot record torques");
        return false;
    }
    if(!record)
        return false;

    for(int i = 0; i < Joint::Jn;i++) {
        trqvals[_record_counter][i] = trq[i+1];
    }
    return true;
}
