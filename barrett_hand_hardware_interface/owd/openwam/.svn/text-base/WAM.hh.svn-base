/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2011 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/


#ifndef __WAM_H__
#define __WAM_H__

#include <semaphore.h>
#include <list>

#include "CANbus.hh"
#include "Joint.hh"
#include "Motor.hh"
#include "ControlLoop.hh"
#include "Link.hh"
#include "Kinematics.hh"
#include "Dynamics.hh"

#include "PulseTraj.hh"
#include "SE3Traj.hh"
#include "SE3CtrlPD.hh"
#include "JointsTraj.hh"
#include "ParaJointTraj.hh"
#include "MacJointTraj.hh"
#include "Sigmoid.hh"
#include "JSController.hh"
// #include "JointCtrlPID.hh"
#include "MultiSync.hh"
#include "DataRecorder.cc"
#include "Butterworth.h"

#include <owd_msgs/PIDgains.h>


class WAMstats{
public:
  double trajtime;
  double jsctrltime;
  int safetycount;
  double loopread;
  double loopctrl;
  double loopsend;
  double looptime;
  double slowcount;
  double slowavg;
  double slowmax;
  double slowreadtime;
  double slowctrltime;
  double slowsendtime;
  int missed_reads;
  double dyntime;
  int hitorquecount[7];
  double hitorqueavg[7];

  WAMstats() : trajtime(0.0f),
               jsctrltime(0.0f),
               safetycount(0),
               loopread(0.0f),
               loopctrl(0.0f),
               loopsend(0.0f),
               looptime(0.0f),
               slowcount(0.0f),
               slowavg(0.0f),
	       slowmax(0.0f),
               slowreadtime(0.0f),
               slowctrltime(0.0f),
               slowsendtime(0.0f),
               dyntime(0.0f)
  {
    for (unsigned int i=0; i<7; ++i) {
      hitorquecount[i]=0;
      hitorqueavg[i]=0;
    }
  }

  void rosprint(int rc) const;
  
};


class WAM{
public:
  double mN1;  //joint ratios
  double mN2;
  double mN3;
  double mN4;
  double mN5;
  double mN6;
  double mN7;
  double mn3;
  double mn6;

  static const double M2_MM2 = 1.0/1000000;


#ifdef OWD_RT
  RT_MUTEX rt_mutex;
#else // ! OWD_RT
  pthread_mutex_t mutex;                   // Use to lock the WAM
#endif // ! OWD_RT
  Joint joints[Joint::Jn+1];               // Array of joints
  Motor motors[Motor::Mn+1];               // Array of motors
  OWD::Link links[Link::Ln+1];                  // Array of links
  OWD::Link original_links[Link::Ln+1];         // holds values before extra
                                           //   mass props are set
  OWD::Link sim_links[Link::Ln+1];              // Array of links (simulated)
  OWD::Link L7_with_260_hand,
    L7_with_280_hand,
    L7_with_280FT_hand,
    L7_with_280FT_Flipped_hand,
    L7_with_FT_and_Robotiq_hand,
    L7_with_Robotiq_hand,
    L7_without_hand,
    L7_with_ARMS_calibration_target;
  OWD::Link L4_without_wrist_with_260_hand,
    L4_without_wrist_without_hand;
  double heldPositions[Joint::Jn+1];
  bool check_safety_torques;
  double stall_sensitivity;
  std::vector<double> pid_torq;
  std::vector<double> sim_torq;
  std::vector<double> dyn_torq;
  std::vector<double> traj_torq;
  OWD::Trajectory::TrajControl tc;
 
  SE3 E0n;                                 // forward kinematics transformation

  SE3Traj     *se3traj;                    // Cartesian trajectory
  SE3CtrlPD    se3ctrl;                    // basic Cartesian controller
  OWD::Trajectory *jointstraj;
  int last_traj_state;
  char last_traj_error[500];
  PulseTraj   *pulsetraj;                  // trajectory of joint acceleration pulses
  int safetytorquecount[7];
  double safetytorquesum[7];

  MultiSync *ms;

  DefaultJSController default_jscontroller;
  JSController *jscontroller, *new_jscontroller;
  double jscontroller_blend_time;
  double jscontroller_blend_period;

  //  JointCtrlPID jointsctrl[Joint::Jn+1];    // joint controllers

  bool rec;                                // Are we recording data
  bool wsdyn;                              // Is the WS dynamics turned on?
  bool jsdyn;                              // Is the JS dynamics turned on?
  bool holdpos;                            // Should we hold position?
  bool exit_on_pendant_press;              // once running, should we exit if the user
                                           // presses e-stop or shift-idle?
  double pid_sum;
  int pid_count;
  bool safety_hold;
  bool log_controller_data; // will log joint positions, torques, etc while
                            // holding a position or running a trajectory

  WAMstats stats;
  inline void rosprint_stats() { stats.rosprint(recorder.count); bus->rosprint_stats();}

  int recv_mpos();
  int send_mtrq();

  void mpos2jpos();
  void jpos2mpos();   
  void jtrq2mtrq();
  void update_velocities_or_estimate_positions();
  void update_joint_velocity(unsigned int joint);

  R6 WSControl(double dt);
  void newJSControl_rt(double q_target[], double q[], double dt, double pid_torq[]);
  bool check_for_idle_rt();

  void lock(const char *name="unspecified");
  bool lock_rt(const char *name="unspecified");
  void unlock(const char *name=NULL);

  CANbus* bus;                             // pointer to the CAN bus
  OWD::ControlLoop ctrl_loop;            // control loop object

  WAM(CANbus* cb, int BH_model, bool forcetorque, bool flipped_hand, bool tactile,
      bool log_ctrl_data=false);
  ~WAM();
 
  int init();                       // initialise the WAM
  void dump();                      // print information of the WAM

  void printjpos();                 // print joints positions
  void printmtrq();                 // print motor torques

  int start();                      // start the control loop
  void stop();                      // stop the control loop
  void newcontrol_rt(double dt);          // main control function
  double enforce_jointtorque_limits(double t, int j);
  bool safety_torques_exceeded(std::vector<double>); // check pid torqs against thresholds

  int  set_targ_jpos(double* pos);          // set the target joint positions online 
  int  set_jpos(double pos[]);   // set the joint positions offline
  int  set_joint_offsets(double offsets[]); // set fixed offset for each joint
  bool set_gains(int joint, owd_msgs::PIDgains &gains);
  bool get_gains(std::vector<owd_msgs::PIDgains> &gains);

  void get_current_data(double *pos, double *trq, double *nettrq, 
			double *sim_torq=NULL, double *traj_torq=NULL);
                  // get the joint positions, torques, net torques, and
                  // simulated torques (any pointer can be NULL)
  void get_abs_positions(double *abs_pos);
  void get_jtrq(double trq[Joint::Jn+1]);   // get the joint torques
  void get_net_jtrq(double trq[Joint::Jn+1]); // get the net joint torques

  int hold_position(double jval[] = NULL, bool grab_lock=true); // call to hold current position.
                                            // returns position if pointer is supplied.
  void release_position(bool grab_lock=true
);          // call to release current position from PID control
  void set_stiffness(float stiffness); // how "hard" to hold position
  void move_sigmoid(const SE3& E0ns);
  void move_sigmoid(const double q[Joint::Jn+1]);
  void move_trapezoid(const SE3& E0ns);
  void move_trapezoid(const double q[Joint::Jn+1]);

  int run_trajectory(OWD::Trajectory *traj);  // start a trajectory
  int pause_trajectory();
  int resume_trajectory();
  int cancel_trajectory();

  bool& wsdynamics(){return wsdyn;} // use to set the dynamics
  bool& jsdynamics(){return jsdyn;} // use to set the dynamics
  
  bool& record(){return rec;}       // use to set the recording
  SE3 FK(){  SE3 e0n; lock();   e0n=E0n;   unlock();   return e0n;  }

  friend ostream& operator << (ostream& s, WAM& wam);

  typedef enum {MOTORS_OFF,MOTORS_IDLE,MOTORS_ACTIVE} motor_state_t;
  motor_state_t motor_state;

  float stiffness;
  DataRecorder<double> recorder;

  int BH_model;
  bool ForceTorque;
  bool FlippedHand;
  bool Tactile;
  SE3 SE3_endpoint; // result of forward kinematics calculation
  std::vector<double> last_control_position;
  bool slip_joints_on_high_torque;

  double previous_joint_val[Joint::Jn+1];
  unsigned long long previous_encoder_clocktime[Joint::Jn+1];
  std::vector<Butterworth<double> *> velocity_filter;
  double arm_velocity[Joint::Jn+1];
  R3 elbow_vel, endpoint_vel;
  double barrett_endpoint_vel;
  double vel_damping_gain;
};

#endif

