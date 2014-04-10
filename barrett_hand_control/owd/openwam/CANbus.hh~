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

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include <iostream>
#include <fstream>
#include <unistd.h>          // usleep
#include <pthread.h>
#include <math.h>            // M_PI
#include <iomanip>
#include <map>

#ifndef OWDSIM

#ifdef ESD_CAN
#include <ntcan.h>
#else // ! ESD_CAN
#include <stdint.h>
#endif // ! ESD_CAN

#ifdef PEAK_CAN
#include <libpcan.h>
#endif  // PEAK_CAN

#endif // OWDSIM

#include <stdint.h>
#ifdef OWD_RT
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/intr.h>
#include <native/pipe.h>
#else // ! OWD_RT
#include <queue>
typedef unsigned long long RTIME; // usually defined in xenomai types.h
#endif // ! OWD_RT

#include <sys/mman.h>

#include "Group.hh"
#include "globals.h"
#include "../openmath/R3.hh"
#include "Butterworth.h"

#include "DataRecorder.cc"

#ifndef __CANBUS_H__
#define __CANBUS_H__

using namespace std;

#define NODE_MIN 1
#define NODE_MAX 15
// it looks like NUM_NODES is 1 too many, since +1 seems to be added elsewhere.
#define NUM_NODES (NODE_MAX - NODE_MIN + 2)

const int MODE_IDLE      = 0;
const int MODE_TORQUE    = 2;
const int MODE_PID       = 3;
const int MODE_VELOCITY  = 4;
const int MODE_TRAPEZOID = 5;

const int CMD_HI = 13;
const int CMD_M  = 19;

class CANstats{
public:
  double cansend_time;
  double canread_sendtime;
  double canread_readtime;
  int    canread_badpackets;

  CANstats() : cansend_time(0.0f),
	       canread_sendtime(0.0f),
	       canread_readtime(0.0f),
	       canread_badpackets(0)
  {}

  void rosprint();
    
};

class CANmsg {
public:
  int32_t nodeid;
  int32_t property;
  int32_t value;

};


class CANbus{
private:
  pthread_t canthread;
  int bus_run;
  int32_t puck_state;
#ifdef PEAK_CAN
#define MAX_FILTERS (5)
  int32_t can_accept[MAX_FILTERS];
  int32_t mask[MAX_FILTERS];
#endif // PEAK_CAN

public:
  CANstats stats;
  bool BH280_installed;
  int32_t id;
  int32_t fw_vers;
  Group groups[NUM_GROUPS+1];
  int32_t* trq;
  double* pos;
  int *rawpos;
  double* jpos;
  double* forcetorque_data;
  double* accelerometer_data;
  double* filtered_forcetorque_data;
  Butterworth<R3> ft_force_filter, ft_torque_filter;
  float* tactile_data;
  bool valid_forcetorque_data, valid_filtered_forcetorque_data;
  int valid_forcetorque_flag;
  bool valid_tactile_data;
  bool tactile_top10;
  Puck *pucks;
  int n_arm_pucks;
  bool simulation;
  char last_error[200];
  int32_t received_position_flags;
  int32_t received_state_flags;
  int hand_motion_state_sequence;
  bool ignore_breakaway_encoders;
  int32_t hsg_value;
  bool squeeze_after_stalling;
  RTIME *jumptime;
  bool *firstupdate;
  int max_safety_torque;
  bool ok;

  static std::map<int,std::string> propname; // reverse number-to-name lookup

#ifndef OWDSIM

bool log_canbus_data;

typedef struct {
  int32_t secs;
  int32_t usecs;
  bool send;
  int32_t msgid;
  int32_t msglen;
  int32_t msgdata[8];
} canio_data;
DataRecorder<canio_data> candata;

  int unread_packets;
  HANDLE handle;

#ifdef OWD_RT
  RT_INTR rt_can_intr;
#endif // OWD_RT

#endif // ! OWDSIM

#ifdef OWD_RT
  RT_PIPE handpipe;
  int handpipe_fd;
#else // ! OWD_RT
  std::queue<CANmsg> hand_command_queue;
  std::queue<CANmsg> hand_response_queue;
#endif // ! OWD_RT

  int load();
  int open();
  int check();
  
  int allow_message(int32_t id, int32_t mask);   
  int wake_puck(int32_t id);
  
  int status(int32_t* nodes);
  int parse(int32_t msgid, uint8_t* msg, int32_t msglen,
	    int32_t* nodeid, int32_t* property, 
	    int32_t* value, int32_t *value2=NULL);
  int compile(int32_t property, int32_t value, uint8_t *msg, int32_t *msglen);
  
  int set_property_rt(int32_t nid, int32_t property, int32_t value, bool check =false, int32_t usecs=200);

  /// Get a property from a puck (call only from control loop)
  /// \param nid Node number
  /// \param property Property number
  /// \param value Pointer to a place to store the value
  /// \param usecs Time to wait for response (defaults to 2000
  ///              if not specified)
  /// \param retries Number of times to resend the request after a timeout
  int get_property_rt(int32_t nid, int32_t property, int32_t* value, int32_t usecs=2000, int32_t retries=0);

  /// Send out a property request, but don't wait for it to return
  /// The main message processing loop will dispatch the return message
  /// to the matching processing function.
  /// \param id Node number
  /// \param property Property number
  int request_property_rt(int32_t id, int32_t property);
  
  int read_rt(int32_t* msgid, uint8_t* msg, int32_t* msglen, int32_t usecs);
  int send_rt(int32_t  msgid, uint8_t* msg, int32_t  msglen, int32_t usecs);
  
  int clear();
  int32_t get_puck_state();
  int set_puck_state_rt();
  int set_puck_group_id(int32_t nid);

  int send_torques_rt();
  int read_positions_rt();
  int extra_bus_commands();
  void send_puck_reset(int32_t low, int32_t high);

  int request_positions_rt(int32_t groupid);
  int request_puck_state_rt(int32_t nodeid);
  int request_hand_state_rt();
  int request_tactile_rt();
  int request_strain_rt();
  int request_forcetorque_rt();
  int request_accelerometer_rt();
  int request_ecminmax_rt(int32_t id);

  int process_positions_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_arm_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_safety_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_hand_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_forcetorque_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_get_property_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);

  int read_forcetorque_rt();
  static int ft_combine(unsigned char msb, unsigned char lsb);

  int read_tactile_rt();
  int configure_tactile_sensors();

  void initPropertyDefs(int32_t firmwareVersion);
  RTIME time_now_ns();

  inline void rosprint_stats() {
    stats.rosprint();
  }


  /// Constructor
  /// \param bus_id numeric identifier of the CANbus device (/dev/can# for
  ///               ESD cards and /dev/pcan# for PEAK cards)
  /// \param number_of_arm_pucks 4 for arm only or 7 for arm + wrist
  ///                            (don't count hand)
  /// \param bh280 set to true if the CANbus hand (model BH280) is installed
  /// \param ft set to true if the CANbus force/torque sensor is installed
  /// \param tactile set to true if the hand has tactile sensors installed
  /// \param log_canbus_data set to true if CANbus traffic should be
  ///               recorded and dumped to file at end of run
  CANbus(int32_t bus_id,
	 int number_of_arm_pucks,
	 bool bh280=false,
	 bool ft=false,
	 bool tactile=false,
	 bool log_canbus_data=false);

  ~CANbus();

  int init();
  int start();
  int stop();
  int run();
  void dump();
  void printpos();
  
  int send_torques(int32_t* mtrq);
  int read_positions(double* mpos);
  //  int read_torques(int32_t* mtrq);
  int send_positions(double* mpos);
  int send_AP(int32_t* apval);
  int emergency_shutdown(int faulttype=-1, int motor=-1);

  // mutex functions that work when compiled with or without Realtime kernel
#ifdef OWD_RT
  inline int mutex_init(RT_MUTEX *mutex) {
    return rt_mutex_create(mutex,NULL);
  }

  inline int mutex_lock(RT_MUTEX *mutex) {
    return rt_mutex_acquire(mutex,TM_INFINITE);
  }

  inline int mutex_trylock(RT_MUTEX *mutex) {
    return rt_mutex_acquire(mutex,TM_NONBLOCK);
  }

  inline int mutex_unlock(RT_MUTEX *mutex) {
    return rt_mutex_release(mutex);
  }
#else // ! OWD_RT
  inline int mutex_init(pthread_mutex_t *mutex) {
    return pthread_mutex_init(mutex,NULL);
  }

  inline int mutex_lock(pthread_mutex_t *mutex) {
    return pthread_mutex_lock(mutex);
  }

  inline int mutex_trylock(pthread_mutex_t *mutex) {
    return pthread_mutex_trylock(mutex);
  }

  inline int mutex_unlock(pthread_mutex_t *mutex) {
    return pthread_mutex_unlock(mutex);
  }
#endif // ! OWD_RT


  // enum {HANDSTATE_UNINIT=0,
  // 	HANDSTATE_DONE,
  // 	HANDSTATE_MOVING,
  // 	HANDSTATE_STALLED};

  int32_t hand_positions[4+1];
  int32_t last_hand_positions[4+1];
  int encoder_changed[4];
  int32_t hand_secondary_positions[4+1];
  int32_t hand_goal_positions[4+1];
  double hand_inner_links[3+1];
  double hand_outer_links[3+1];
  double hand_strain[4+1];
  bool hand_breakaway[3+1];
#ifdef OWD_RT
  RT_MUTEX hand_queue_mutex;
  RT_MUTEX hand_cmd_mutex;
  RT_MUTEX ft_mutex;
#else // ! OWD_RT
  pthread_mutex_t hand_queue_mutex;
  pthread_mutex_t hand_cmd_mutex;
  pthread_mutex_t ft_mutex;
#endif // ! OWD_RT
  int32_t handstate[4];
  int32_t endpoint[4];
  bool apply_squeeze[4];

public:
  double finger_encoder_to_radians(int32_t enc);
  int32_t finger_radians_to_encoder(double radians);
  double spread_encoder_to_radians(int32_t enc);
  int32_t spread_radians_to_encoder(double radians);
  void update_link_positions(unsigned int finger);
  int hand_get_property(int32_t id, int32_t prop, int32_t *val);
  int hand_set_property(int32_t id, int32_t prop, int32_t val);
  int hand_set_state_rt();
  int hand_activate(int32_t *nodes);
  int hand_reset();
  int hand_move(std::vector<double> p);
  int hand_velocity(const std::vector<double> &v);
  int hand_torque(const std::vector<double> &t);
  int hand_relax();
  int hand_get_positions(double &p1, double &p2, double &p3, double &p4);
  int hand_get_inner_links(double &l1, double &l2, double &l3);
  int hand_get_outer_links(double &l1, double &l2, double &l3);
  int hand_get_breakaway(bool &b1, bool &b2, bool &b3);
  int hand_get_strain(double &s1, double &s2, double &s3);
  int hand_get_state(int32_t *state);
  int hand_set_speed(const std::vector<double> &v);

  /* Compliant finger */
  int hand_finger_compliant(const bool enable, const int32_t& strain);
  int hand_compliant_callback(const int& fingerId, const int32_t& strain);
  bool m_compliantFinger;
  int32_t m_compliantStrain;
  int32_t m_compliantDeadband;
  int32_t m_compliantReference;
 
  int send_finger_reset(int32_t id);
  int wait_for_finger_reset(int32_t id);

  int ft_get_data(double *values, double *filtered_values=NULL);
  int ft_get_state();
  int ft_tare();

  int tactile_get_data(float *f1, float *f2, float *f3, float *palm);
  int tactile_set_hires(bool hires);

  int set_limits();
  friend void* canbus_handler(void* argv);

  bool finger_hi_pending[4];
  int hand_puck_mode[4];
  int hand_puck_temp[4];
  int hand_motor_temp[4];

  uint32_t get_property_expecting_id;
  uint32_t get_property_expecting_prop;

  static const int ft_tare_values_to_average = 20;
  int force_tare_values_collected, 
      torque_tare_values_collected;
  double ft_tare_avg[6];

  int32_t hand_initial_torque;
  int32_t hand_sustained_torque;
  double max_cartesian_velocity;
  
  std::vector<unsigned long long> next_encoder_clocktime, last_encoder_clocktime;
};

#endif // __CANBUS_H__
