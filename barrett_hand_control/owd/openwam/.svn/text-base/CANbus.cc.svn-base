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


#include "CANbus.hh"
#include "CANdefs.hh"
#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>



CANbus::CANbus(int32_t bus_id, int number_of_arm_pucks, bool bh280, bool ft, bool tactile, bool log_cb_data) : 
  puck_state(-1),BH280_installed(bh280),id(bus_id),fw_vers(0),trq(NULL),
  pos(NULL), rawpos(NULL), jpos(NULL), forcetorque_data(NULL), accelerometer_data(NULL),
  filtered_forcetorque_data(NULL),
  ft_force_filter(2,10.0), ft_torque_filter(2,10.0),
  tactile_data(NULL),
  valid_forcetorque_data(false), valid_filtered_forcetorque_data(false),
  valid_forcetorque_flag(-1), 
  valid_tactile_data(false),
  tactile_top10(false), pucks(NULL),n_arm_pucks(number_of_arm_pucks),
  simulation(false), received_position_flags(0), received_state_flags(0),
  hand_motion_state_sequence(0),
  ignore_breakaway_encoders(true),
  hsg_value(0),
  squeeze_after_stalling(false),
  jumptime(NULL),
  firstupdate(NULL),
  max_safety_torque(0),
  ok(true),
  log_canbus_data(log_cb_data),
  candata(0),
  unread_packets(0),
  get_property_expecting_id(0),
  get_property_expecting_prop(0),
  force_tare_values_collected(0),
  torque_tare_values_collected(0),
  hand_initial_torque(2200),
  hand_sustained_torque(1100),
  next_encoder_clocktime(15),
  last_encoder_clocktime(15),
  m_compliantFinger(false),
  m_compliantStrain(0.0),
  m_compliantDeadband(0.0),
  m_compliantReference(0.0)
{

  // hand_queue_mutex is used to manage access to the hand command/response queues
  mutex_init(&hand_queue_mutex);

  // hand_cmd_queue is used to prevent trouble with multiple ROS service calls occurring at once.  it
  // makes sure that the response you get from the queue corresponds to the command you sent.
  mutex_init(&hand_cmd_mutex);

  mutex_init(&ft_mutex); // keeps the filters safe

  for (unsigned int i=0; i<4; ++i) {
    handstate[i] = HANDSTATE_UNINIT;
  }
  hand_positions[1]=hand_positions[2]=hand_positions[3]=hand_positions[4]=0;
  last_hand_positions[1]=last_hand_positions[2]=last_hand_positions[3]=last_hand_positions[4]=0;
  hand_secondary_positions[1]=hand_secondary_positions[2]=hand_secondary_positions[3]=hand_secondary_positions[4]=0;
  // initialize the inner and outer links to crazy values, so that we
  // can tell if they've been set
  hand_inner_links[1]=hand_inner_links[2]=hand_inner_links[3]=-12345;
  hand_outer_links[1]=hand_outer_links[2]=hand_outer_links[3]=-12345;
  encoder_changed[0]=encoder_changed[1]=encoder_changed[2]=encoder_changed[3] = 0;
  apply_squeeze[0]=apply_squeeze[1]=apply_squeeze[2]=apply_squeeze[3] = false;
  finger_hi_pending[0]=finger_hi_pending[1]=finger_hi_pending[2]=finger_hi_pending[3] = false;
  hand_puck_mode[0]=hand_puck_mode[1]=hand_puck_mode[2]=hand_puck_mode[3]=-1;
  hand_puck_temp[0]=hand_puck_temp[1]=hand_puck_temp[2]=hand_puck_temp[3]=-1;
  hand_motor_temp[0]=hand_motor_temp[1]=hand_motor_temp[2]=hand_motor_temp[3]=-1;
  // the fourth finger finger puck also returns a strain value when we 
  // ask group 5 for strain, but it is meaningless and is not used
  // outside of OWD, so even though we have a four position in the array it
  // is not necessary to initialize it.
  hand_strain[1]=hand_strain[2]=hand_strain[3]=0.0f;
  hand_breakaway[1]=hand_breakaway[2]=hand_breakaway[3]=false;

#ifdef OWD_RT
  int err = rt_pipe_create(&handpipe,"HANDPIPE",P_MINOR_AUTO,0);
  if (err) {
    ROS_ERROR("CANbus::CANbus: Could not create RT message pipe for hand communications: %d",err);
    throw OW_FAILURE;
  }
  handpipe_fd = ::open("/proc/xenomai/registry/native/pipes/HANDPIPE",O_RDWR);
  if (handpipe_fd < 0) {
    ROS_ERROR("CANbus::Canbus: Could not open user-side of RT message pipe: %d",errno);
    throw OW_FAILURE;
  }
#endif // OWD_RT

  pucks = new Puck[n_arm_pucks+1];
  trq = new int32_t[n_arm_pucks+1];
  pos = new double[n_arm_pucks+1];
  rawpos = new int[n_arm_pucks+1];
  jpos = new double[n_arm_pucks+1];
  jumptime = new RTIME[n_arm_pucks+1];
  firstupdate = new bool[n_arm_pucks+1];
  if (ft) {
    forcetorque_data = new double[6];
    filtered_forcetorque_data = new double[6];
    accelerometer_data = new double[6];
    for (int i=0; i<6; ++i) {
      ft_tare_avg[i]=0;
    }
  }
  if (tactile) {
    // allocate with calloc so that the values are initialized to zero
    tactile_data = (float *) calloc(96, sizeof(float));
  }
  if (log_canbus_data) {
    candata.resize(100000);
  }
  for(int p=1; p<=n_arm_pucks; p++){
    pos[p] = 0.0;
    trq[p] = 0;
    pucks[p].ID = p;
    pucks[p].motor_id = p;
    if (p<5) { // 4-DOF
      pucks[p].group_id = 1;
      pucks[p].group_order = p;
    } else if (p<8) { // WRIST
      pucks[p].group_id = 2;
      pucks[p].group_order = p-4;
    } else { // HAND
      pucks[p].group_id = 3;
      pucks[p].group_order = p-7;
    }
    pucks[p].cpr = 4096;
    pucks[p].j_cpr = 4096;
    jumptime[p]=0;
    firstupdate[p]=true;
  }

  for(int p=1; p<=n_arm_pucks; p++){
    if(groups[ pucks[p].group() ].insert(&pucks[p]) == OW_FAILURE){
      ROS_ERROR("CANbus::CANbus: insert failed.");
      throw OW_FAILURE;
    }
  }
  
#ifdef PEAK_CAN
  can_accept[0] = 0x0000;  mask[0] = 0x03E0;
  can_accept[1] = 0x0403;  mask[1] = 0x03E0;
  can_accept[2] = 0x0406;  mask[2] = 0x03E0;
  can_accept[3] = 0x040A;  mask[2] = 0x03E0;
  can_accept[4] = 0x040B;  mask[2] = 0x03E0;
#endif // PEAK_CAN

  strcpy(last_error,"");
}

int CANbus::init(){ 
  if(open() == OW_FAILURE){
    ROS_ERROR("CANbus::init: open failed.");
    return OW_FAILURE;
  }

  if(check() == OW_FAILURE){
    ROS_ERROR("CANbus::init: check failed.");
    return OW_FAILURE;
  }

#ifdef OWD_RT
#ifdef ESD_CAN
  //  int err = rt_intr_create(&rt_can_intr, "ESDCAN_IRQ", 12, I_PROPAGATE);
#endif
#ifdef PEAK_CAN
  //  int err = rt_intr_create(&rt_can_intr, "PEAKCAN_IRQ", 19, I_PROPAGATE);
#endif
  
  //  if (err) {
  //    ROS_ERROR("Failed to create interrupt management object: %d",err);
  //    return OW_FAILURE;
  //  }

  //  err = rt_intr_enable(&rt_can_intr);
  //  if (err) {
  //    ROS_ERROR("Failed to enable interrupt for CAN interface: %d",err);
  //    return OW_FAILURE;
  //  }
#endif // OWD_RT

  return OW_SUCCESS;
}

int CANbus::open(){ 
   
#ifdef PEAK_CAN
  DWORD  err;

  char devicename[20];
  snprintf(devicename,20,"/dev/pcan%d",id);
  handle = LINUX_CAN_Open(devicename,O_RDWR);
  
  if (!handle) {
    ROS_ERROR("CANbus::open: CAN_Open(): cannot open device");
    throw OW_FAILURE;
  }
  
  // Clear Status
  err = CAN_Status(handle);
  ROS_DEBUG("CANbus::open: bus status = 0x%x",err);
  
  err = CAN_Init(handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
  if (err) {
    ROS_ERROR("CANbus::open: CAN_Init(): failed with 0x%x",err);
    return OW_FAILURE;		
  }
  
  if ((err = CAN_ResetFilter(handle))) {
    ROS_ERROR("CANbus::open: Could not Reset Filter: 0x%x",err);
  }
  if ((err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD))) {
    ROS_ERROR("CANbus::open: Could not set Msg Filter: 0x%x",err);
  }
	
#endif // PEAK_CAN
#ifdef ESD_CAN

  if(canOpen(id, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, 
	     TX_TIMEOUT, RX_TIMEOUT, &handle) != NTCAN_SUCCESS){  
    ROS_ERROR("CANbus::open: canOpen failed.");
    return OW_FAILURE;
  }   
  
  // 0 = 1Mbps, 2 = 500kbps, 4 = 250kbps
  if(canSetBaudrate(handle, 0) != NTCAN_SUCCESS){
    ROS_ERROR("CANbus::open: canSetBaudrate failed.");
    return OW_FAILURE;
  }
    
  // Mask 3E0: 0000 0011 1110 0000
  // Messages sent directly to host
  if(allow_message(0x0000, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 3 messages
  if(allow_message(0x0403, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 6 messages
  if(allow_message(0x0406, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 10 messages
  if(allow_message(0x040A, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 11 messages
  if(allow_message(0x040B, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
#endif // ESD_CAN

  // Set the minimum required property values
  // (we'll set the rest once we know the pucks' firmware version
  VERS = 0;
  STAT = 5;
  PROP_END = 10;
   
  return OW_SUCCESS;
}

int CANbus::check(){
  int32_t nodes[NUM_NODES+1];//, value;
  int online_pucks;

  //Get status of pucks on bus
  usleep(100000);
  if(status(nodes) == OW_FAILURE){
    return OW_FAILURE;
  }
   
  //count number of live pucks
  online_pucks = 0;
  int low=NODE_MIN, high=NODE_MAX;
#ifdef BH280_ONLY
  if (forcetorque_data) {
    low=8;
  } else {
    low=11;
  }
  high=14;
#endif // BH280_ONLY

  for(int n=low; n<=high; n++){
    
    if(nodes[n] != STATUS_OFFLINE){      // display online nodes
      ROS_DEBUG_NAMED("cancheck","Node (id, status): (%d,%s)",n,(nodes[n] == STATUS_RESET)?"reset":"running");
    }
    
    if(nodes[n]!=STATUS_OFFLINE && n!=SAFETY_MODULE) {
      online_pucks++;
    }
  }
  
  if (online_pucks==0){
    ROS_INFO_NAMED("cancheck","The wam appears to be turned off");
    return OW_FAILURE;
  }
  int n_expected_pucks=n_arm_pucks;
  if (BH280_installed) {
    n_expected_pucks += 4;
  }
  if (forcetorque_data) {
    n_expected_pucks += 1;
  }
  if(online_pucks < n_expected_pucks){
    ROS_INFO_NAMED("cancheck","Bus has %d pucks. We expected %d",online_pucks,n_expected_pucks);
    send_puck_reset(low,high);
    return OW_FAILURE;
  }  

#ifndef BH280_ONLY
  if(nodes[SAFETY_MODULE] == STATUS_OFFLINE){
    ROS_DEBUG_NAMED("cancheck","Safety module is offline");
    return OW_FAILURE;
  }

  int reset_pucks = 0;
  int running_pucks = 0;
  for(int p=1; p<=n_arm_pucks; p++){
    if(nodes[ pucks[p].id() ] == STATUS_RESET)
      reset_pucks++;
    else if(nodes[ pucks[p].id() ] != STATUS_OFFLINE)
      running_pucks++;
  }
   
  ROS_DEBUG_NAMED("cancheck","Expected: %d; Online: %d",n_arm_pucks,online_pucks);
  ROS_DEBUG_NAMED("cancheck","Running: %d; Reset: %d",running_pucks,reset_pucks);
      
  if((running_pucks+reset_pucks)!=n_arm_pucks){
    ROS_WARN("CANbus::check: Some of the pucks must have reset unexpectedly");
    ROS_WARN("ALL pucks should be in either running or reset state");
    return OW_FAILURE;
  }
      
  ROS_INFO_NAMED("cancheck","  Initializing pucks 1 to %d...",n_arm_pucks);
  for(int p=1; p<=n_arm_pucks; p++){

    ROS_DEBUG_NAMED("cancheck","Checking puck %d",pucks[p].id());
	 
    if(nodes[ pucks[p].id() ] == STATUS_RESET){
      ROS_DEBUG_NAMED("cancheck","Waking up the puck %d...",pucks[p].id());
      if(wake_puck(pucks[p].id()) == OW_FAILURE){
	ROS_WARN("wake_puck failed.");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
      usleep(10000);
    } else {
      ROS_DEBUG_NAMED("cancheck"," (running)");
      ROS_DEBUG_NAMED("cancheck","setting idle mode...");
      if(set_property_rt(pucks[p].id(), MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
	ROS_WARN("set_property failed (MODE)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }      
    ROS_DEBUG_NAMED("cancheck","Setting puck index values...");
    if(set_property_rt(pucks[p].id(),PIDX, pucks[p].order(),false,15000)==OW_FAILURE){
      ROS_WARN("CANbus::check: set_property failed (PID)");
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("cancheck","OK");
    
    ROS_DEBUG_NAMED("cancheck","Setting max torque...");
    int32_t max_torque;
    if (1 <= p && p <= 7) {
      max_torque = Puck::MAX_TRQ[p-1];
    } else {
      ROS_ERROR("CANbus::check: Unknown puck id of %d",p);
      throw -1;  // unknown puck id
    }
    if(set_property_rt(pucks[p].id(), MT, max_torque, false,15000) == OW_FAILURE){
      ROS_WARN("CANbus::check: set_property failed (max torque)");
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("cancheck","OK");

    ROS_DEBUG_NAMED("cancheck","Setting group ID values...");
    if (set_puck_group_id(pucks[p].id()) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: set_puck_group_id(%d) failed", pucks[p].id());
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("cancheck","OK");

  }
  ROS_INFO_NAMED("cancheck","done.");
#endif  // not BH280_ONLY

  if (BH280_installed) {
    ROS_INFO_NAMED("can_bh280","  Initializing hand pucks 11 to 14...");
    if (hand_activate(nodes) != OW_SUCCESS) {
      ROS_WARN_NAMED("can_bh280","CANbus::check: Hand not initialized");
      return OW_FAILURE;
    }
    ROS_INFO_NAMED("can_bh280","done.");
  }

  if (forcetorque_data) {
    ROS_INFO_NAMED("can_ft","  Initializing force/torque sensor...");
    if(wake_puck(8) == OW_FAILURE){
      ROS_WARN_NAMED("can_ft","CANbus::check: wake_puck failed.");
      return OW_FAILURE;
    }
    ROS_INFO_NAMED("can_ft","done.");

    if (set_puck_group_id(8) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: set_puck_group_id(8) failed");
      return OW_FAILURE;
    }
  }

#ifndef BH280_ONLY
  for (int nodeid=1; nodeid <=14; ++nodeid) {
    if ((nodeid > n_arm_pucks) && (nodeid < 8)) {
      // if 4 dof arm, skip to puck 8 after when done enumerating the 4 arm pucks
      nodeid = 8;
    }
    if ((nodeid ==8) && !forcetorque_data) {
      nodeid=11; // skip to the hand
    }
    if (nodeid ==9) {
      nodeid = 11; // skip over non-existent pucks 9 and 10
    }
    if ((nodeid >=11) && (nodeid <=14) && (! BH280_installed)) {
      continue;
    }
    
    int32_t a(-1),b(-1),c(-1),v(-1);
    if (get_property_rt(nodeid,GRPA,&a,20000) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: Failed to get puck %d GRPA during final check",nodeid);
      return OW_FAILURE;
    }
    if (get_property_rt(nodeid,GRPB,&b,20000) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: Failed to get puck %d GRPB during final check",nodeid);
      return OW_FAILURE;
    }
    if (get_property_rt(nodeid,GRPC,&c,20000) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: Failed to get puck %d GRPC during final check",nodeid);
    }
    if (get_property_rt(nodeid,VERS,&v,20000) != OW_SUCCESS) {
      ROS_WARN("CANbus::check: Failed to get puck %d VERS during final check",nodeid);
    }
    ROS_INFO_NAMED("cancheck","Puck %d: GRPA=%d, GRPB=%d, GRPC=%d, VERS=%d",
		   nodeid,a,b,c,v);
  }

  if (set_limits() != OW_SUCCESS) {
    ROS_WARN("Canbus::check: Failed to set safety puck limits");
  }

  int32_t voltlevel;
  if (get_property_rt(SAFETY_MODULE,VOLTL1,&voltlevel) == OW_SUCCESS) {
    ROS_DEBUG_NAMED("safety","VOLTL1 = %d",voltlevel);
  }
  if (get_property_rt(SAFETY_MODULE,VOLTL2,&voltlevel) == OW_SUCCESS) {
    ROS_DEBUG_NAMED("safety","VOLTL2 = %d",voltlevel);
  }
  if (get_property_rt(SAFETY_MODULE,VOLTH1,&voltlevel) == OW_SUCCESS) {
    ROS_DEBUG_NAMED("safety","VOLTH1 = %d",voltlevel);
  }
  if (get_property_rt(SAFETY_MODULE,VOLTH2,&voltlevel) == OW_SUCCESS) {
    ROS_DEBUG_NAMED("safety","VOLTH2 = %d",voltlevel);
  }
#endif  // not BH280_ONLY

  return OW_SUCCESS;
}

void CANbus::send_puck_reset(int32_t low, int32_t high) {
  for (int32_t i=low; i<=high; ++i) {
    set_property_rt(i,MODE,0,false,15000);
    //set_property_rt(i,STAT,0,false,15000);
  }
}

void CANbus::dump(){
  //  ROS_DEBUG("Printing bus information.");
  for(int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++){
    //    if(groups[g].id() != GROUP_INVALID)
      // ROS_DEBUG_STREAM(groups[g]);
  }
}
   
int CANbus::allow_message(int32_t id, int32_t mask){
  
#ifdef PEAK_CAN
  DWORD err;
  if ((err = CAN_ResetFilter(handle)) ||
      (err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD))) {
    ROS_WARN("CANbus::allow_message: Could not set Msg Filter: 0x%x",err);
    return OW_FAILURE;
  }
#endif // PEAK_CAN

#ifdef ESD_CAN
  int i;
  for(i=0; i<2048; i++){
    if((i & ~mask) == id){
      if(canIdAdd(handle, i) != NTCAN_SUCCESS){
	ROS_WARN("CANbus::allow_message: canIdAdd failed.");
	return OW_FAILURE;
      }
    }
  }
#endif  // ESD_CAN

  return OW_SUCCESS;
}
 
int CANbus::wake_puck(int32_t puck_id){ 
    if(set_property_rt(puck_id, STAT, STATUS_READY, false) == OW_FAILURE){
      ROS_WARN("CANbus::wake_puck: set_property failed for puck %d.",puck_id);
        return OW_FAILURE;
    }
    usleep(750000); // Wait 750ms for puck to initialize    
    return OW_SUCCESS;
}

int CANbus::clear() {
  unsigned char msg[8];
  int32_t msgid, msglen;
  
  int count(0);
  while (read_rt(&msgid, msg, &msglen, 50) == OW_SUCCESS) {
    ++count;
  }
  return count;
}


// Got to make sure that nodes is NUM_NODES+1 long !!!
int CANbus::status(int32_t* nodes){
  unsigned char msg[8];
  int32_t msgid, msglen, nodeid, property;
  int firstFound = 0;

  ROS_INFO_NAMED("canstatus","Probing for nodes on the CAN bus");

  int low=NODE_MIN;
  int high=NODE_MAX;
#ifdef BH280_ONLY
  if (forcetorque_data) {
    low=8;
  } else {
    low=11;
  }
  high=14;
#endif // BH280_ONLY
  for(int n=low; n<=high; ++n){
    msg[0] = (unsigned char) 5;   // STAT = 5 for all firmware versions
    nodes[n] = STATUS_OFFLINE;      // Initialize node as offline

    if(send_rt(NODE2ADDR(n), msg, 1, 100) == OW_FAILURE){
      ROS_WARN_NAMED("canstatus","send failed: %s",last_error);
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("canstatus","Sent probe message to puck %d id %d",n,NODE2ADDR(n));

    if(read_rt(&msgid, msg, &msglen, 40000) == OW_FAILURE){
      ROS_DEBUG_NAMED("canstatus","node %d didn't answer: %s",NODE2ADDR(n), last_error);
    }
    else{
      if(parse(msgid, msg, msglen,&nodeid,&property,&nodes[n])==OW_FAILURE){
	ROS_DEBUG_NAMED("canstatus","parse failed: %s",last_error);
	return OW_FAILURE;
      } else {
          // parsed ok
	ROS_DEBUG_NAMED("canstatus","parsed response from node %d",NODE2ADDR(n));
	// only check the firmware version if we're not reading from
	// the force/torque sensor
	if (!firstFound && (n != 8)) {
	  ROS_DEBUG_NAMED("canstatus","trying to wake node %d",n);
	  if ((wake_puck(NODE2ADDR(n)) == OW_SUCCESS) &&
	      (get_property_rt(NODE2ADDR(n), VERS, &fw_vers, 40000) == OW_SUCCESS)){
	    ROS_DEBUG_NAMED("canstatus","puck %d firmware version %d",n,fw_vers);
	    initPropertyDefs(fw_vers);
	    firstFound = 1;
	  }
	  else {
	    ROS_DEBUG_NAMED("canstatus","unable to get firmware vers from puck %d",n);
	    return OW_FAILURE;
	  }
	}
      }
    }
  }

  if (!firstFound) {
      // we never got a firmware version, so we never initialized
      // the property definitions.
    ROS_WARN_NAMED("canstatus","Could not determine puck firmware version");
    ROS_WARN_NAMED("canstatus","Unable to continue");
    send_puck_reset(low,high);
    return OW_FAILURE;
  }
  
  ROS_INFO_NAMED("canstatus","done");
  return OW_SUCCESS;
}

 int CANbus::set_property_rt(int32_t nid, int32_t property, int32_t value,bool check, int32_t usecs){
  uint8_t msg[8];
  int32_t response;
  int32_t msglen;
   
  if(compile(property, value, msg, &msglen) ==OW_FAILURE){
    ROS_WARN("CANbus::set_property: compile failed.");
    return OW_FAILURE;
  }
  msg[0] |= (uint8_t)0x80; // Set the 'Set' bit

  if(send_rt(NODE2ADDR(nid), msg, msglen, usecs) == OW_FAILURE){
    ROS_WARN("CANbus::set_property: send failed: %s",last_error);
    return OW_FAILURE;
  }
   
  if(check){
    // Get the new value of the property
    if(get_property_rt(nid, property, &response, usecs) == OW_FAILURE){
      ROS_WARN("CANbus::set_property: get_property failed.");
      return OW_FAILURE;
    }

    // Compare response to value
    if(response != value){
      ROS_WARN("CANbus::set_property: value confirmation failed.");
      ROS_WARN("Puck %d, Property %d Set Value %d Response Value %d",nid,property,value,response);
      return OW_FAILURE;
    }
  }   

  return OW_SUCCESS;
}

int CANbus::request_property_rt(int32_t id, int32_t property){
  uint8_t msg[8];

  if ((property > PROP_END) || (property < 0)) {
    ROS_WARN("CANbus::request_property: requested property is not valid for this puck's firmware version");
    return OW_FAILURE;
  }

  msg[0] = (uint8_t)property;   

  if (send_rt(id, msg, 1, 1000) == OW_FAILURE) {
    ROS_WARN("CANbus::request_property: send failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::get_property_rt(int32_t nid, int32_t property, int32_t* value, int32_t usecs, int32_t retries){

  uint8_t  msg[8];
  int32_t msgid, msglen, nodeid, prop;
  do {
    if (request_property_rt(nid, property) != OW_SUCCESS) {
      ROS_ERROR("CANbus::get_property: error while sending request");
      return OW_FAILURE;
    }

  REREAD:
    if(read_rt(&msgid, msg, &msglen, usecs) == OW_FAILURE){
      ROS_WARN("CANbus::get_property: read failed: %s",last_error);
      ++unread_packets;
      continue;
    }
    
    // Parse the reply
    if(parse(msgid, msg, msglen, &nodeid, &prop, value) == OW_FAILURE){
      ROS_WARN("CANbus::get_property: parse failed: %s",last_error);
      continue;
    }
    
    // Check that the ids and properties match
    if(nodeid!=nid) {
      // ROS_WARN("Asked for property %d from node %d but got answer from node %d, %d previously missed messages",
      //	       property, nid, nodeid, unread_packets);
      if (unread_packets > 0) {
	//	ROS_WARN("Throwing out packet and waiting for next one (%d missed messages remaining)", unread_packets);
	--unread_packets;
	goto REREAD;
      }
      continue;
    }
    else if (prop!=property) {
      //ROS_WARN("Asked node %d for property %d but got property %d",
      //	       nid, property, prop);
      if (unread_packets > 0) {
	//	ROS_WARN("Throwing out packet and waiting for next one (%d missed messages remaining)",
	//		 unread_packets);
	--unread_packets;
	goto REREAD;
      }
      continue;
    }
    
    return OW_SUCCESS;
  } while (retries-->0);
  return OW_FAILURE;
  
}

int CANbus::send_torques(int32_t* torques){
  for(int p=1; p<=n_arm_pucks; p++) {
    trq[p] = torques[p];
  }
  return OW_SUCCESS;
}

int CANbus::send_torques_rt()
{
  int i;
  
  uint8_t msg[8];
  int32_t torques[NUM_ORDERS+1];
  Puck* puck;

  static double sendtime=0.0f;
  static unsigned int sendcount=0;

  static int32_t *mytorqs = (int32_t *) malloc(NUM_NODES * sizeof(int32_t));

  memcpy(mytorqs,trq,NUM_NODES*sizeof(int32_t));

#ifdef RT_STATS
  RTIME bt1 = time_now_ns();
#endif

  for (int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++)
  {
    if (groups[g].id() != GROUP_INVALID)
    {
      for (int p=PUCK_ORDER_MIN; p<=PUCK_ORDER_MAX; p++)
      {
        puck = groups[g].puck(p);
        if (!puck)
        {
          torques[p] = 0;
          continue;
        }
        
        torques[p] = mytorqs[puck->motor()];

        /* Due to the way the differentials work, we may see motor torques above
         * the per-motor maximums (up to Puck::MAX_CLIPPABLE_TRQ).
         * Do a limited clip here to account for that. */
        if (-Puck::MAX_CLIPPABLE_TRQ[puck->id()-1] <= torques[p] && torques[p] <= -Puck::MAX_TRQ[puck->id()-1])
           torques[p] = -Puck::MAX_TRQ[puck->id()-1];
        if (Puck::MAX_TRQ[puck->id()-1] <= torques[p] && torques[p] <= Puck::MAX_CLIPPABLE_TRQ[puck->id()-1])
           torques[p] = Puck::MAX_TRQ[puck->id()-1];
        
        /* Check each puck's torque against its per-puck torque limits.
         * This should never happen, because torques are checked per-joint before this. */
        if (!(-Puck::MAX_TRQ[puck->id()-1] <= torques[p] && torques[p] <= Puck::MAX_TRQ[puck->id()-1]))
        {
          emergency_shutdown(2,puck->id());
          ROS_FATAL("Torque %d for puck %d exceeded per-puck legal range %d to %d; motors have been idled",
              torques[p], puck->id(), -Puck::MAX_TRQ[puck->id()-1], Puck::MAX_TRQ[puck->id()-1]);
          {
            int g2;
            int p2;
            ROS_FATAL("Other puck torques:");
            for (g2=GROUP_ID_MIN; g2<=GROUP_ID_MAX; g2++) if (groups[g2].id() != GROUP_INVALID)
            {
              for (p2=PUCK_ORDER_MIN; p2<=PUCK_ORDER_MAX; p2++) if (groups[g2].puck(p2))
              {
                ROS_FATAL("   puck %d torque %d min %d max %d\n",
                  groups[g2].puck(p2)->id(),
                  mytorqs[groups[g2].puck(p2)->motor()],
                  -Puck::MAX_TRQ[groups[g2].puck(p2)->id()-1],
                  Puck::MAX_TRQ[groups[g2].puck(p2)->id()-1]);
              }
            }
          }
          ROS_FATAL("This should never happen! Fix the bug in owd ...");
          return OW_FAILURE;
        }
      }
      
      /* We have 14 bits (including sign bit) in the packed torque message per puck.
       * If any of the torques are more extreme than this, abort!
       * This should never happen, because torques are checked before this,
       * unless someone raised the MAX_TRQ limits too high. */
      for (i=1; i<5; i++)
      {
        if (!(-8192 <= torques[i] && torques[i] <= 8191))
        {
          emergency_shutdown(2,groups[g].puck(i)->id());
          ROS_FATAL("Torque %d for group %d index %d exceeded bit packing limit; motors have been idled",
              torques[i], g, i-1);
          ROS_FATAL("This should never happen! Fix the bug in owd ...");
          return OW_FAILURE;
        }
      }

      msg[0] = TORQ | 0x80;
      msg[1] = (uint8_t)(( torques[1]>>6)&0x00FF);
      msg[2] = (uint8_t)(((torques[1]<<2)&0x00FC) | ((torques[2]>>12)&0x0003));
      msg[3] = (uint8_t)(( torques[2]>>4)&0x00FF);
      msg[4] = (uint8_t)(((torques[2]<<4)&0x00F0) | ((torques[3]>>10)&0x000F));
      msg[5] = (uint8_t)(( torques[3]>>2)&0x00FF);
      msg[6] = (uint8_t)(((torques[3]<<6)&0x00C0) | ((torques[4]>>8) &0x003F));
      msg[7] = (uint8_t)(  torques[4]    &0x00FF);
          
      if(send_rt(GROUPID(groups[g].id()), msg, 8, 100) == OW_FAILURE)
      {
        ROS_ERROR("CANbus::set_torques: send failed: %s",last_error);
        return OW_FAILURE;
      }
    }
  }

#ifdef RT_STATS
  RTIME bt2 = time_now_ns();
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
  if (++sendcount == 1000)
  {
    stats.cansend_time = sendtime/1000.0;
    sendcount=0;
    sendtime=0.0f;
  }
#endif // RT_STATS

  return OW_SUCCESS;
}

int CANbus::extra_bus_commands() {
  CANmsg handmsg;
  bool message_received(false);
#ifdef OWD_RT
  ssize_t bytecount = rt_pipe_read(&handpipe,&handmsg,sizeof(CANmsg),TM_NONBLOCK);
  if (bytecount > 0) {
    if (bytecount < sizeof(CANmsg)) {
      snprintf(last_error,200,"Incomplete read of CANmsg from RT message pipe");
      return OW_FAILURE;
    }
    message_received=true;
  }
#else // ! OWD_RT
  bool mutex_locked=false;
  if (!mutex_trylock(&hand_queue_mutex)) {
    mutex_locked=true;
    if (hand_command_queue.size() > 0) {
      handmsg = hand_command_queue.front();
      hand_command_queue.pop();
      message_received=true;
    }
  }
#endif // ! OWD_RT
  if (message_received) {
    if (handmsg.property & 0x80) {
      // if bit 7 is 1 it's a set
      
      if (set_property_rt(handmsg.nodeid,handmsg.property & 0x7F,handmsg.value,false)
	  != OW_SUCCESS) {
	  snprintf(last_error,200,"Error setting property %d = %d on hand puck %d",
		   handmsg.property, handmsg.value, handmsg.nodeid);
#ifndef OWD_RT
	  mutex_unlock(&hand_queue_mutex);
#endif // ! OWD_RT
	  return OW_FAILURE;
      }
      if ((handmsg.property & 0x7F)== MODE) {
	if ((handmsg.value == MODE_TRAPEZOID)
	    || (handmsg.value == MODE_VELOCITY)
	      || (handmsg.value == MODE_TORQUE)) {
	  // record the fact that the motion command has actually been sent
	  hand_motion_state_sequence=2;
	}
      }
      if ((handmsg.property & 0x7f) == FT) {
	// we just sent the tare command to the sensor, so now
	// we're ready to start collecting values for a more accurate tare.
	// we'll still throw away the first value that comes back.
	force_tare_values_collected
	  = torque_tare_values_collected
	  = -1;
      }
    } else {
      if (request_property_rt(handmsg.nodeid, handmsg.property) != OW_SUCCESS) {
	snprintf(last_error,200,"Error getting property %d from hand puck %d",
		 handmsg.property, handmsg.nodeid);
#ifndef OWD_RT
	mutex_unlock(&hand_queue_mutex);
#endif // ! OWD_RT
	return OW_FAILURE;
      }
    }
  }
#ifndef OWD_RT
  if (mutex_locked) {
    mutex_unlock(&hand_queue_mutex);
  }
#endif // ! OWD_RT
  return OW_SUCCESS;
}

#ifdef NDEF
/*
int CANbus::read_torques(int32_t* mtrq){
  uint8_t  msg[8];
  int32_t value;
  int32_t msgid, msglen, nodeid, property;

  // Compile the packet
  msg[0] = (uint8_t)TORQ;


  if(send_rt(GROUPID(0), msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::read_torques: send failed: %s", last_error);
    return OW_FAILURE;
  }
  for(int p=1; p<=n_arm_pucks; ){
    if(read(&msgid, msg, &msglen, 100) == OW_FAILURE){
      ROS_WARN("CANbus::read_torques: read failed on puck %d: %s",p,last_error);
      return OW_FAILURE;
    }
    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      ROS_WARN("CANbus::read_torques: parse failed: %s",last_error);
      return OW_FAILURE;
    }
    if(property == TORQ){
      trq[nodeid] = value;
      p++;
    }
  }

  return OW_SUCCESS;
}
*/
#endif

int CANbus::read_positions(double* positions){

  for(int p=1; p<=n_arm_pucks; p++) {
    positions[p] = pos[p];
  }

  return OW_SUCCESS;
}

int CANbus::request_positions_rt(int32_t id) {
  uint8_t  msg[8];

  // clear the flags that keep track of which responses we've gotten
  if (id == 0x404) {
    // clear the arm flags (lower 8 bits)
    received_position_flags &= 0xFF00;
  } else if (id == 0x405) {
    // clear the hand flags (upper 8 bits)
    received_position_flags &= 0x00FF;
  } else if (id == 11) {
    received_position_flags &= ~(0x0800);
  } else if (id == 12) {
    received_position_flags &= ~(0x1000);
  } else if (id == 13) {
    received_position_flags &= ~(0x2000);
  } else if (id == 14) {
    received_position_flags &= ~(0x4000);
  } else {
    // clear all the flags
    received_position_flags=0;
  }

  // Compile the packet
  msg[0] = (uint8_t)AP;

#ifdef RT_STATS
  RTIME bt1 = time_now_ns();
#endif // RT_STATS

  if(send_rt(id, msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_positions: send failed: %s",last_error);
    return OW_FAILURE;
  }

  // now that the request has been sent on the CANbus, record the time that
  // we expect the puck will take the position readings.  Barrett's estimate is
  // 75 microseconds after the puck receives the request.
  int low,high;
  if (id == 0x404) {
    low=1; high=7;
  } else if (id == 0x405) {
    low=11; high=14;
  } else {
    low = id; high=id;
  }
  for (int p=low; p<=high; ++p) {
    next_encoder_clocktime[p] = time_now_ns() + 75000; // +75 microseconds
  }

#ifdef RT_STATS
  static double sendtime=0.0f;
  static int loopcount=0;
  RTIME bt2 = time_now_ns();
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
  if (++loopcount == 1000) {
    stats.canread_sendtime=sendtime/1000.0;
    sendtime=loopcount=0;
  }
#endif // RT_STATS
  
  return OW_SUCCESS;
}

int CANbus::process_positions_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  int32_t nodeid, property, value, value2;

  // extract the payload
  if(parse(msgid, msg, msglen, &nodeid, &property, &value, &value2) == OW_FAILURE){
    //    ROS_WARN("CANbus::process_positions: parse failed: %s",last_error);
    return OW_FAILURE;
  }

  // verify the property
  if (property != AP) {
    //ROS_WARN("CANbus::process_positions: unexpected property %d", property);
    return OW_FAILURE;
  }

  // convert the result
  if ((nodeid >= 1) && (nodeid <=n_arm_pucks)) {

    // check to see if the puck encoder value jumped by more than 1000
    // counts in a single cycle, and if so log the time (to capture a
    // puck problem we've been seeing on the ARM-S right arm joint 1)
    int32_t oldvalue = pos[ pucks[nodeid].motor() ] / 2.0 / M_PI * pucks[nodeid].CPR();
    if ((fabs(value - oldvalue) > 1000) && (!firstupdate[pucks[nodeid].motor()])) {
      jumptime[pucks[nodeid].motor()] = time_now_ns() / 1e6;
    } else {
      // update the new value
      rawpos[pucks[nodeid].motor()]=value;
      pos[ pucks[nodeid].motor() ] = 2.0*M_PI*( (double) value )/ 
	( (double) pucks[nodeid].CPR() );
      firstupdate[pucks[nodeid].motor()] = false;
      // record the time that this reading was taken
      last_encoder_clocktime[pucks[nodeid].motor()] = 
	next_encoder_clocktime[pucks[nodeid].motor()];
    }

    if (msglen==6) {
      // this puck also sent a joint encoder value
      jpos[pucks[nodeid].motor() ] = 2.0*M_PI*( (double) value2 )/ 
      ( (double) pucks[nodeid].J_CPR() );
    }
  } else if (BH280_installed && (nodeid >=11) && (nodeid <=14)) {
    hand_positions[nodeid-10] = value;
    if (labs(value - last_hand_positions[nodeid-10]) > 50) {
      encoder_changed[nodeid-11] = 6;
    } else {
      // it will take 6 stationary cycles before encoder_changed
      // becomes zero
      if (encoder_changed[nodeid-11] > 0) {
	--encoder_changed[nodeid-11];
      }
    }
    last_hand_positions[nodeid-10] = value;
    if ((msglen==6) && (!ignore_breakaway_encoders)) {
      // this puck also sent a value for the secondary encoder
      hand_secondary_positions[nodeid-10] = value2;

      update_link_positions(nodeid-10);

      // If the finger has not gone into breakaway, then the outer link angle
      // should always be approx. 1/3 of the inner link angle.  If it
      // strays from this ratio by more than 9,500 (5% of full range) we
      // mark the finger as being in breakaway
      // Note that this check only happens if we received the
      // secondary encoder values.  Some 280 hands still don't have
      // working inner-link encoders.
      if (nodeid < 14) { // no breakaway for spread joint
	if ((hand_positions[nodeid-10]
	     - 2.5*hand_secondary_positions[nodeid-10]) > 9500) {
	  hand_breakaway[nodeid-10] = true;
	}
      }
    }
  }

  // set the flag
  received_position_flags |= 1 << nodeid;

  return OW_SUCCESS;
}

int CANbus::read_positions_rt(){
  // send request to Group 4 pucks (all of arm)
  if (request_positions_rt(GROUPID(4)) != OW_SUCCESS) {
    ROS_WARN("Could not send request for positions");
    return OW_FAILURE;
  }

  while ((received_position_flags & 0xFE) != 0xFE) {
    uint8_t  msg[8];
    int32_t msgid, msglen;
    if (read_rt(&msgid, msg, &msglen, 2000) == OW_FAILURE){
      ROS_WARN("CANbus::read_positions_rt: timeout waiting for response");
      return OW_FAILURE;
    }
    if ((msgid & 0x41F) == 0x403) {
      // 22-bit AP response
      process_positions_rt(msgid, msg, msglen);
    } else {
      // ROS_WARN("CANbus::read_positions_rt: received unexpected CAN message ID %X while waiting for AP", msgid);
    }
  }

  return OW_SUCCESS;
}

int CANbus::ft_combine(unsigned char lsb, unsigned char msb) {
  int value = ((int) msb << 8) | lsb;
  if (value & 0x00008000) {
    value |= 0xffff0000;
  }
  return value;
}

int CANbus::request_forcetorque_rt() {
  if (! forcetorque_data) {
    return OW_FAILURE;
  }
  uint8_t  msg[8];

  // Compile the packet
  msg[0] = (uint8_t)FT;


  if(send_rt(8, msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_forcetorque_rt: send failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::request_accelerometer_rt() {
  if (! accelerometer_data) {
    return OW_FAILURE;
  }
  uint8_t  msg[8];

  // Compile the packet
  msg[0] = (uint8_t)A;


  if(send_rt(8, msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_accelerometer_rt: send failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::request_ecminmax_rt(int32_t id) {
  uint8_t  msg[8];

  // Compile the packet
  msg[0] = (uint8_t)ECMIN;

  if(send_rt(id, msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_ecminmax_rt: request for ECMIN failed: %s",last_error);
    return OW_FAILURE;
  }
  // Compile the packet
  msg[0] = (uint8_t)ECMAX;

  if(send_rt(id, msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_ecminmax_rt: request for ECMAX failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::process_forcetorque_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  int32_t nodeid = ADDR2NODE(msgid);

  // verify the sender
  if (nodeid != 8) {
    //    ROS_WARN("CANbus::process_forcetorque_response_rt: response from unexpected puck id %d", nodeid);
    return OW_FAILURE;
  }

  if (((msgid & 0x41F) == 0x40A) && valid_forcetorque_data) {

    // Group 10 is Force
    forcetorque_data[0]=ft_combine(msg[0], msg[1]) / 256.0;
    forcetorque_data[1]=ft_combine(msg[2], msg[3]) / 256.0;
    forcetorque_data[2]=ft_combine(msg[4], msg[5]) / 256.0;

    if (force_tare_values_collected < ft_tare_values_to_average) {

      // make sure we're not still waiting for the tare command to be
      // sent out (counter equal to -2).  Once it is, the counter gets
      // incremented to -1, and then we'll still throw out the first
      // one after that (because it might have been requested before
      // the sensor got the tare command).
      if (force_tare_values_collected != -2) {
	if (++force_tare_values_collected > 0) {
	  // the ft_tare_avg field will hold the sum until we
	  // get enough values.  The valid_filtered_forcetorque_data field
	  // ensures that the avg values won't be used until
	  // we sum enough values and then calc the average.
	  ft_tare_avg[0] += forcetorque_data[0];
	  ft_tare_avg[1] += forcetorque_data[1];
	  ft_tare_avg[2] += forcetorque_data[2];
	}
	if (force_tare_values_collected == ft_tare_values_to_average) {
	  ft_tare_avg[0] /= ft_tare_values_to_average;
	  ft_tare_avg[1] /= ft_tare_values_to_average;
	  ft_tare_avg[2] /= ft_tare_values_to_average;
	  if (torque_tare_values_collected == ft_tare_values_to_average) {
	    // if we also have enough torque values, then our data
	    // is finally valid
	    valid_filtered_forcetorque_data=true;
	  }
	}
      }
      // since we're still collecting data, we'll make it look like the
      // sensor is still perfectly tared
      forcetorque_data[0]
	= forcetorque_data[1]
	= forcetorque_data[2]
	=0;
    } else {
      // subtract the post-tare average
      //forcetorque_data[0] -= ft_tare_avg[0];
      //forcetorque_data[1] -= ft_tare_avg[1];
      //forcetorque_data[2] -= ft_tare_avg[2];
    }
    // update the filtered force values
    R3 force(forcetorque_data[0],forcetorque_data[1],forcetorque_data[2]);
    mutex_lock(&ft_mutex);
    force = ft_force_filter.eval(force);
    mutex_unlock(&ft_mutex);
    filtered_forcetorque_data[0] = force[0];
    filtered_forcetorque_data[1] = force[1];
    filtered_forcetorque_data[2] = force[2];

  } else if ((msgid & 0x41F) == 0x40B) {
    // Group 11 is Torque

    valid_forcetorque_data=true;
    valid_forcetorque_flag = 0;

    if (msglen == 7) {
      // FT error byte has been sent
      if (msg[6] & 64) {
	// Bad forcetorque data flag has been set
	// This is the case when the sensor is actively saturated due to overload
	// Discard data
	valid_forcetorque_data = false;
	valid_forcetorque_flag = -1; 
      }
      else {
	//This is the case when the sensor is not actively saturated
	//But a load cell has been saturated since the latest re-tare
	//Report bits containing which of the axis have saturated since latest re-tare
	valid_forcetorque_flag = msg[6] & 63;
      }
    }
  
    if (valid_forcetorque_data) {
      forcetorque_data[3]=ft_combine(msg[0], msg[1]) / 4096.0;
      forcetorque_data[4]=ft_combine(msg[2], msg[3]) / 4096.0;
      forcetorque_data[5]=ft_combine(msg[4], msg[5]) / 4096.0;
      
      if (torque_tare_values_collected < ft_tare_values_to_average) {
	// same as above, but for torques
	if (torque_tare_values_collected != -2) {
	  if (++torque_tare_values_collected > 0) {
	    ft_tare_avg[3] += forcetorque_data[3];
	    ft_tare_avg[4] += forcetorque_data[4];
	    ft_tare_avg[5] += forcetorque_data[5];
	  }
	  if (torque_tare_values_collected == ft_tare_values_to_average) {
	    ft_tare_avg[3] /= ft_tare_values_to_average;
	    ft_tare_avg[4] /= ft_tare_values_to_average;
	    ft_tare_avg[5] /= ft_tare_values_to_average;
	    if (force_tare_values_collected == ft_tare_values_to_average) {
	      // if we also have enough force values, then our data
	      // is finally valid
	      valid_filtered_forcetorque_data=true;
	    }
	  }
	}
	// since we're still collecting data, we'll make it look like the
	// sensor is still perfectly tared
	forcetorque_data[3]
	  = forcetorque_data[4]
	  = forcetorque_data[5]
	  =0;
      } else {
	// subtract the post-tare average
	//      forcetorque_data[3] -= ft_tare_avg[3];
	//      forcetorque_data[4] -= ft_tare_avg[4];
	//      forcetorque_data[5] -= ft_tare_avg[5];
      }
      // update the filtered torque values
      R3 torque(forcetorque_data[3],forcetorque_data[4],forcetorque_data[5]);
      mutex_lock(&ft_mutex);
      torque = ft_torque_filter.eval(torque);
      mutex_unlock(&ft_mutex);
      filtered_forcetorque_data[3] = torque[0];
      filtered_forcetorque_data[4] = torque[1];
      filtered_forcetorque_data[5] = torque[2];
    } 
  } //end of torque block

    else if ((msgid & 0x41F) == 0x40C) {
    // Group 12 is accelerometer data
    if (accelerometer_data) {
      accelerometer_data[0]=ft_combine(msg[0], msg[1]) / 1024.0;
      accelerometer_data[1]=ft_combine(msg[2], msg[3]) / 1024.0;
      accelerometer_data[2]=ft_combine(msg[4], msg[5]) / 1024.0;
    }
  } else {
    //    ROS_ERROR("CANbus::process_forcetorque_response_rt: Unknown message type %X", msgid);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::request_tactile_rt() {
  if (! tactile_data) {
    return OW_FAILURE;
  }

  uint8_t  msg[8];

  // Compile the packet
  msg[0] = (uint8_t)TACT;

  if(send_rt(GROUPID(5), msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_tactile_rt: send failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::send_positions(double* mpos){
  int32_t position[n_arm_pucks+1];

  for(int p=1; p<=n_arm_pucks; p++){
    position[p] = (int32_t)floor( mpos[ pucks[p].motor() ]*
			    pucks[p].CPR()/(2.0*M_PI) );
  }   
  return send_AP(position);
}

int CANbus::send_AP(int32_t* apval){

  if(set_property_rt(SAFETY_MODULE, IFAULT, 8, false ,15000) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property IFAULT=8 failed.");
    return OW_FAILURE;
  }

  for(int p=1; p<=n_arm_pucks; p++){
    if(set_property_rt(pucks[p].id(), AP, apval[p], false) == OW_FAILURE){
      ROS_WARN("CANbus::send_AP: set_property AP failed.");
      return OW_FAILURE;
    }
  }

  // let the safety module see the new positions
  read_positions_rt();
   
  // start monitoring tip velocity again
  if(set_property_rt(SAFETY_MODULE, ZERO, 1,false,15000) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property ZERO=1 failed.");
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::emergency_shutdown(int faulttype, int motor) {
  ok=false; // let the top-level loop know
  if ((set_property_rt(GROUPID(1), MODE, MODE_IDLE, false, 10000) == OW_FAILURE) ||
      (set_property_rt(GROUPID(2), MODE, MODE_IDLE, false, 10000) == OW_FAILURE)) {
    ROS_FATAL("Could not idle the pucks for emergency shutdown");
    return OW_FAILURE;
  }
  if ((faulttype == 2) && (motor > 0) && (motor <= NUM_NODES)) {
    // Torque fault, so send a high torque message to the appropriate
    // puck so that the safety board will see it and do its own safety
    // shutdown
    uint8_t msg[8];
    uint32_t torques[5];
    torques[1]=torques[2]=torques[3]=torques[4]=0;
    int groupid(1);
    if ((1 <= motor) && (motor <= 4)) {
      torques[motor]=max_safety_torque+1;
    } else if ((5 <= motor) && (motor <= 7)) {
      groupid=2;
      torques[motor-4]=max_safety_torque+1;
    } else {
      ROS_ERROR("Cannot trigger safety torque fault for requested motor %d (out of range", motor);
      return OW_FAILURE;
    }

    msg[0] = TORQ | 0x80;
    msg[1] = (uint8_t)(( torques[1]>>6)&0x00FF);
    msg[2] = (uint8_t)(((torques[1]<<2)&0x00FC) | ((torques[2]>>12)&0x0003));
    msg[3] = (uint8_t)(( torques[2]>>4)&0x00FF);
    msg[4] = (uint8_t)(((torques[2]<<4)&0x00F0) | ((torques[3]>>10)&0x000F));
    msg[5] = (uint8_t)(( torques[3]>>2)&0x00FF);
    msg[6] = (uint8_t)(((torques[3]<<6)&0x00C0) | ((torques[4]>>8) &0x003F));
    msg[7] = (uint8_t)(  torques[4]    &0x00FF);
    if (motor > 4) {
      groupid=2;
    }
    if(send_rt(GROUPID(groupid), msg, 8, 100) == OW_FAILURE) {
      ROS_ERROR("Could not trigger a torque fault by the safety system");
      return OW_FAILURE;
    }
  }
  return OW_SUCCESS;
}


int CANbus::parse(int32_t msgid, uint8_t* msg, int32_t msglen,
		  int32_t* nodeid, int32_t* property, int32_t* value,
		  int32_t *value2){

  int32_t i;
  int32_t dataHeader;

  if (!msg || !nodeid || !property || !value) {
    ROS_ERROR("CANbus::parse requires non-NULL pointers for msg, nodeid, property, and value");
    return OW_FAILURE;
  }

  *nodeid = ADDR2NODE(msgid);
  if(*nodeid == -1){
    snprintf(last_error,200,"invalid node id %d",*nodeid);
    return OW_FAILURE;
  }
   
  dataHeader = ((msg[0] >> 6) & 0x0002) | ((msgid & 0x041F) == 0x0403);

  switch (dataHeader){
      
  case 3:  // Data is a packed 22-bit position, SET 
    *value = 0x00000000;
    *value |= ( (int32_t)msg[0] << 16) & 0x003F0000;
    *value |= ( (int32_t)msg[1] << 8 ) & 0x0000FF00;
    *value |= ( (int32_t)msg[2] )      & 0x000000FF;
      
    if(*value & 0x00200000) { // If negative 
      *value |= 0xFFC00000; // Sign-extend 
    }

    *property = AP;

    if ((msglen==6) && value2) {
      // the next three data bytes encode the 22-bit joint encoder
      *value2 = 0;
      *value2 |= ( (int32_t)msg[3] << 16) & 0x003F0000;
      *value2 |= ( (int32_t)msg[4] << 8 ) & 0x0000FF00;
      *value2 |= ( (int32_t)msg[5] ) & 0x000000FF;
      
      if (*value2 & 0x00200000) { /* If negative */
	*value2 |= 0xFFC00000; /* Sign-extend */
      }
    }
    break;

  case 2:  // Data is normal, SET 

    // copied from Barrett's source code
     *property = msg[0] & 0x7F;
     *value = msg[msglen-1] & 0x80 ? -1L : 0;
      for (i = msglen-1; i >= 2; i--)
      *value = *value << 8 | msg[i];
      break;

  case 0:  // Assume firmware request (GET) 
      
    // A negative (or zero) property means GET
    *property = -(msg[0] & 0x7F); 
    *value = 0;
    break;
      
  default:
    snprintf(last_error,200,"Illegal message header.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::compile(int32_t property, int32_t value, 
		    uint8_t *msg, int32_t *msglen){
  int i;
   
  // Check the property
  if(PROP_END < property){
    ROS_WARN("CANbus::compile: invalid property (%d > %d).",property,PROP_END);
    return OW_FAILURE;
  }
   
  if (property < 0) {
    ROS_WARN("CANbus::compile: property was not defined for this firmware version");
    return OW_FAILURE;
  }

  msg[0] = (uint8_t)property; // Insert the property
  msg[1] = 0;                 // To align the values for the tater's DSP
   
  // Append the value 
  for(i=2; i<6; i++){
    msg[i] = (uint8_t)(value & 0x000000FF);
    value >>= 8;
  }
   
  // Record the proper data length 
  // this was also removed from Barrett code:
  //    *msglen = (dataType[property] & 0x0007) + 2;
  *msglen = 6;
   
  return OW_SUCCESS;
}

int CANbus::read_rt(int32_t* msgid, uint8_t* msgdata, int32_t* msglen, int32_t usecs){

  int i, err;
  
  RTIME sleeptime; // time to wait for interrupts, in microseconds
  if (usecs < 10000) {
    sleeptime=200; // for short delays, sleep in 200 microsecond intervals
  } else {
    sleeptime=4000; // for longer delays, sleep for 4ms
  }
#ifdef OWD_RT
  int retrycount = usecs / sleeptime + 0.5; // round to nearest int
#endif // OWD_RT

  std::vector<canio_data> crecord;
  canio_data cdata;
  struct timeval tv;


#ifdef PEAK_CAN
  TPCANRdMsg cmsg;
  
#ifdef OWD_RT
  bool done=false;
  while (!done) {
    err=LINUX_CAN_Read_Timeout(handle,&cmsg,0);
    if (err == CAN_ERR_QRCVEMPTY) {
      if (retrycount-- > 0) {
	if (!rt_task_self()) {
	  // we're not being called from an RT context, so use regular usleep
	  usleep(sleeptime);
	} else {
	  if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	    snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	    return OW_FAILURE;
	  }
	}
	if (log_canbus_data) {
	  // record the sleep event
	  gettimeofday(&tv,NULL);
	  cdata.secs = tv.tv_sec;
	  cdata.usecs = tv.tv_usec;
	  cdata.send=false;
	  cdata.msgid = -1;  // key value for SLEEP
	  cdata.msglen = 1;
	  cdata.msgdata[0] = sleeptime;
	  for (unsigned int i=1; i<8; ++i) {
	    cdata.msgdata[i] = 0; // must pad the extra space with zeros
	  }
	  crecord.push_back(cdata);
	  candata.add(crecord);
	  crecord.clear();
	}
      } else {
	snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
	if (log_canbus_data) {
	  // record the timeout event
	  gettimeofday(&tv,NULL);
	  cdata.secs = tv.tv_sec;
	  cdata.usecs = tv.tv_usec;
	  cdata.send=false;
	  cdata.msgid = -2;  // key value for TIMEOUT
	  cdata.msglen = 1;
	  cdata.msgdata[0] = usecs;
	  for (unsigned int i=1; i<8; ++i) {
	    cdata.msgdata[i] = 0; // must pad the extra space with zeros
	  }
	  crecord.push_back(cdata);
	  candata.add(crecord);
	  crecord.clear();
	}
	return OW_FAILURE;
      }
    }
    if (err == CAN_ERR_OK) {
      done=true;
      break;
    } else {
      snprintf(last_error,200,"LINUX_CAN_Read_Timeout failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
#else // ! OWD_RT
  err = LINUX_CAN_Read_Timeout(handle,&cmsg,usecs);
  if (err == CAN_ERR_QRCVEMPTY) {
    snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
    return OW_FAILURE;
  } else if (err != CAN_ERR_OK) {
    snprintf(last_error,200,"LINUX_CAN_Read_Timeout failed: 0x%x",err);
    return OW_FAILURE;
  }
#endif // ! OWD_RT

  *msgid = cmsg.Msg.ID;
  *msglen = cmsg.Msg.LEN;
  for (i=0; i< *msglen; ++i) {
    msgdata[i] = cmsg.Msg.DATA[i];
  }

#endif // PEAK_CAN

#ifdef ESD_CAN
  CMSG cmsg;

  int zerocount(0);
  int32_t len(1);
  bool done=false;
  while (!done) {
    err=canTake(handle, &cmsg, &len);
    if ((err == NTCAN_RX_TIMEOUT)
	|| ((err == NTCAN_SUCCESS) && (len == 0))) {
      if (retrycount-- > 0) {
#ifdef OWD_RT
	if (!rt_task_self()) {
	  // we're not being called from an RT context, so use regular usleep
	  usleep(sleeptime);
	} else {
	  if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	    snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	    return OW_FAILURE;
	  }
	}
#else // ! OWD_RT
	usleep(sleeptime);	// give time for the CAN message to arrive
#endif // ! OWD_RT
	if (log_canbus_data) {
	  // record the sleep event
	  gettimeofday(&tv,NULL);
	  cdata.secs = tv.tv_sec;
	  cdata.usecs = tv.tv_usec;
	  cdata.send=false;
	  cdata.msgid = -1;  // key value for SLEEP
	  cdata.msglen = 1;
	  cdata.msgdata[0] = sleeptime;
	  for (unsigned int i=1; i<8; ++i) {
	    cdata.msgdata[i] = 0; // must pad the extra space with zeros
	  }
	  crecord.push_back(cdata);
	  candata.add(crecord);
	  crecord.clear();
	}
	len=1; // make sure we pass in the right len each time
      } else {
	if (log_canbus_data) {
	  // record the timeout event
	  gettimeofday(&tv,NULL);
	  cdata.secs = tv.tv_sec;
	  cdata.usecs = tv.tv_usec;
	  cdata.send=false;
	  cdata.msgid = -2;  // key value for TIMEOUT
	  cdata.msglen = 1;
	  cdata.msgdata[0] = usecs;
	  for (unsigned int i=1; i<8; ++i) {
	    cdata.msgdata[i] = 0; // must pad the extra space with zeros
	  }
	  crecord.push_back(cdata);
	  candata.add(crecord);
	  crecord.clear();
	}
	snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
	return OW_FAILURE;
      }
    } else if (err == NTCAN_SUCCESS) {
      done=true;
      break;
    } else {
      snprintf(last_error,200,"canTake failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
  
  if (len != 1) {
    snprintf(last_error,200,"received a message of length: %d",len);
    return OW_FAILURE;
  }

  *msgid = cmsg.id;
  *msglen = cmsg.len;
  for(i=0; i<*msglen; i++)
    msgdata[i] = cmsg.data[i];
 
#endif // ESD_CAN

  if (log_canbus_data) {
    gettimeofday(&tv,NULL);
    cdata.secs = tv.tv_sec;
    cdata.usecs = tv.tv_usec;
    cdata.send=false;
    cdata.msgid = *msgid;
    cdata.msglen = *msglen;
    for (int i=0; i<8; ++i) {
      if (i < *msglen) {
	cdata.msgdata[i] = msgdata[i];
      } else {
	cdata.msgdata[i] = 0; // must pad the extra space with zeros
      }
    }
    crecord.push_back(cdata);
    candata.add(crecord);
  }

  return OW_SUCCESS;
}

int CANbus::send_rt(int32_t msgid, uint8_t* msgdata, int32_t msglen, int32_t usecs) {

  int i;
  int32_t err;

  int32_t sleeptime;
  if (usecs < 2000) {
    sleeptime=100; // for short delays, sleep in 100 microsecond intervals
  } else {
    sleeptime=1000; // for longer delays, sleep for 1ms
  }
  int retrycount = usecs / sleeptime + 0.5;

  if (log_canbus_data) {
    std::vector<canio_data> crecord;
    canio_data cdata;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    cdata.secs = tv.tv_sec;
    cdata.usecs = tv.tv_usec;
    cdata.send=true;
    cdata.msgid = msgid;
    cdata.msglen = msglen;
    for (int i=0; i<8; ++i) {
      if (i < msglen) {
	cdata.msgdata[i] = msgdata[i];
      } else {
	cdata.msgdata[i] = 0; // must pad the extra space with zeros
      }
    }
    crecord.push_back(cdata);
    candata.add(crecord);
  }
  
  
#ifdef PEAK_CAN
  TPCANMsg msg;

  msg.ID = msgid;
  msg.MSGTYPE = MSGTYPE_STANDARD;
  msg.LEN = msglen & 0x0F;
  for (i=0; i<msglen; ++i) {
    msg.DATA[i] = msgdata[i];
  }
  
  while (((err = LINUX_CAN_Write_Timeout(handle,&msg,sleeptime)) != CAN_ERR_OK) &&
	 (retrycount-- > 0)) {
#ifdef OWD_RT
    if (!rt_task_self()) {
      // we're not being called from an RT context, so use regular usleep
      usleep(sleeptime);
    } else {
      if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	return OW_FAILURE;
      }
    }
#else // ! OWD_RT
    if ((err == CAN_ERR_BUSHEAVY) || (err==CAN_ERR_XMTFULL)
	|| (err==CAN_ERR_BUSOFF) || (err=CAN_ERR_QXMTFULL)) {
      snprintf(last_error,200,"CAN_Write failed (0x%x): write buffer full?",err);
      return OW_FAILURE;
    }
#endif // ! OWD_RT
  }
  if (err != CAN_ERR_OK) {
    snprintf(last_error,200,"canWrite failed: 0x%x",err);
    return OW_FAILURE;
  }

  return OW_SUCCESS;
#endif // PEAK_CAN


#ifdef ESD_CAN
  CMSG cmsg;

  cmsg.id = msgid;
  cmsg.len = (uint8_t)(msglen & 0x0F);
  for(i=0; i<msglen; i++)
    cmsg.data[i] = msgdata[i];
  
  int32_t len = 1;
  while (((err=canSend(handle, &cmsg, &len)) != NTCAN_SUCCESS) && 
	 (retrycount-- > 0)) {
#ifdef OWD_RT
    if (!rt_task_self()) {
      // we're not being called from an RT context, so use regular usleep
      usleep(sleeptime);
    } else {
      if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	return OW_FAILURE;
      }
    }
#else // ! OWD_RT
    usleep(sleeptime);
#endif // ! OWD_RT
  }
  if (err != NTCAN_SUCCESS) {
    snprintf(last_error,200,"canSend failed: 0x%x",err);
    return OW_FAILURE;
  }
    
  return OW_SUCCESS;
#endif // ESD_CAN
}

// Set warning/fault levels on safety puck
int CANbus::set_limits(){
   
  // Set max torque level on safety puck
  // Set the fault level to the highest of the values set in Puck.cc.
  // Set the warning level to 70% of the fault level
  max_safety_torque=0;
  for (int p=0; p<n_arm_pucks; ++p) {
    if (Puck::MAX_TRQ[p] > max_safety_torque) {
      max_safety_torque = Puck::MAX_TRQ[p];
    }
  }
  if ((set_property_rt(SAFETY_MODULE,TL1,0.7*max_safety_torque,false,15000) == OW_FAILURE) ||
      (set_property_rt(SAFETY_MODULE,TL2,max_safety_torque,false,15000) == OW_FAILURE)) {
    return OW_FAILURE;
  }

  // Set max velocity in safety puck
  // Cartesian velocity is calculated at the elbow (550mm from the shoulder joint)
  // and at the base of the hand (350mm beyond the elbow).
  // VL1 is the level at which the pendant warning indicator will light.
  // VL2 is the level at which the safety puck will IDLE the arm.
  if ((set_property_rt(SAFETY_MODULE,VL1,
		       static_cast<int32_t>(0.5 * max_cartesian_velocity * 0x1000), 
		       false, 15000) == OW_FAILURE) ||
      (set_property_rt(SAFETY_MODULE,VL2,
		       static_cast<int32_t>(max_cartesian_velocity * 0x1000),
		       false, 15000) == OW_FAILURE)) {
    return OW_FAILURE;
  }
  ROS_INFO("Setting velocity limits to %2.2f m/s (warn) and %2.2f m/s (fault)",
	   0.7*max_cartesian_velocity,
	   max_cartesian_velocity);

#ifdef SET_VOLTAGE_LIMITS
  int32_t voltlevel;
  // set appropriate high-voltage levels for battery operation
  if (get_property_rt(SAFETY_MODULE,VOLTH1,&voltlevel) == OW_FAILURE) {
    ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
    return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH1 was %d, changing to 54",voltlevel);
  if (set_property_rt(SAFETY_MODULE,VOLTH1,54,false,15000) == OW_FAILURE) {
    ROS_ERROR("CANbus::limits: set_prop failed");
    return OW_FAILURE;
  }

  if (get_property_rt(SAFETY_MODULE,VOLTH2,&voltlevel) == OW_FAILURE) {
    ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
    return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH2 was %d, changing to 57",voltlevel);
  if (set_property_rt(SAFETY_MODULE,VOLTH2,57,false,15000) == OW_FAILURE) {
    ROS_ERROR("CANbus::limits: set_prop failed");
    return OW_FAILURE;
  }
#endif

  return OW_SUCCESS;
}

int CANbus::run(){
  int r;
  r = bus_run;
  return r;
}

void CANbus::printpos(){
  for(int p=1; p<=n_arm_pucks; p++)
    ROS_DEBUG("%f",pos[p]);
}

int32_t CANbus::get_puck_state() {
  int32_t pstate;
  pstate = puck_state;
  return pstate;
}

int CANbus::request_puck_state_rt(int32_t nodeid) {
  int32_t puck1_state;
  
  received_state_flags &= ~(1 << nodeid);
  if (request_property_rt(nodeid,MODE) != OW_SUCCESS) {
    ROS_WARN("Failure while requesting MODE of puck #%d",nodeid);
    puck1_state = -1;
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::process_arm_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  int32_t nodeid, property, value;
  // extract the payload
  if(parse(msgid, msg, msglen, &nodeid, &property, &value) != OW_SUCCESS){
    //ROS_WARN("CANbus::process_arm_response_rt: parse failed: %s",last_error);
    return OW_FAILURE;
  }

  // verify the sender
  if ((nodeid < 1) || (nodeid > 7)) {
    //ROS_ERROR("CANbus::process_arm_response_rt: don't know how to handle response from puck %d", nodeid);
    return OW_FAILURE;
  }

  // check the property
  if (property == MODE) {
    received_state_flags |= (1 << nodeid);
    puck_state = value;
  } else {
    //    ROS_WARN("CANbus::process_arm_response_rt: unexpected property %d", property);
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::set_puck_group_id(int32_t nodeid) {
  // CANBUS GROUP ID CODES:
  //   0 = all motor pucks (pucks 1-7 and 11-14)
  //   1 = 4-DOF pucks (pucks 1 through 4)
  //   2 = Wrist pucks (pucks 5 through 7)
  //   4 = Arm pucks (pucks 1 through 7)
  //   5 = Hand pucks (pucks 11 through 14)
  //  10 = Force/Torque puck (puck 8)

  // each puck can be a member of up to 3 groups: a, b, and c
  
  int32_t a,b,c;
  if ((nodeid >= 1) && (nodeid <=4)) {
    // lower arm pucks
    a=0; b=1; c=4;
  } else if ((nodeid >=5) && (nodeid <=7)) {
    // wrist pucks
    a=0; b=2; c=4;
  } else if (nodeid == 8) {
    // force/torque puck
    a=10; b=10; c=10;
  } else if ((nodeid >=11) && (nodeid <=14)) {
    // handpucks
    a=0; b=5; c=5;
  }

  // check/set each group value
  int32_t group;
  if (get_property_rt(nodeid,GRPA,&group,30000) != OW_SUCCESS) {
    ROS_WARN("Could not get GRPA from puck %d",nodeid);
    return OW_FAILURE;
  }
  if (group != a) {
    ROS_WARN_NAMED("cancheck","Changing puck %d GRPA from %d to %d",
		   nodeid, group, a);
    if (set_property_rt(nodeid,GRPA,a,false) != OW_SUCCESS) {
      ROS_ERROR("FAILED to set puck %d GRPA", nodeid);
      return OW_FAILURE;
    }
  }

  if (get_property_rt(nodeid,GRPB,&group,20000) != OW_SUCCESS) {
    ROS_WARN("Could not get GRPB from puck %d",nodeid);
    return OW_FAILURE;
  }
  if (group != b) {
    ROS_WARN_NAMED("cancheck","Changing puck %d GRPB from %d to %d",
		   nodeid, group, b);
    if (set_property_rt(nodeid,GRPB,b,false) != OW_SUCCESS) {
      ROS_ERROR("FAILED to set puck %d GRPB", nodeid);
      return OW_FAILURE;
    }
  }

  if (get_property_rt(nodeid,GRPC,&group,20000) != OW_SUCCESS) {
    ROS_WARN("Could not get GRPC from puck %d",nodeid);
    return OW_FAILURE;
  }
  if (group != c) {
    ROS_WARN_NAMED("cancheck","Changing puck %d GRPC from %d to %d",
		   nodeid, group, c);
    if (set_property_rt(nodeid,GRPC,c,false) != OW_SUCCESS) {
      ROS_ERROR("FAILED to set puck %d GRPC", nodeid);
      return OW_FAILURE;
    }
  }

  return OW_SUCCESS;
}

int CANbus::ft_get_data(double *values, double *filtered_values) {
  if (!forcetorque_data || !valid_forcetorque_data) {
    return OW_FAILURE;
  }

  if (filtered_values && !valid_filtered_forcetorque_data) {
    return OW_FAILURE;
  }

  for (int i=0; i<6; ++i) {
    if (values) {
      values[i] = forcetorque_data[i];
    }
    if (filtered_values) {
      filtered_values[i] = filtered_forcetorque_data[i];
    }
  }
  return OW_SUCCESS;
}

//return forcetorque state flag
int CANbus::ft_get_state() {
  int flag;
  flag = valid_forcetorque_flag;
  return flag;
}



int CANbus::ft_tare() {

  // Set the count to -2 initially.  Once the request is sent we'll set it
  // to -1.  We'll still through out the first response received after the
  // request (which will increment the counter to zero), and then start
  // accumulating readings after that until we reach ft_tare_values_to_average.
  force_tare_values_collected
    = torque_tare_values_collected
    = -2;
  
  for (int i=0; i<6; ++i) {
    ft_tare_avg[i]=0;
  }
  valid_forcetorque_data=false;
  valid_filtered_forcetorque_data=false;

  // reset the filters
  mutex_lock(&ft_mutex);
  ft_force_filter.reset();
  ft_torque_filter.reset();
  mutex_unlock(&ft_mutex);

  if (hand_set_property(8,FT,0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  
  struct timeval tv;
  gettimeofday(&tv,NULL);
  int waitsecs = tv.tv_sec + 10;
  while (!valid_forcetorque_data && (tv.tv_sec < waitsecs)) {
    usleep(100000); // might return sooner than 100ms...
    gettimeofday(&tv,NULL); // so measure the actual time.
  }
  if (!valid_forcetorque_data) {
    ROS_WARN("Taring of FT sensor failed after waiting for 10 seconds");
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::tactile_get_data(float *f1, float *f2, float *f3, float *palm) {
  if (tactile_data && valid_tactile_data) {
    if (f1) {
      memcpy(f1,tactile_data,24*sizeof(float));
    }
    if (f2) {
      memcpy(f2,tactile_data+24,24*sizeof(float));
    }
    if (f3) {
      memcpy(f3,tactile_data+48,24*sizeof(float));
    }
    if (palm) {
      memcpy(palm,tactile_data+72,24*sizeof(float));
    }
    return OW_SUCCESS;
  }
  return OW_FAILURE;
}


int CANbus::hand_set_property(int32_t id, int32_t prop, int32_t val) {
  CANmsg msg;
  msg.nodeid=id;
  msg.property=prop | 0x80; // set the high bit to 1
  msg.value=val;

#ifdef OWD_RT
  // use mutex to protect against re-entry from multiple service calls
  mutex_lock(&hand_cmd_mutex);
  ssize_t bytes = ::write(handpipe_fd,&msg,sizeof(CANmsg));
  mutex_unlock(&hand_cmd_mutex);
  if (bytes < sizeof(CANmsg)) {
    if (bytes < 0) {
      ROS_ERROR_NAMED("can_bh280","Error writing to hand message pipe: %zd",bytes);
    } else {
      ROS_ERROR_NAMED("can_bh280","Incomplete write of data to hand message pipe: only %zd of %ld bytes written",
		      bytes,sizeof(CANmsg));
    }
    return OW_FAILURE;
  }
#else // ! OWD_RT

  if (!mutex_lock(&hand_queue_mutex)) {
    hand_command_queue.push(msg);
    mutex_unlock(&hand_queue_mutex);
  } else {
    ROS_ERROR_NAMED("can_bh280","Unable to acquire hand_queue_mutex; hand_set_property Node %d Prop %d=%d failed.",id,prop,val);
    return OW_FAILURE;
  }

#endif // ! OWD_RT

  return OW_SUCCESS;
}
 
int CANbus::hand_get_property(int32_t id, int32_t prop, int32_t *value) {
  CANmsg msg;
  msg.nodeid=id;
  msg.property=prop;
  msg.value=0;
  get_property_expecting_id = id;
  get_property_expecting_prop = prop;

  if (mutex_lock(&hand_cmd_mutex)) {
    ROS_ERROR_NAMED("can_bh280","Could not lock hand command mutex");
    get_property_expecting_id = 0;
    return OW_FAILURE;
  }

#ifdef OWD_RT

  int bytes = ::write(handpipe_fd,&msg,sizeof(CANmsg));
  if (bytes < sizeof(CANmsg)) {
    if (bytes < 0) {
      ROS_ERROR_NAMED("can_bh280","Error writing to hand message pipe: %d", bytes);
    } else {
      ROS_ERROR_NAMED("can_bh280","Incomplete write of data to hand message pipe: only %d of %ld bytes written",
		      bytes,sizeof(CANmsg));
    }
    mutex_unlock(&hand_cmd_mutex);
    get_property_expecting_id = 0;
    return OW_FAILURE;
  }
  // now read from the pipe; the read will return as soon as the data is available
  bytes = ::read(handpipe_fd, &msg, sizeof(CANmsg));

  if (bytes < 0) {
    ROS_ERROR_NAMED("can_bh280","Error reading data from hand message pipe: %d", errno);
    mutex_unlock(&hand_cmd_mutex);
    get_property_expecting_id = 0;
    return OW_FAILURE;
  }
  if (size_t(bytes) < sizeof(CANmsg)) {
    ROS_ERROR_NAMED("can_bh280","Incomplete read of message from hand message pipe: expected %ld but got %d bytes",
		    sizeof(CANmsg),bytes);
    get_property_expecting_id = 0;
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }

#else // ! OWD_RT
  mutex_lock(&hand_queue_mutex);
  hand_command_queue.push(msg);
  mutex_unlock(&hand_queue_mutex);

  // wait for the response
  bool done=false;
  int retry=20;
  do {
    usleep(1000);
    mutex_lock(&hand_queue_mutex);
    if (hand_response_queue.size() > 0) {
      msg = hand_response_queue.front();
      hand_response_queue.pop();
      done=true;
    }
    mutex_unlock(&hand_queue_mutex);
  } while (!done && ros::ok() && (--retry > 0));
  get_property_expecting_id = 0;
  if (!ros::ok()) {
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }
  if (retry == 0) {
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }
#endif // ! OWD_RT

  mutex_unlock(&hand_cmd_mutex);

  if (msg.nodeid != id) {
    ROS_ERROR_NAMED("can_bh280","Expecting response from hand puck %d but got message from puck %d",
	     id,msg.nodeid);
    ROS_DEBUG_NAMED("can_bh280","Property was %d, value was %d",
		    msg.property, msg.value);
    return OW_FAILURE;
  }
  if (msg.property != prop) {
    ROS_ERROR_NAMED("can_bh280","Asked hand puck %d for property %d but got property %d",id,prop,msg.property);
    ROS_DEBUG_NAMED("can_bh280","Node ID was %d, value was %d",
		    msg.nodeid, msg.value);
    return OW_FAILURE;
  }
  *value = msg.value;
  return OW_SUCCESS;
}

int CANbus::hand_activate(int32_t *nodes) {
  for (int32_t nodeid=11; nodeid<15; ++nodeid) {
    if (nodes[nodeid] == STATUS_RESET) {
      ROS_DEBUG_NAMED("can_bh280","Waking up puck %d...",nodeid);
      if (wake_puck(nodeid) != OW_SUCCESS) {
	ROS_WARN_NAMED("can_bh280","Could not wake hand puck %d",nodeid);
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("can_bh280","done");
    } else {
      ROS_DEBUG_NAMED("can_bh280","setting puck %d to idle mode...",nodeid);
      // this was set to CONTROLLER_IDLE originally
      if(set_property_rt(nodeid, MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
	ROS_WARN("Failed to set MODE=MODE_IDLE on puck %d",nodeid);
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("can_bh280","done");
    }

    if (set_puck_group_id(nodeid) != OW_SUCCESS) {
      ROS_WARN("set_puck_group_id(%d) failed", nodeid);
      return OW_FAILURE;
    }
  }
  return OW_SUCCESS;
}

int CANbus::request_hand_state_rt() {
  // request the puck and motor temps
  if (request_property_rt(GROUPID(5),TEMP) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280",
		   "Failed to request TEMP from hand pucks: %s",
		   last_error);
    return OW_FAILURE;
  }
  if (request_property_rt(GROUPID(5),THERM) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280",
		   "Failed to request THERM from hand pucks: %s",
		   last_error);
    return OW_FAILURE;
  }
  
  // request state from hand pucks
  if (request_property_rt(GROUPID(5),MODE) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280",
		   "Failed to request MODE from hand pucks: %s",
		   last_error);
    return OW_FAILURE;
  }

  // if we had recently sent a move command, record the fact that
  // we have now requested the state and subsequent state responses
  // will be valid
  if (hand_motion_state_sequence == 2) {
    hand_motion_state_sequence=0;
  }

  return OW_SUCCESS;
}

int CANbus::process_hand_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  int32_t nodeid, property, value;
  // extract the payload
  if(parse(msgid, msg, msglen, &nodeid, &property, &value) != OW_SUCCESS){
    //    ROS_WARN("CANbus::process_hand_response_rt: parse failed: %s",last_error);
    return OW_FAILURE;
  }

  // verify the sender
  if ((nodeid < 11) || (nodeid > 14)) {
    //    ROS_ERROR("CANbus::process_hand_response_rt: don't know how to handle response from puck %d", nodeid);
    return OW_FAILURE;
  }

  // check the property
  if (property == MODE) {
    if ( hand_motion_state_sequence != 0 ) {
      // either the hand_command_queue has not delivered the move request
      // to the CANbus yet, or a subsequent state request has not yet been
      // made, so we have to ignore this response as out-of-date
      return OW_SUCCESS;
    }
    received_state_flags |= (1 << nodeid);
    int32_t mode=value;
    hand_puck_mode[nodeid-11]=mode;

    switch (mode) {
    case MODE_IDLE: // fall through
    case MODE_PID:
      if (finger_hi_pending[nodeid-11]) {
	// we had been waiting for the finger to finish its HI
	finger_hi_pending[nodeid-11]=false;
	handstate[nodeid-11] = HANDSTATE_DONE;
	if (nodeid != 14) {
	  hand_breakaway[nodeid-10] = false;
	}
	break;
      }
      if (nodeid == 14) {
	handstate[nodeid-11] = HANDSTATE_DONE;
	break;
      }
      if (labs(hand_positions[nodeid-10] - hand_goal_positions[nodeid-10]) > 600) {
	if (apply_squeeze[nodeid-11]) {
	  apply_squeeze[nodeid-11]=false;

	  if (hand_breakaway[nodeid-10]) {
	    // don't apply the squeeze if we're in breakaway, because it
	    // just causes the distal link to press against its stop if
	    // we're closing, or can cause repeated failures if opening.
	    return OW_SUCCESS;
	  }

	  // Reapply the command with no torque
	  // stopping and with a torque limit low enough that it can keep
	  // applying the force indefinitely without overheating.
	  handstate[nodeid-11] = HANDSTATE_MOVING;
	  if (hand_set_property(nodeid,MT,hand_sustained_torque) != OW_SUCCESS) {
	    return OW_FAILURE;
	  }
	  if (hand_set_property(nodeid,TSTOP,0) != OW_SUCCESS) {
	    return OW_FAILURE;
	  }
	  // the value of E was already set, so we just change MODE
	  if (hand_set_property(nodeid,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
	    return OW_FAILURE;
	  }
	} else {
	  handstate[nodeid-11] = HANDSTATE_DONE;
	}
      } else {
	handstate[nodeid-11] = HANDSTATE_DONE;
	apply_squeeze[nodeid-11]=false;
      }	
      break;

    case MODE_TRAPEZOID:
      // if near goal, state=done
      // if changing, state=moving
      // otherwise state=stalled
      if ((labs(hand_positions[nodeid-10] - hand_goal_positions[nodeid-10])
	   < 600) && !apply_squeeze[nodeid-11]) {
	  // if we're within 600 encoder ticks of the goal, and we've
	  // already applied the squeeze, call it done
	  handstate[nodeid-11] = HANDSTATE_DONE;
	  // could try switching back to MODE_IDLE/MODE_PID here
      } else {
	if (encoder_changed[nodeid-11]) {
	  handstate[nodeid-11] = HANDSTATE_MOVING;
	} else {
	  handstate[nodeid-11] = HANDSTATE_STALLED;
	}
      }
      break;
      
    case MODE_VELOCITY:
    case MODE_TORQUE:
      handstate[nodeid-11] = HANDSTATE_MOVING;
      break;

    }
    return OW_SUCCESS;

  } else if (property == SG) {
    // 12-bit straingauge data
    if (msglen<4) {
      //      ROS_WARN("Not enough bytes for straingauge data (%d)",msglen);
      return OW_FAILURE;
    }
    int32_t value = (msg[3]<<8) + msg[2] - 2048; // center on zero
    hand_strain[nodeid-10]=value;
    hand_compliant_callback(nodeid-10, value);
    return OW_SUCCESS;

  } else if ((msgid & 0x41F) == 0x408) { // group 8
    // Tactile Top 10 data
    // The 8 data fields look like this:
    //  [HighSSSS] [Mid SSSS] [Low SSSS] [AAAABBBB] [CCCCDDDD] [EEEEFFFF] [GGGGHHHH] [JJJJKKKK]
    //  SSSS = 24-bit sensor map, exactly 10 bits will be '1', the rest '0'
    //  AAAA = 4-bit value of the lowest sensor ID in the map (N/cm2)
    //  BBBB = 4-bit value of the next lowest sensor ID in the map (N/cm2)
    if (msglen != 8) {
      //      ROS_ERROR("Bad TACTILE response: payload was only %d bytes instead of 8",
      //		msglen);
      return OW_FAILURE;
    }
    uint32_t bitmap = msg[0] << 16 | msg[1] << 8 | msg[2];
    uint8_t top10[10];
    top10[0]=msg[3] >> 4;  // upper 4 bits
    top10[1]=msg[3] & 0xF; // lower 4 bits
    top10[2]=msg[4] >> 4;  // etc
    top10[3]=msg[4] & 0xF;
    top10[4]=msg[5] >> 4;
    top10[5]=msg[5] & 0xF;
    top10[6]=msg[6] >> 4;
    top10[7]=msg[6] & 0xF;
    top10[8]=msg[7] >> 4;
    top10[9]=msg[7] & 0xF;
    
    unsigned int top10_id=0;
    unsigned int offset = (nodeid-11)*24;
    for (unsigned int i=0; i<24; ++i) {
      if (bitmap & 0x800000) {
	// if the leftmost bit is set, it indicates that the next top10
	// value corresponds to sensor number i
	tactile_data[i+offset] = top10[top10_id++];
	if (top10_id > 9) {
	  break; // all done, don't need to keep searching
	}
      }
      bitmap <<= 1;  // left shift by 1 bit
    }
    static uint8_t top10_received(0);
    top10_received |= 1 << (nodeid-11);
    if ((top10_received & 0xF) == 0xF) {
      valid_tactile_data=true;
    }

  } else if ((msgid & 0x41F) == 0x409) { // group 9
    // Tactile hires data
    // It will take five CANbus responses to convey the entire 24-cell array.
    // Each of the five will contain 5 12-bit values
    // The 8 data fields look like this:
    // [NNNNAAAA] [aaaaaaaa] [BBBBbbbb] [bbbbCCCC] [cccccccc] [DDDDdddd] [ddddEEEE] [eeeeeeee]
    //  NNNN = 4-bit sensor group: 0 = sensors 1-5, 1 = sensors 6-10, etc.
    //  AAAAaaaaaaaa = 12-bit sensor data from first sensor in group, divide by 256 to get N/cm2
    //  BBBBbbbbbbbb = 12-bit sensor data from second sensor in group, divide by 256 to get N/cm2
    if (msglen != 8) {
      //      ROS_ERROR("Bad TACTILE response: payload was only %d bytes instead of 8",
      //		msglen);
      return OW_FAILURE;
    }
    int sensor_group=msg[0]>>4;
    int offset = (nodeid-11)*24 + sensor_group * 5;
    tactile_data[offset]  = (msg[0] & 0xF) + msg[1] / 256.0;
    tactile_data[offset+1]= ((msg[2]<<4) + (msg[3]>>4)) / 256.0;
    tactile_data[offset+2]= (msg[3] & 0xF) + msg[4] / 256.0;
    tactile_data[offset+3]= ((msg[5]<<4) + (msg[6]>>4)) / 256.0;
    if (sensor_group < 4) {
      // there are only 24 sensors, so only use the 5th member
      // of the group if we aren't processing the final group
      tactile_data[offset+4]= (msg[6] & 0xF) + msg[7] / 256.0;
    }
    static uint8_t hires_received(0);
    hires_received |= 1 << (nodeid-11);
    if ((hires_received & 0xF) == 0xF) {
      valid_tactile_data=true;
    }
  } else if (property == TEMP) {
    // Puck temperature
    hand_puck_temp[nodeid-11]=value;
  } else if (property == THERM) {
    // Motor temperature
    hand_motor_temp[nodeid-11]=value;
  } else {
    // unexpected message type
    snprintf(last_error,200,"Unexpected property response received (nodeid = %d, property=%d",nodeid, property);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::process_get_property_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  // it's a response to a GetHandProperty request, so send
  // the result back through the hand queue/pipe

  int32_t nodeid, property, value;
  // extract the payload
  if(parse(msgid, msg, msglen, &nodeid, &property, &value) != OW_SUCCESS){
    return OW_FAILURE;
  }
  
  CANmsg handmsg;
  handmsg.nodeid = nodeid;
  handmsg.property = property;
  handmsg.value = value;
#ifdef OWD_RT
  ssize_t bytecount = rt_pipe_write(&handpipe,&handmsg,sizeof(CANmsg),P_NORMAL);
  if (size_t(bytecount) < sizeof(CANmsg)) {
    if (bytecount < 0) {
      snprintf(last_error,200,"Error writing to hand message pipe: %zd",bytecount);
    } else {
      snprintf(last_error,200,"Incomplete write to hand message pipe: only %zd of %ld bytes were written",
	       bytecount,sizeof(CANmsg));
    }
    return OW_FAILURE;
  }
#else // ! OWD_RT
  hand_response_queue.push(handmsg);
#endif // ! OWD_RT
  return OW_SUCCESS;
}

int CANbus::process_safety_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen) {
  int32_t nodeid, property, value;
  // extract the payload
  if(parse(msgid, msg, msglen, &nodeid, &property, &value) != OW_SUCCESS){
    //    ROS_WARN("CANbus::process_safety_response: parse failed: %s",last_error);
    return OW_FAILURE;
  }

  // verify the sender
  if (nodeid != 10) {
    //    ROS_WARN("CANbus::process_safety_response: node id %d is not the safety puck", nodeid);
    return OW_FAILURE;
  }
  
  // nothing else is handled for the safety puck
  return OW_SUCCESS;
}

int CANbus::request_strain_rt() {
  uint8_t  msg[8];

  // Compile the packet
  msg[0] = (uint8_t)SG;

  if(send_rt(GROUPID(5), msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::request_strain_rt: send failed: %s",last_error);
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::send_finger_reset(int32_t nodeid) {
  if ((nodeid < 11) || (nodeid > 14)) {
    return OW_FAILURE;
  }
  
  // note the fact that a state change is in progress and we should ignore
  // any old state messages until a new request is sent
  hand_motion_state_sequence = 2;

  // set a flag so that the process_hand_response_rt function knows what
  // to look for
  finger_hi_pending[nodeid-11] = true;

  // set the state
  handstate[nodeid-11] = HANDSTATE_UNINIT;

  // send the HI to this finger
  if (set_property_rt(nodeid,CMD,CMD_HI) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Error sending HI to hand puck %d",nodeid);
    return OW_FAILURE;
  }
  
  return OW_SUCCESS;
}

int CANbus::wait_for_finger_reset(int32_t nodeid) {
  // now wait for the change in mode
  int32_t mode = MODE_VELOCITY;
  usleep(50000); // give it a chance
  int32_t sleepcount=0;
  // wait until the puck returns to MODE_IDLE (normal for pucks 11-13) or
  //   MODE_PID (normal for puck 14)
  while ((get_property_rt(nodeid,MODE,&mode,1000000) == OW_SUCCESS) &&
	 (mode != MODE_IDLE) && (mode != MODE_PID) &&
	 ros::ok()) {
    usleep(50000);
    if (++sleepcount ==100) {
      ROS_WARN_NAMED("can_bh280","Still waiting for finger %d to finish HI; mode is %d", nodeid, mode);
      sleepcount=0;
    }
  }
  finger_hi_pending[nodeid-11] = false;
  hand_motion_state_sequence = 0;

  if (!ros::ok()) {
    return OW_FAILURE;
  }
  if ((mode != MODE_IDLE) && (mode != MODE_PID)) {
    ROS_WARN_NAMED("can_bh280","No response within 5s from finger puck %d while waiting for HI",nodeid);
    return OW_FAILURE;
  }

  // set the state
  handstate[nodeid-11] = HANDSTATE_DONE;
  return OW_SUCCESS;
}

int CANbus::hand_reset() {
  // Reset Strategy (from Barrett):
  //    Repeat 3 times{
  //       Open F1
  //       Open F2
  //       Open F3
  //    }
  //    Open F4
  bool ready=true;
  for (int nodeid=11; nodeid<=13; ++nodeid) {
    int32_t ct;
    if (get_property_rt(nodeid,CT,&ct,4000) != OW_SUCCESS) {
      ROS_ERROR("Could not get CT property from hand puck %d",
		nodeid);
      return OW_FAILURE;
    }
    if (ct != finger_radians_to_encoder(2.4)) {
      ready=false;
      break;
    }
  }
  
  if (ready) {
    // all the fingers have already been HI'd
    for (unsigned int i=0; i<4; ++i) {
      handstate[i]=HANDSTATE_DONE;
    }

    if (tactile_data) {
      if (configure_tactile_sensors() != OW_SUCCESS) {
	ROS_ERROR("Could not initialize Tactile Sensors");
	return OW_FAILURE;
      }
    }
    
    return OW_SUCCESS;
  }
  
  ROS_FATAL("Please move the hand to a safe position and hit <RETURN> to reset the hand");
  char *line=NULL;
  size_t linelen = 0;
  linelen = getline(&line,&linelen,stdin);
  free(line);
  
  for (unsigned int attempts =0; attempts < 3; ++attempts) {
    // F1-F3
    for (int32_t nodeid=11; nodeid<14; ++nodeid) {
      ROS_INFO_NAMED("can_bh280", "Resetting finger puck %d", nodeid);
      if ((send_finger_reset(nodeid) != OW_SUCCESS) &&
	  (attempts == 2)) {
	ROS_WARN_NAMED("can_bh280","Failed to send reset to finger puck %d",nodeid);
	handstate[nodeid-11] = HANDSTATE_UNINIT;
	return OW_FAILURE;
      }
      if (wait_for_finger_reset(nodeid) != OW_SUCCESS) {
	return OW_FAILURE;
      }
	
    }
  }
  // F4
  ROS_INFO_NAMED("can_bh280", "Resetting finger puck 14");
  if (send_finger_reset(14) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Failed to send reset to finger puck 14");
    handstate[3] = HANDSTATE_UNINIT;
    return OW_FAILURE;
  }
  if (wait_for_finger_reset(14) != OW_SUCCESS) {
    return OW_FAILURE;
  }

  // Set the torque stop value to reduce heating on stall
  for (int32_t nodeid=11; nodeid<=13; ++nodeid) {
    if (set_property_rt(nodeid,TSTOP,50) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    usleep(100);
    // F1 to F3 have HOLD=0 because they're not backdrivable
    if (set_property_rt(nodeid,HOLD,0) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    usleep(100);
    // Set the CT so that torque is cut after 2.4 radians
    if (set_property_rt(nodeid,CT,finger_radians_to_encoder(2.4)) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }
  if (set_property_rt(14,TSTOP,200) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  usleep(100);
  // F4 has HOLD=1 so that it won't flop around
  if (set_property_rt(14,HOLD,1) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  usleep(100);
  // Reduce the gains on the fingers for smoother operation
  for (int32_t nodeid=11; nodeid<=13; ++nodeid) {
    if (set_property_rt(nodeid,KP,125) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    if (set_property_rt(nodeid,KD,1250) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }
  
  // Pull the fingers away from their open stop while we have
  // the default MT (max torque)
  for (int32_t nodeid=11; nodeid<=13; ++nodeid) {
    if (hand_set_property(nodeid,E,finger_radians_to_encoder(0.05)) 
	!= OW_SUCCESS) {
      return OW_FAILURE;
    }
    if (hand_set_property(nodeid,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }

  if (tactile_data) {
    if (configure_tactile_sensors() != OW_SUCCESS) {
      ROS_ERROR("Could not initialize Tactile Sensors");
      return OW_FAILURE;
    }
  }
  for (unsigned int i=0; i<4; ++i) {
    handstate[i] = HANDSTATE_DONE;
  }
  return OW_SUCCESS;
}

int CANbus::configure_tactile_sensors() {
  for (int32_t nodeid=11; nodeid<=14; ++nodeid) {
    if (tactile_top10) {
      if (set_property_rt(nodeid,TACT,1) != OW_SUCCESS) {
	return OW_FAILURE;
      }
    } else {
      if (set_property_rt(nodeid,TACT,2) != OW_SUCCESS) {
	return OW_FAILURE;
      }
    }
  }
  
  // it appears that sending the SET TACT message triggers a
  // tactile data response to be sent, just as if we had done
  // a GET TACT.  we need to read these responses now so that 
  // they don't linger on the bus
  
  // first, wait for pucks to process the request
  usleep(20000);
  int tactile_responses;
  if (tactile_top10) {
    tactile_responses=4;
  } else {
    tactile_responses=20;
  }
  // now, try to read the responses
  for (int i=0; i<tactile_responses; ++i) {
    uint8_t  msg[8];
    int32_t msgid, msglen;
    if (read_rt(&msgid, msg, &msglen,5000) == OW_FAILURE) {
      return OW_FAILURE;
    }
  }
  return OW_SUCCESS;
}

int CANbus::hand_move(std::vector<double> p) {
  if (p.size() != 4) {
    ROS_ERROR_NAMED("bhd280", "hand_move requires 4 position arguments");
    return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("bhd280", "executing hand_move");

  // First set the finger pucks back to the higher-torque threshold
  // with TSTOP enabled so that they stop when done/stalled
  for (int32_t node=11; node<=13; ++node) {
    if (hand_set_property(node,TSTOP,150) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    if (hand_set_property(node,MT,hand_initial_torque) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }

  // Now set the endpoints and issue the move command
  for (int32_t i=0; i<3; ++i) {
    // make sure we don't try to move beyond the 0 to 2.4 range
    if (p[i] < 0) {
      p[i]=0;
    }
    if (p[i] > 2.4) {
      p[i]=2.4;
    }
    hand_goal_positions[i+1] = finger_radians_to_encoder(p[i]);
    if (hand_set_property(11+i,E,hand_goal_positions[i+1]) 
	!= OW_SUCCESS) {
      return OW_FAILURE;
    }
  }
  // set the spread position
  hand_goal_positions[4] = spread_radians_to_encoder(p[3]);
  if (hand_set_property(14,E,hand_goal_positions[4])
      != OW_SUCCESS) {
    return OW_FAILURE;
  }

  // record the fact that a motion request is in progress, so we should
  // ignore any state responses that were made between now and when the
  // move command actually makes it to the pucks
  hand_motion_state_sequence = 1;

  // send the move command
  if (hand_set_property(GROUPID(5),MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
    return OW_FAILURE;
  }

  if (squeeze_after_stalling) {
    for (unsigned int i=0; i<4; ++i) {
      // record the fact that once the hand stops, we want to keep
      // applying pressure.  this helps to ensure that even if a gripped
      // object slips, we will still adjust until we reach the goal position.
      apply_squeeze[i]=true;
    }
  }

  for (unsigned int i=0; i<4; ++i) {
    handstate[i] = HANDSTATE_MOVING;
    encoder_changed[i] = 6;
  }
  received_state_flags &= ~(0x7800); // clear the four hand bits
  return OW_SUCCESS;
}

int CANbus::hand_velocity(const std::vector<double> &v) {
  ROS_DEBUG_NAMED("bhd280", "executing hand_velocity");

  // turn off any pending squeezes remaining from a previous position move
  for (int i=0; i<4; ++i) {
    apply_squeeze[i]=false;
  }

  if (hand_set_property(11,V,finger_radians_to_encoder(v[0])/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(12,V,finger_radians_to_encoder(v[1])/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(13,V,finger_radians_to_encoder(v[2])/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(14,V,spread_radians_to_encoder(v[3])/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  
  if (v[0] != 0.0) {
    handstate[0] = HANDSTATE_MOVING;
    encoder_changed[0] = 6;
  }
  if (v[1] != 0.0) {
    handstate[1] = HANDSTATE_MOVING;
    encoder_changed[1] = 6;
  }
  if (v[2] != 0.0) {
    handstate[2] = HANDSTATE_MOVING;
    encoder_changed[2] = 6;
  }
  if (v[3] != 0.0) {
    handstate[3] = HANDSTATE_MOVING;
    encoder_changed[3] = 6;
  }

  // we send the MODE_VELOCITY to all fingers, even ones that might have
  // v=0, because they might have been previously moving and this will
  // be the way to stop them.
  if (hand_set_property(GROUPID(5),MODE,MODE_VELOCITY) != OW_SUCCESS) {
    return OW_FAILURE;
  }

  // record the fact that a motion request is in progress, so we should
  // ignore any state responses that were made between now and when the
  // move command actually makes it to the pucks
  hand_motion_state_sequence = 1;

  return OW_SUCCESS;
}
 
int CANbus::hand_torque(const std::vector<double> &t) {
  ROS_DEBUG_NAMED("bhd280", "executing hand_torque");

  for (unsigned int f=0; f<4; ++f) {
    if (t[f] != 0) {
      if ((hand_set_property(11+f,MODE,MODE_TORQUE) != OW_SUCCESS) ||
	  (hand_set_property(11+f,T,t[f]) != OW_SUCCESS)) {
	ROS_ERROR_NAMED("bhd280","could not set torque for F%d",f);
	return OW_FAILURE;
      }
      handstate[f] = HANDSTATE_MOVING;
      encoder_changed[f] = 6;
    }
  }

  // record the fact that a motion request is in progress, so we should
  // ignore any state responses that were made between now and when the
  // move command actually makes it to the pucks
  hand_motion_state_sequence = 1;

  return OW_SUCCESS;
}

int CANbus::hand_relax() {
  if (hand_set_property(GROUPID(5),MODE,MODE_IDLE) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  for (unsigned int i=0; i<4; ++i) {
    handstate[i] = HANDSTATE_DONE;
  }
  return OW_SUCCESS;
}

int CANbus::hand_set_speed(const std::vector<double> &v) {
  if (v.size() != 4) {
    return OW_FAILURE;
  }
  // three fingers
  for (unsigned int i=0; i<3; ++i) {
    int32_t speed = finger_radians_to_encoder(v[i])/1000;
    if (speed == 0) {
      speed =1; // need a minimum speed of 1 or there is no limit
    }
    if (hand_set_property(11+i,MV,speed) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }
  // spread
  int32_t speed = spread_radians_to_encoder(v[3])/1000;
  if (speed == 0) {
    speed =1; // need a minimum speed of 1 or there is no limit
  }
  if (hand_set_property(14,MV,speed) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::hand_finger_compliant(const bool enable, const int32_t& strain)
{
    // JUST DO FINGER 1 FOR NOW

    m_compliantFinger    = enable;
    m_compliantStrain    = strain;
    m_compliantDeadband  = 50;
    m_compliantReference = hand_strain[1];

    //Set the T-stop to zero
	if (hand_set_property(11,TSTOP,100) != OW_SUCCESS) {
   	  ROS_ERROR_NAMED("bhd280","could not set tstop");
      return OW_FAILURE;
    }

    //Set the max torque to 1700
	if (hand_set_property(11,MT,1700) != OW_SUCCESS) {
   	  ROS_ERROR_NAMED("bhd280","could not set mt");
      return OW_FAILURE;
    }
 
    return OW_SUCCESS;
}
 
int CANbus::hand_compliant_callback(const int& fingerId, const int32_t& strain)
{
    if(!m_compliantFinger) {
        return OW_SUCCESS;
    }

    if(fingerId != 1) {
        return OW_SUCCESS;
    }

    if  (strain > (m_compliantStrain+m_compliantDeadband)) {
        ROS_INFO("OPEN %d, %d", fingerId,  strain);
        if (hand_set_property(11,V,100) != OW_SUCCESS) {
          ROS_INFO("OPEN FAILED");
          return OW_FAILURE;
        }
    } else if(strain < (m_compliantStrain-m_compliantDeadband)) {
        ROS_INFO("CLOSE %d, %d", fingerId,  strain);
        if (hand_set_property(11,V,-100) != OW_SUCCESS) {
          ROS_INFO("CLOSE FAILED");
          return OW_FAILURE;
        }
    } else {
        ROS_INFO("MAINTAIN %d, %d", fingerId,  strain);
    }

    return OW_SUCCESS;
}

int CANbus::hand_get_positions(double &p1, double &p2, double &p3, double &p4) {
  p1 = finger_encoder_to_radians(hand_positions[1]);
  p2 = finger_encoder_to_radians(hand_positions[2]);
  p3 = finger_encoder_to_radians(hand_positions[3]);
  p4 = spread_encoder_to_radians(hand_positions[4]);
  return OW_SUCCESS;
}
 
int CANbus::hand_get_inner_links(double &l1, double &l2, double &l3) {
  if (ignore_breakaway_encoders) {
    return OW_FAILURE;
  }
  for (int i=0; i<3; ++i) {
    if (hand_inner_links[i] == -12345) {
      // we haven't received data from the hand yet
      return OW_FAILURE;
    }
  }
  l1 = hand_inner_links[1];
  l2 = hand_inner_links[2];
  l3 = hand_inner_links[3];
  return OW_SUCCESS;
}

int CANbus::hand_get_outer_links(double &l1, double &l2, double &l3) {
  if (ignore_breakaway_encoders) {
    return OW_FAILURE;
  }
  for (int i=0; i<3; ++i) {
    if (hand_outer_links[i] == -12345) {
      // we haven't received data from the hand yet
      return OW_FAILURE;
    }
  }
  l1 = hand_outer_links[1];
  l2 = hand_outer_links[2];
  l3 = hand_outer_links[3];
  return OW_SUCCESS;
}

int CANbus::hand_get_breakaway(bool &b1, bool &b2, bool &b3) {
  if (ignore_breakaway_encoders) {
    return OW_FAILURE;
  }
  for (int i=0; i<3; ++i) {
    if ((hand_inner_links[i] == -12345) ||
	(hand_outer_links[i] == -12345)) {
      // breakaway detection is only valid if the hand is
      // sending secondary encoder values
      return OW_FAILURE;
    }
  }
  b1 = hand_breakaway[1];
  b2 = hand_breakaway[2];
  b3 = hand_breakaway[3];
  return OW_SUCCESS;
}

int CANbus::hand_get_strain(double &s1, double &s2, double &s3) {
  s1 = hand_strain[1];
  s2 = hand_strain[2];
  s3 = hand_strain[3];
  return OW_SUCCESS;
}

int CANbus::hand_get_state(int32_t *state) {
  if (state != NULL) {
    for (unsigned int i=0; i<4; ++i) {
      state[i]=handstate[i];
    }    
  
  }
  return OW_SUCCESS;
}

double CANbus::finger_encoder_to_radians(int32_t enc) {
  // encoder range: 0 to 199,111.1
  // degree range: 0 to 140
  return  ((double)enc / 199111.1) * 140.0 * 3.1416/180.0;
}

int32_t CANbus::finger_radians_to_encoder(double radians) {
  return(radians * 180.0/3.1416 / 140.0 * 199111.1);
}

double CANbus::spread_encoder_to_radians(int32_t enc) {
  // encoder range: 0 to 35840
  // degree range: 0 to 180
  return ((double)enc / 35840.0) * 180.0 * 3.1416/180.0;
}

int32_t CANbus::spread_radians_to_encoder(double radians) {
  return(radians * 180.0/3.1416 / 180.0 * 35840.0);
}


void CANbus::update_link_positions(unsigned int f) {
  // INNER LINK
  // encoder range: 0 to 199,111.1
  // degree range: 0 to 140
  // need a mysterious 2.5 factor
  hand_inner_links[f] = (double)hand_secondary_positions[f] / 199111.1 * 140.0 * 3.1416/180.0 * 2.50;

  // OUTER LINK
  // Use formula from Barrett documentation that calculates link angle
  // relative to the palm plane, then subtract out the inner link angle to
  // get the angle of the outer link relative to the inner link
  const double outer_offset = 42.0 * 3.1416 / 180.0; // 42-degrees
  hand_outer_links[f] = (double)hand_positions[f] / (4096.0*375/4) * 360.0 * 3.1416 / 180.0 + outer_offset - hand_inner_links[f];

  return;
}

void CANstats::rosprint() {
#ifdef RT_STATS
  ROS_DEBUG_NAMED("times","CANbus::send %2.1fms per group (2 groups)",
		  cansend_time);
  ROS_DEBUG_NAMED("times","CANbus::read: send=%2.1fms, read=%2.1fms",
		  canread_sendtime, canread_readtime);
  ROS_DEBUG_NAMED("times","CANbus::read: bad packets = %d",
		  canread_badpackets);
  canread_badpackets = 0;
#endif
}
  
RTIME CANbus::time_now_ns() {
#ifdef OWD_RT
   return rt_timer_ticks2ns(rt_timer_read());
#else // ! OWD_RT
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return (tv.tv_sec * 1e9 + tv.tv_usec * 1e3);
#endif // ! OWD_RT
 }

#define DEFPROP(s,x) s=x; propname[x]=#s
 
void CANbus::initPropertyDefs(int32_t firmwareVersion){
  if(firmwareVersion < 40){
    throw "Unsupported early firmware version";
  } else {
    /* Common */
    DEFPROP(VERS , 0);
    DEFPROP(ROLE , 1); /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
    DEFPROP(SN   , 2);
    DEFPROP(ID   , 3);
    DEFPROP(ERROR, 4);
    DEFPROP(STAT , 5);
    DEFPROP(ADDR , 6);
    DEFPROP(VALUE, 7);
    DEFPROP(MODE , 8);
    DEFPROP(TEMP , 9);
    DEFPROP(PTEMP, 10);
    DEFPROP(OTEMP, 11);
    DEFPROP(BAUD , 12);
    DEFPROP(_LOCK, 13);
    DEFPROP(DIG0 , 14);
    DEFPROP(DIG1 , 15);
    DEFPROP(TENSION,16); DEFPROP(FET0,16);
    DEFPROP(BRAKE, 17);  DEFPROP(FET1,17);
    DEFPROP(ANA0 , 18);
    DEFPROP(ANA1 , 19);
    DEFPROP(THERM, 20);
    DEFPROP(VBUS , 21);
    DEFPROP(IMOTOR,22);
    DEFPROP(VLOGIC,23);
    DEFPROP(ILOGIC,24);
    DEFPROP(SG   , 25);
    DEFPROP(GRPA , 26);
    DEFPROP(GRPB , 27);
    DEFPROP(GRPC , 28);
    /* CMD is for sending hand commands to the hand pucks:
            RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
    DEFPROP(CMD  , 29); 
    DEFPROP(SAVE , 30);
    DEFPROP(LOAD , 31);
    DEFPROP(DEF  , 32);
    DEFPROP(FIND , 33);
    DEFPROP(X0   , 34);
    DEFPROP(X1   , 35);
    DEFPROP(X2   , 36);
    DEFPROP(X3   , 37);
    DEFPROP(X4   , 38);
    DEFPROP(X5   , 39);
    DEFPROP(X6   , 40);
    DEFPROP(X7   , 41);

    /* Safety puck */
    DEFPROP(ZERO , 42);
    DEFPROP(PEN  , 43);
    DEFPROP(SAFE , 44);
    DEFPROP(VL1  , 45);
    DEFPROP(VL2  , 46);
    DEFPROP(TL1  , 47);
    DEFPROP(TL2  , 48);
    DEFPROP(VOLTL1,49);
    DEFPROP(VOLTL2,50);
    DEFPROP(VOLTH1,51);
    DEFPROP(VOLTH2,52);
    DEFPROP(PWR  , 53);
    DEFPROP(MAXPWR,54);
    DEFPROP(IFAULT,55);
    DEFPROP(VNOM , 56);
    
    /* Motor pucks */
    DEFPROP(T    , 42);  DEFPROP(TORQ,42);
    DEFPROP(MT   , 43);
    DEFPROP(V    , 44);
    DEFPROP(MV   , 45);
    DEFPROP(MCV  , 46);
    DEFPROP(MOV  , 47);
    DEFPROP(AP   , 48); DEFPROP(P,48); /* 32-Bit Present Position */
    DEFPROP(P2   , 49);
    DEFPROP(DP   , 50); /* 32-Bit Default Position */
    DEFPROP(DP2  , 51);
    DEFPROP(E    , 52); /* 32-Bit Endpoint */
    DEFPROP(E2   , 53);
    DEFPROP(OT   , 54); /* 32-Bit Open Target */
    DEFPROP(OT2  , 55);
    DEFPROP(CT   , 56); /* 32-Bit Close Target */
    DEFPROP(CT2  , 57);
    DEFPROP(M    , 58); /* 32-Bit Move command for CAN*/
    DEFPROP(M2   , 59);
    DEFPROP(_DS  , 60);
    DEFPROP(MOFST, 61);
    DEFPROP(IOFST, 62);
    DEFPROP(UPSECS,63);
    DEFPROP(OD   , 64);
    DEFPROP(MDS  , 65);
    DEFPROP(MECH , 66); /* 32-Bit */
    DEFPROP(MECH2, 67);
    DEFPROP(CTS  , 68); /* 32-Bit */
    DEFPROP(CTS2 , 69);
    DEFPROP(PIDX , 70);
    DEFPROP(HSG  , 71);
    DEFPROP(LSG  , 72);
    DEFPROP(IVEL , 73);
    DEFPROP(IOFF , 74); /* 32-Bit */
    DEFPROP(IOFF2, 75);
    DEFPROP(MPE  , 76);
    // property 77 varies with firmware version
    DEFPROP(TSTOP, 78);
    DEFPROP(KP   , 79);
    DEFPROP(KD   , 80);
    DEFPROP(KI   , 81);
    DEFPROP(ACCEL, 82);
    DEFPROP(TENST, 83);
    DEFPROP(TENSO, 84);
    DEFPROP(JIDX , 85);
    DEFPROP(IPNM , 86);
    DEFPROP(HALLS, 87);
    DEFPROP(HALLH, 88); /* 32-Bit */
    DEFPROP(HALLH2,89);
    DEFPROP(POLES, 90);
    DEFPROP(IKP  , 91);
    DEFPROP(IKI  , 92);
    DEFPROP(IKCOR, 93);
    DEFPROP(ECMAX, 101);
    DEFPROP(ECMIN, 102);
    DEFPROP(LFLAGS,103);
    DEFPROP(LCTC , 104);

    if (firmwareVersion < 132) {
      DEFPROP(EN ,  77);
      DEFPROP(HOLD ,94);
      DEFPROP(TIE  ,95);
      DEFPROP(LCVC ,100);
      PROP_END=101;
    } else { // firmwareVersion >= 132
      DEFPROP(HOLD , 77);
      DEFPROP(EN   , 94);
      DEFPROP(EN2  , 95);
      DEFPROP(JP   , 96);
      DEFPROP(JP2  , 97);
      DEFPROP(JOFST, 98);
      DEFPROP(JOFST2,99);
      DEFPROP(TIE  , 100);
      DEFPROP(LCVC , 105);
      DEFPROP(TACT , 106);
      DEFPROP(TACTID,107);
      PROP_END=108;
    }

    if(firmwareVersion >=175){
      DEFPROP(IHIT , 108);
      PROP_END=109;
    }

    /* Force/Torque sensor */
    DEFPROP(FT   , 54);
    DEFPROP(A    , 64);
  }
}

template<> inline bool DataRecorder<CANbus::canio_data>::dump(const char *fname) {
  FILE *csv = fopen(fname,"w");
  if (csv) {
    ROS_FATAL("Dumping CANbus log to %s",fname);
    for (unsigned int i=0; i<count; ++i) {
      CANbus::canio_data cdata = data[i];
      char timestring[100];
      time_t logtime = cdata.secs;
	//	+ 4*3600; // shift by 4 hours to account for EDT - GMT shift
      strftime(timestring,100,"%F %T",localtime(&logtime));
	
      fprintf(csv,"[%s,%06d] ",timestring,cdata.usecs);
      if (cdata.msgid == -1) {
	fprintf(csv,"SLEEP %d usecs\n",cdata.msgdata[0]);
	continue;
      }
      if (cdata.msgid == -2) {
	fprintf(csv,"TIMEOUT after %d usecs\n",cdata.msgdata[0]);
	continue;
      }
      if (cdata.send) {
	fprintf(csv,"SEND ");
      } else {
	fprintf(csv,"READ ");
      }
      int32_t recv_id = cdata.msgid & 0x1F; // bits 0-4
      int32_t send_id = (cdata.msgid >> 5) & 0x1F;  // bits 5-9
      fprintf(csv,"%02d ", send_id);
      if (cdata.msgid & 0x400) {
	fprintf(csv,"G%02d ",recv_id);
      } else {
	fprintf(csv," %02d ",recv_id);
      }
      if ((cdata.msgid & 0x41F) == 0x408) {
	// group 8 messages are the Tactile Top-10 data
	uint32_t top10_bitmap = cdata.msgdata[0] << 16 
	  | cdata.msgdata[1] << 8 
	  | cdata.msgdata[2];
	bool first(true);
	std::stringstream top10_string;
	for (int i=0; i<24; ++i) {
	  if (top10_bitmap & 0x800000) {
	    if (first) {
	      first=false;
	    } else {
	      top10_string << ",";
	    }
	    top10_string << i;
	  }
	  top10_bitmap <<= 1;  // left shift by 1 bit
	}
	fprintf(csv,"SET TACTILE TOP10=%s",top10_string.str().c_str());
      } else if ((cdata.msgid & 0x41F) == 0x409) {
	// group 9 messages are the Tactile Full data
	int tactile_group=cdata.msgdata[0] >> 4;
	fprintf(csv,"SET TACTILE FULL GROUP=%d",tactile_group);
      } else if ((cdata.msgid & 0x41F) == 0x40A) {
	// group 10 messages are 3-axis Force from the F/T sensor
	double X,Y,Z;
	X=CANbus::ft_combine(cdata.msgdata[0],cdata.msgdata[1]) / 256.0;
	Y=CANbus::ft_combine(cdata.msgdata[2],cdata.msgdata[3]) / 256.0;
	Z=CANbus::ft_combine(cdata.msgdata[4],cdata.msgdata[5]) / 256.0;
	fprintf(csv,"SET FORCE X=%3.3f Y=%3.3f Z=%3.3f", X, Y, Z);
        if (cdata.msglen > 6) {
	  if (cdata.msgdata[6] & 128) {
	    fprintf(csv," RE-TARE");
	  }
	  if (cdata.msgdata[6] & 64) {
	    fprintf(csv," BAD");
	  }
	}
     } else if ((cdata.msgid & 0x41F) == 0x40B) {
	// group 11 messages are 3-axis Torque from the F/T sensor
	double X,Y,Z;
	X=CANbus::ft_combine(cdata.msgdata[0],cdata.msgdata[1]) / 4096.0;
	Y=CANbus::ft_combine(cdata.msgdata[2],cdata.msgdata[3]) / 4096.0;
	Z=CANbus::ft_combine(cdata.msgdata[4],cdata.msgdata[5]) / 4096.0;
	fprintf(csv,"SET TORQUE X=%3.3f Y=%3.3f Z=%3.3f", X, Y, Z);
        if (cdata.msglen > 6) {
	  if (cdata.msgdata[6] & 128) {
	    fprintf(csv," RE-TARE");
	  }
	  if (cdata.msgdata[6] & 64) {
	    fprintf(csv," BAD");
	  }
	}
      } else if ((cdata.msgid & 0x41F) == 0x40C) {
	// group 12 messages are 3-axis accelerometer values from the F/T sensor
	double X,Y,Z;
	X=CANbus::ft_combine(cdata.msgdata[0],cdata.msgdata[1]) / 1024.0;
	Y=CANbus::ft_combine(cdata.msgdata[2],cdata.msgdata[3]) / 1024.0;
	Z=CANbus::ft_combine(cdata.msgdata[4],cdata.msgdata[5]) / 1024.0;
	fprintf(csv,"SET ACCEL X=%3.3f Y=%3.3f Z=%3.3f", X, Y, Z);
      } else if (cdata.msgdata[0] & 0x80) {  // SET
	if ((cdata.msgdata[0] == (42 | 0x80))
	    && ((cdata.msgid & 0x41F) == 0x404)) {
	  // Packed Torque message
	  int32_t tq1 = (cdata.msgdata[1] << 6) +
	    (cdata.msgdata[2] >> 2);
	  int32_t tq2 = ((cdata.msgdata[2] & 0x03) << 12) + 
	    (cdata.msgdata[3] << 4) +
	    (cdata.msgdata[4] >> 4);
	  int32_t tq3 = ((cdata.msgdata[4] & 0x0F) << 10) +
	    (cdata.msgdata[5] << 2) +
	    (cdata.msgdata[6] >> 6);
	  int32_t tq4 = ((cdata.msgdata[6] & 0x3F) << 8) +
	    cdata.msgdata[7];
	  if (tq1 & 0x00002000) { // If negative 
	      tq1 |= 0xFFFFC000; // Sign-extend
	  }
	  if (tq2 & 0x00002000) {
	      tq2 |= 0xFFFFC000; 
	  }
	  if (tq3 & 0x00002000) {
	      tq3 |= 0xFFFFC000; 
	  }
	  if (tq4 & 0x00002000) {
	      tq4 |= 0xFFFFC000; 
	  }
	  fprintf(csv,"SET T=%d,%d,%d,%d", tq1,tq2,tq3,tq4);
	} else if ((cdata.msgid & 0x41F) == 0x403) {
	  // messages sent to GROUP 3 are 22-bit position updates
	  int32_t value = ((cdata.msgdata[0] & 0x3F) << 16) +
	    (cdata.msgdata[1] << 8) +
	    cdata.msgdata[2];
	  if (value & 0x00200000) { // If negative 
	    value |= 0xFFC00000; // Sign-extend
	  }
	  fprintf(csv,"SET P=%d",value);
	  if (cdata.msglen == 6) {
	    // this puck also sent a secondary value (JP)
	    value = ((cdata.msgdata[3] & 0x3F) << 16) +
	      (cdata.msgdata[4] << 8) +
	      cdata.msgdata[5];
	    if (value & 0x00200000) { // If negative 
	      value |= 0xFFC00000; // Sign-extend
	    }
	    fprintf(csv," JP=%d",value);
	  }
	} else if ((cdata.msgid & 0x41F) == 0x407) {
	  // messages sent to GROUP 7 are 22-bit joint positions
	  int32_t value = ((cdata.msgdata[0] & 0x3F) << 16) +
	    (cdata.msgdata[1] << 8) +
	    cdata.msgdata[2];
	  if (value & 0x00200000) { // If negative 
	    value |= 0xFFC00000; // Sign-extend
	  }
	  fprintf(csv,"SET JP=%d",value);
	  if (cdata.msglen == 6) {
	    // this puck also sent a secondary value (JP2?)
	    value = ((cdata.msgdata[3] & 0x3F) << 16) +
	      (cdata.msgdata[4] << 8) +
	      cdata.msgdata[5];
	    if (value & 0x00200000) { // If negative 
	      value |= 0xFFC00000; // Sign-extend
	    }
	    fprintf(csv," ??=%d",value);
	  }
	} else {
	  // regular property
	  int32_t value = (cdata.msgdata[3] << 8) + cdata.msgdata[2];
	  if (cdata.msglen == 5) {
	    // 24-bit value
	    value += (cdata.msgdata[4] << 16);
	    if (value & 0x00800000) { // If negative 
	      value |= 0xFF000000; // Sign-extend
	    }
	  } else if (cdata.msglen == 6) {
	    // 32-bit value
	    value += (cdata.msgdata[4] << 16) + (cdata.msgdata[5] << 24);
	  } else {
	    // 16-bit value
	    if (value & 0x00008000) { // If negative 
	      value |=  0xFFFF0000; // Sign-extend
	    }
	  }	    
	  if (CANbus::propname.find(cdata.msgdata[0] & 0x7F) != CANbus::propname.end()) {
	    fprintf(csv,"SET %s=%d",CANbus::propname[cdata.msgdata[0] & 0x7F].c_str(), value);
	  } else {
	    fprintf(csv,"SET %03d=%d",cdata.msgdata[0] & 0x7F, value);
	  }
	}
      } else {  // GET
	if (CANbus::propname.find(cdata.msgdata[0]) != CANbus::propname.end()) {
	  fprintf(csv,"GET %s",CANbus::propname[cdata.msgdata[0]].c_str());
	} else {
	  fprintf(csv,"GET %03d",cdata.msgdata[0]);
	}
      }
      
      fprintf(csv,"\n");
    }
    fclose(csv);
    ROS_FATAL("  ...done dumping CANbus log");
    return true;
  } else {
    ROS_FATAL("Unable to dump CANbus log to %s: %s",
	      fname,strerror(errno));
    return false;
  }
}

 CANbus::~CANbus(){
   ROS_FATAL("Destroying class CANbus");
   //   rt_intr_delete(&rt_can_intr);   
   if(pucks!=NULL) delete pucks; 
   if(trq!=NULL) delete trq; 
   if(pos!=NULL) delete pos;
   if (log_canbus_data) {
     char dumpname[200];
     snprintf(dumpname,200,"candata%d.log",id);
     candata.dump(dumpname);
   }
 }
 
std::map <int, std::string> CANbus::propname;

int VERS=0; // always
int ROLE=0; // always
int SN=-10;
int ID=-10;
int ERROR=-10;
int STAT=5; // always
int ADDR=-10;
int VALUE=-10;
int MODE=8; // always
int TEMP=-10;
int PTEMP=-10;
int OTEMP=-10;
int BAUD=-10;
int _LOCK=-10;
int DIG0=-10;
int DIG1=-10;
int FET0=-10;
int FET1=-10;
int ANA0=-10;
int ANA1=-10;
int THERM=-10;
int VBUS=-10;
int IMOTOR=-10;
int VLOGIC=-10;
int ILOGIC=-10;
int SG=-10;
int GRPA=-10;
int GRPB=-10;
int GRPC=-10;
int CMD=-10;
int SAVE=-10;
int LOAD=-10;
int DEF=-10;
int FIND=-10;
int X0=-10;
int X1=-10;
int X2=-10;
int X3=-10;
int X4=-10;
int X5=-10;
int X6=-10;
int X7=-10;
int T=-10;
int MT=-10;
int V=-10;
int MV=-10;
int MCV=-10;
int MOV=-10;
int P=-10;
int P2=-10;
int DP=-10;
int DP2=-10;
int E=-10;
int E2=-10;
int OT=-10;
int OT2=-10;
int CT=-10;
int CT2=-10;
int M=-10;
int M2=-10;
int _DS=-10;
int MOFST=-10;
int IOFST=-10;
int UPSECS=-10;
int OD=-10;
int MDS=-10;
int MECH=-10;
int MECH2=-10;
int CTS=-10;
int CTS2=-10;
int PIDX=-10;
int HSG=-10;
int LSG=-10;
int IVEL=-10;
int IOFF=-10;
int IOFF2=-10;
int MPE=-10;
int HOLD=-10;
int TSTOP=-10;
int KP=-10;
int KD=-10;
int KI=-10;
int ACCEL=-10;
int TENST=-10;
int TENSO=-10;
int JIDX=-10;
int IPNM=-10;
int HALLS=-10;
int HALLH=-10;
int HALLH2=-10;
int POLES=-10;
int IKP=-10;
int IKI=-10;
int IKCOR=-10;
int EN=-10;
int EN2=-10;
int JP=-10;
int JP2=-10;
int JOFST=-10;
int JOFST2=-10;
int TIE=-10;
int ECMAX=-10;
int ECMIN=-10;
int LFLAGS=-10;
int LCTC=-10;
int LCVC=-10;
int TACT=-10;
int TACTID=-10;
int IHIT=-10;
int PROP_END=-10;
int AP=-10;
int TENSION=-10;
int BRAKE=-10;

/* Safety puck properties */
int ZERO=-10;
int PEN=-10;
int SAFE=-10;
int VL1=-10;
int VL2=-10;
int TL1=-10;
int TL2=-10;
int VOLTL1=-10;
int VOLTL2=-10;
int VOLTH1=-10;
int VOLTH2=-10;
int PWR=-10;
int MAXPWR=-10;
int IFAULT=-10;
int VNOM=-10;

/* Force/Torque properties */
int FT=-10;
int A =-10;

// older properties (firmware < 40)
int D=-10;
int TORQ=-10;
int B=-10;
int MD=-10;
int AP2=-10;
int SAMPLE=-10;
int UNITS=-10;
int RATIO=-10;
int LOG=-10;
int DUMP=-10;
int LOG1=-10;
int LOG2=-10;
int LOG3=-10;
int LOG4=-10;
int GAIN1=-10;
int GAIN2=-10;
int GAIN3=-10;
int OFFSET1=-10;
int OFFSET2=-10;
int OFFSET3=-10;
