#include "MultiSync.hh"
#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

void MultiSync::MasterInfo::Init(int c) {
  for (int i=1; i<c; ++i) {
    controller_info[i]=ControllerInfo(std::string("(none)"));
  }
  // only set the controller_count after all the individual info
  // blocks have been initialized, because any slaves will be waiting
  // for the count to go non-zero (assuming the system clears it in
  // the first place)
  time_factor=1.0;
  allowance = 0.010;
  controller_count = c;
}

MultiSync::ControllerInfo::ControllerInfo(std::string name)
  : status(ABORTED) {
  snprintf(controller_name,20,"%s",name.c_str());
  strcpy(traj_id,"(none)");
}

MultiSync::MultiSync(std::string controller_name,
		     unsigned int controller_id,
		     bool force,
		     int timeout,
		     unsigned int shm_key)
  : id(controller_id) {
  
  if (id == 0) {
    SharedMemConnect(shm_key,true);
  } else {
    SharedMemConnect(shm_key,false, timeout);
    if (id >= master_info->controller_count) {
      throw "Controller number out of range";
    }
    if (strcmp(master_info->controller_info[id].controller_name,
	       "(none)") &&
	strncmp(master_info->controller_info[id].controller_name,
		controller_name.c_str(),
		20) &&
	!force) {
      throw "Another controller is already using my controller_info slot";
    }
  }
  master_info->controller_info[id] 
    = ControllerInfo(controller_name);
  strcpy(last_error,"");
}

MultiSync::~MultiSync() {
  strcpy(master_info->controller_info[id].controller_name,"(none)");
  master_info->controller_info[id].status = ABORTED;
  SharedMemDisconnect();
}

void MultiSync::SharedMemConnect(unsigned int shm_key, bool master, int timeout) throw (const char *) {
  int shm_id;
  int retry(timeout*2);
  if (master) {
    shm_id = shmget(shm_key,sizeof(MasterInfo),IPC_CREAT|S_IRWXU|S_IRWXG);
  } else {
    shm_id = shmget(shm_key,sizeof(MasterInfo),0);
    while ((shm_id == -1) && (errno == ENOENT) && 
	   ((timeout == -1) || (retry-- > 0))) {
      // Keep retrying until the master creates the segment or we time out
      // timeout of -1 means keep trying indefinitely
      usleep(500000);
      shm_id = shmget(shm_key,sizeof(MasterInfo),0);
    }
    if ((shm_id == -1) && (errno == ENOENT)) {
      char errmsg[200];
      snprintf(errmsg,200,"Gave up attaching to shared memory segment after waiting %d seconds for the master to create it", timeout);
      throw errmsg;
    }
  }
  if (shm_id == -1) {
    char errmsg[200];
    switch (errno) {
    case EACCES:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: PERMISSION DENIED");
      break;
    case EINVAL:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: INVALID SIZE");
      break;
    case ENFILE:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: TOO MANY OPEN FILES ON SYSTEM");
      break;
    case ENOMEM:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: OUT OF MEMORY");
      break;
    case ENOSPC:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: OUT OF SHARED MEMORY");
      break;
    default:
      snprintf(errmsg,200,"Error creating/reattaching to shared memory segment: UNEXPECTED ERROR (errno = %d)",errno);
      break;
    }
    throw errmsg;
  }
  master_info = (MasterInfo *) shmat(shm_id, NULL, 0);
  if ((void *)master_info == (void *)-1) {
    char errmsg[200];
    switch (errno) {
    case EACCES:
      snprintf(errmsg,200,"Error retrieving shared memory segment: PERMISSION DENIED");
      break;
    case EINVAL:
      snprintf(errmsg,200,"Error retrieving shared memory segment: SHMID NOT FOUND");
      break;
    case ENOMEM:
      snprintf(errmsg,200,"Error retrieving shared memory segment: OUT OF MEMORY");
      break;
    }
    throw errmsg;
  }

  if (!master) {
    // wait until the master initializes the master_info block.
    // for this wait cycle we will only sleep 10ms, so multiply
    // any remaining time by 50 (but add 1 to make sure we have at
    // least one chance).
    retry *= 50;  retry++;
    while ((master_info->controller_count == 0) && (retry-- > 0)){
      usleep(10000);
    }
    if (master_info->controller_count == 0) {
      throw "Gave up while waiting for master to initialize the shared memory segment";
    }
  }
  return;
}

void MultiSync::SharedMemDisconnect() throw (const char *) {
  if (shmdt(master_info) == -1) {
    throw "Could not detach shared memory segment for master_info; perhaps the pointer was corrupted?";
  }
}

MultiSyncMaster::MultiSyncMaster(std::string controller_name, 
		  unsigned int controller_count,
		  bool force,
		  unsigned int shm_key)
  : MultiSync(controller_name,
	      0,
	      force,
	      0,
	      shm_key) {
  // initialize the shared data structure (header info and
  // each of the individual controller infos)
  master_info->Init(controller_count);
}

MultiSyncMaster::~MultiSyncMaster() {
  // Let any remaining slaves know that we have gone
  master_info->controller_count=0;
  // Detach our segment
  SharedMemDisconnect();
}

bool MultiSync::register_traj(std::string traj_id, double duration, double timeout) {
  // The slave registers a trajectory by waiting for the master to register it,
  // making sure the durations match and that the master hasn't aborted it,
  // and then setting its own status to READY.

  int retry(timeout / 0.010);
  bool matched_id;
  while (retry-- >0) {
    matched_id = (strncmp(master_info->controller_info[0].traj_id,
			  traj_id.c_str(),
			  50) == 0);
    if (matched_id) {
      break;
    } else {
      // sleep 10ms
      usleep(10000);
    }
  }
  if (!matched_id) {
    // the master never registered the same traj id
    snprintf(last_error,200,"Timed out while waiting for master to register trajectory %s",traj_id.c_str());
    return false;
  }
  if (master_info->traj_duration != duration) {
    snprintf(last_error,200,"Duration mismatch: master showed %2.3f, we have %2.3f",master_info->traj_duration, duration);
    return false;
  }
  if (master_info->controller_info[0].status == ABORTED) {
    snprintf(last_error,200,"Master aborted trajectory");
    return false;
  }
  // everything checks out, so register our trajectory
  master_info->controller_info[id].status=READY;
  master_info->controller_info[id].last_traj_time=0;
  snprintf(master_info->controller_info[id].traj_id,
	   50,
	   "%s",
	   traj_id.c_str());
  return true;
}

bool MultiSync::wait_for_traj_start(double timeout) {
  // The slave just has to wait until the master switches from READY to
  // RUNNING
  int retry(timeout / 0.004);
  bool matched_id;
  int waitcount(0);
  while (retry-- > 0) {
    matched_id=(strncmp(master_info->controller_info[0].traj_id,
			     master_info->controller_info[id].traj_id,
			50) == 0);
    if (matched_id &&
	(master_info->controller_info[0].status != READY)) {
      break;
    } else {
      ++waitcount;
      usleep(4000);
    }
  }
  if (!matched_id) {
    snprintf(last_error,200,"Aborted while waiting for traj start because master ID no longer matches our ID of %s",master_info->controller_info[id].traj_id);
    return false;
  } else if (master_info->controller_info[0].status == RUNNING) {
    return true;
  } else if (master_info->controller_info[0].status == STALLED) {
    return true;
  } else if (master_info->controller_info[0].status == READY) {
    // we timed out
    // set our own flag to ABORTED so that the master knows we gave up
    master_info->controller_info[id].status = ABORTED;
    snprintf(last_error,200,"timed out after sleeping %d cycles (%d remaining)",waitcount,retry);
    return false;
  } else if (master_info->controller_info[0].status == ABORTED) {
    master_info->controller_info[id].status = ABORTED;
    snprintf(last_error,200,"master aborted before beginning");
    return false;
  } else if (master_info->controller_info[0].status == DONE) {
    if (master_info->traj_duration == 0) {
      // this often happens for zero-length trajectories but it's not
      // really a problem
      master_info->controller_info[id].status = DONE;
    } else {
      master_info->controller_info[id].status = ABORTED;
    }
    snprintf(last_error,200,"master finished before we began");
    return false;
  } else {
    master_info->controller_info[id].status = ABORTED;
    snprintf(last_error,200,"master switched to an unknown status");
  }
  return false;
}

double MultiSync::traj_sync_time() throw (const char *) {
  if (strncmp(master_info->controller_info[0].traj_id,
	     master_info->controller_info[id].traj_id, 50)) {
    snprintf(last_error,200,"Master has switched to trajectory ID %s so we cannot continue trajectory ID %s",
	     master_info->controller_info[0].traj_id,
	     master_info->controller_info[id].traj_id);
    abort();
    throw last_error;
  }

  // Each time a slave calculates the time, it should also check to see
  // if any of the other controllers have aborted
  for (unsigned int c=0; c<master_info->controller_count; ++c) {
    if (c == id) {
      continue; // don't bother checking ourself
    }
    if (master_info->controller_info[c].status == ABORTED) {
      // we should also abort now
      abort();
      // use the same time that the aborted controller stopped at
      master_info->controller_info[id].last_traj_time =
	master_info->controller_info[c].last_traj_time;
      snprintf(last_error,200,"Controller %s aborted",
	       master_info->controller_info[c].controller_name);
      throw last_error;
    }
  }
  timeval now;
  if (gettimeofday(&now, NULL) == -1) {
    snprintf(last_error,200,"%s","Could not get system time");
    abort();
    throw last_error;
  }
  // calculate how long it's been since the master set the time
  double elapsed = now.tv_sec - master_info->last_system_time.tv_sec
    + (now.tv_usec - master_info->last_system_time.tv_usec) / 1e6;
  // set our time by taking the master traj time and adding in the
  // increment based on the time elapsed since it calculated its time
  master_info->controller_info[id].last_traj_time = 
    master_info->controller_info[0].last_traj_time
    + elapsed * master_info->time_factor;
  
  if (elapsed > master_info->allowance) {
    // the master stopped incrementing the time; must have died, so
    // abort and alert the others.
    // the will eventually stop at our last_traj_time
    snprintf(last_error,200,"Master controller has not updated for %1.3fs; presumed dead",elapsed);
    abort();
    throw last_error;
  }
  return master_info->controller_info[id].last_traj_time;
}

double MultiSync::time_factor() {
  return master_info->time_factor;
}

void MultiSync::run() {
  master_info->controller_info[id].status=RUNNING;
}

void MultiSync::stall() {
  master_info->controller_info[id].status=STALLED;
}

void MultiSync::abort() {

  master_info->controller_info[id].status=ABORTED;
}

void MultiSync::done() {
  master_info->controller_info[id].status=DONE;
}

MultiSync::STATUS MultiSync::status() {
  return master_info->controller_info[id].status;
}

bool MultiSyncMaster::register_traj(std::string traj_id, double duration, double timeout) {
  // make sure we don't move on to a new traj until all controllers have
  // finished the previous traj
  bool running=true;
  int retry(timeout / 0.05);
  unsigned int c;
  while (running && (retry-- > 0)) {
    running=false;
    for (c=1; c<master_info->controller_count; ++c) {
      if ((master_info->controller_info[c].status == STALLED) ||
	  (master_info->controller_info[c].status == RUNNING)) {
	running=true;
	usleep(50000);
	break;
      }
    }
  }
  if (retry == 0) {
    snprintf(last_error,200,"Controller %s is still executing the previous traj",
	     master_info->controller_info[c].controller_name);
    return false;
  }
  master_info->traj_duration=duration;
  master_info->time_factor=1.0;
  master_info->controller_info[0].status = READY;
  master_info->controller_info[0].last_traj_time = 0;
  snprintf(master_info->controller_info[0].traj_id,50,"%s",traj_id.c_str());
  return true;
}

bool MultiSyncMaster::wait_for_traj_start(double timeout) {
  // The master has to wait for all the slaves to be READY with the
  // same trajectory id
  
  int retry(timeout/0.004);
  bool ready=false;
  int waitcount(0);
  while (!ready && (retry-- > 0)) {
    ready=true;
    for (unsigned int c=1; c<master_info->controller_count; ++c) {
      bool matched_id = (strcmp(master_info->controller_info[0].traj_id,
				master_info->controller_info[c].traj_id) == 0);
      if (matched_id && 
	  (master_info->controller_info[c].status == ABORTED)) {
	// if any slave aborts, then we will tell everyone else to
	// abort, too
	master_info->controller_info[0].status=ABORTED;
	snprintf(last_error,200,"Slave controller %s aborted before beginning",
		 master_info->controller_info[c].controller_name);
	return false;
      } else if ( (!matched_id) ||
		  (master_info->controller_info[c].status != READY)) {
	ready=false;
	usleep(4000);
	waitcount++;
	// after sleeping we'll start over again at the beginning
	break;
      }
    }
  }
  if (!ready) {
    snprintf(last_error,200,"Timed out while waiting for slave controllers to switch to ready, waitcount=%d",waitcount);
    return false;
  }
  // Once everyone is ready, we'll switch to RUNNING
  if (gettimeofday(&master_info->last_system_time, NULL) == -1) {
    if (errno == EFAULT) {
      snprintf(last_error,200,"%s","Could not set time in the specified memory location; should set a local variable first");
    }
    return false;
  }
  master_info->controller_info[0].last_traj_time=0;
  master_info->controller_info[0].status=RUNNING;
  return true;
}

double MultiSyncMaster::traj_sync_time() throw (const char *) {
  // each time the master calculates the time, it needs to also check for
  // any stalled, aborted, or unresponsive slaves
  bool stalled=false;
  for (unsigned int c=1; c<master_info->controller_count; ++c) {
    if (master_info->controller_info[c].status == ABORTED) {
      // this slave has aborted, so tell the others to
      // stop at the same time
      int retval = gettimeofday(&master_info->last_system_time, NULL);
      master_info->controller_info[0].last_traj_time = 
	master_info->controller_info[c].last_traj_time;
      master_info->controller_info[0].status = ABORTED;
      if (retval == -1) {
	snprintf(last_error,200,"%s","Could not get system time");
	throw last_error;
      }
      snprintf(last_error,200,"Controller %s aborted",
	       master_info->controller_info[c].controller_name);
      throw last_error;
    }
    if (master_info->controller_info[c].status == STALLED) {
      // at least one slave is stalled, but we have to keep
      // checking the others for aborted, so just remember it
      stalled=true;
    }
    if ((master_info->controller_info[0].last_traj_time -
	 master_info->controller_info[c].last_traj_time) >
	master_info->allowance) {
      // one slave fell behind, so abort
      int retval = gettimeofday(&master_info->last_system_time, NULL);
      master_info->controller_info[0].status = ABORTED;
      if (retval == -1) {
	snprintf(last_error,200,"%s","Could not get system time");
	throw last_error;
      }
      snprintf(last_error,200,"Controller %s fell too far behind",
	       master_info->controller_info[c].controller_name);
      throw last_error;
    }
  }
  // if a slave or ourself has stalled
  if (stalled || (master_info->controller_info[0].status == STALLED)) {
    if (master_info->time_factor > 0.05f) {
      master_info->time_factor *= 0.97;
    }
  } else if (master_info->time_factor < 1.0f) {
    // we're no longer stalling, so we can start returning to
    // a unity time factor
    master_info->time_factor *= 1.03f;
    if (master_info->time_factor > 1.0f) {
      master_info->time_factor = 1.0f;
    }
  }
  timeval now;
  if (gettimeofday(&now, NULL) == -1) {
    master_info->controller_info[0].status = ABORTED;
    snprintf(last_error,200,"%s","Could not get system time");
    throw last_error;
  }
  double elapsed = now.tv_sec - master_info->last_system_time.tv_sec
    + (now.tv_usec - master_info->last_system_time.tv_usec) / 1e6;
  double trajtime = master_info->controller_info[0].last_traj_time
    + elapsed * master_info->time_factor;
  // set the system time first, so that if a slave tries to calculate the
  // time before we set the last_traj_time, at least they won't jump into
  // the future
  master_info->last_system_time = now;
  master_info->controller_info[0].last_traj_time = trajtime;
  return trajtime;
}

