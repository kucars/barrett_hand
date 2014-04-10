#ifndef MULTISYNC_H
#define MULTISYNC_H

#include <sys/time.h>
#include <string>

#define MAX_CONTROLLERS 10

class MultiSync {
 public:

  typedef enum {
    ABORTED=0,
    READY,
    RUNNING,
    STALLED,
    DONE
  } STATUS;


  class ControllerInfo {
  public:
    char controller_name[20];
    char traj_id[50];
    double last_traj_time;
    MultiSync::STATUS status;

    ControllerInfo(std::string name);
  };

  class MasterInfo {
  public:
    unsigned int controller_count;
    double traj_duration;
    timeval last_system_time;
    double time_factor;
    double allowance;
    ControllerInfo controller_info[MAX_CONTROLLERS];

    void Init(int c);
    
  private:
    MasterInfo(); // don't let anyone construct one directly
  };


  MultiSync(std::string controller_name,
	    unsigned int controller_id,
	    bool force=false,
	    int timeout=-1,
	    unsigned int shm_key=4502);
  ~MultiSync();

  /// \brief Declare that you are ready to run a trajectory
  ///
  /// Before starting a trajectory, you have to register it so that
  /// the master can ensure that everyone has the same trajectory
  /// (identical id and duration) loaded and ready to go.
  /// \param[in] traj_id The unique identifier
  /// \param[in] duration The calculated execution time, in seconds
  /// \param[in] timeout The time to wait for the Master to register the
  ///                same trajectory (defaults to 1 second)
  /// \param[out] bool False if the Master doesn't register the same
  ///                  trajectory within the timout, or if the trajectory
  ///                  durations do not match.  True otherwise
  virtual bool register_traj(std::string traj_id, double duration, double timeout=1.0);

  /// \brief Waits for the Master to initiate execution
  ///
  /// \param[in] timeout Time to wait for start, in seconds (defaults
  ///                    to 1 second).
  /// \param[out] bool True if execution can begin, False if the
  ///                  timeout has expired or if one of the other
  ///                  controllers aborted.
  virtual bool wait_for_traj_start(double timeout=10.0);

  virtual double traj_sync_time() throw (const char *);
  double time_factor();
  void run();
  void stall();
  void abort();
  void done();
  STATUS status();
  char last_error[200];

protected:
  void SharedMemConnect(unsigned int shm_key, bool master, int timeout=-1) throw (const char *);
  void SharedMemDisconnect() throw (const char *);
  MultiSync::MasterInfo *master_info;
  unsigned int id;
};

class MultiSyncMaster : public MultiSync {
 public:
  MultiSyncMaster(std::string controller_name, 
		  unsigned int controller_count,
		  bool force=false,
		  unsigned int shm_key=4502);
  ~MultiSyncMaster();

  virtual bool register_traj(std::string traj_id, double duration, double timeout=1.0);

  virtual bool wait_for_traj_start(double timeout=10.0);

  virtual double traj_sync_time() throw (const char *);
  
};



#endif // MULTISYNC_H
