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

#ifndef __TRAJECTORY_HH__
#define __TRAJECTORY_HH__

#include <pthread.h>
#include <string>
#include <stdint.h>

#include "TrajType.hh"
#include "../openmath/R6.hh"
#include "Butterworth.h"
#include "BinaryData.hh"

namespace OWD {
  class WamDriver;

  /// Custom OWD trajectories must be derived from this class.  The
  /// only required function in a subclass is an implementation of
  /// the evaluate() function.
  class Trajectory{
  public:
    pthread_mutex_t mutex;

    /// The start_position must match either OWD's current position,
    /// if there is no active trajectory, or the end_position of the
    /// last trajectory in the queue.  Otherwise, OWD will reject the
    /// trajectory when you try to add it.
    JointPos start_position;

    /// The position at which the trajectory will be completed.  If
    /// the trajectory is of a nature that the endpoint is not
    /// predefined, then it should just be the best estimate, and
    /// should be updated as execution continues.  OWD will use the
    /// end_position to compare against the start position of any new
    /// trajectory requests.  If they match, the new trajectory will
    /// be added to the queue, but if they don't match, the new
    /// trajectory is rejected.
    JointPos end_position;

    /// The base constructor will initialize runstate to STOP.  Once
    /// OWD is ready to start executing the trajectory, it will call
    /// the run() function, which will set the state to RUN (unless
    /// the option WaitForStart is set to true).  Thereafter, OWD will
    /// continue to call the evaluate() function until that function
    /// changes the runstate to DONE.
    int runstate;

    /// \brief The total time the trajectory has been running, in seconds
    double time;

    static const int STOP = 0;
    static const int RUN  = 1;
    static const int DONE = 2;
    static const int LOG  = 3;
    static const int ABORT =4;

    /// \brief Identification string
    ///
    /// The unique identification string for this trajectory.  This field
    /// is normally set by the client, but will gets assigned by OWD if the
    /// client fails to provide one.
    std::string id;

    /// \brief Trajectory type
    ///
    /// A string naming the type of trajectory.  This name shows up in
    /// the trajectory queue that is part of the WAMState message.
    std::string type;

    /// \brief Total planned execution time
    double duration;


    /// \brief Automatically cancel a trajectory when the arm stalls
    ///
    /// If true, OWD will cancel the trajectory if
    /// the PID torques exceed the safety thresholds while the
    /// trajectory is executing (usually an indication that the arm
    /// has run into something).  Defaults to false, in which case
    /// OWD will keep trying to complete the trajectory.
    bool CancelOnStall;

    /// \brief Do not start the trajectory when queued
    ///
    /// If true, OWD will start the trajectory in the STOP state.  The
    /// user can start the execution with the PauseTrajectory service
    /// call.  Defaults to false, in which case OWD will start the
    /// trajectory in the RUN state.
    bool WaitForStart;

    /// \brief Synchronize execution with other controllers
    ///
    /// If true, OWD will use the MultiSync class to synchronize
    /// execution with one or more additional controllers.  Defaults
    /// to false, in which case OWD will run independently.
    bool Synchronize;

    /// \brief Automatically cancel a trajectory based on sensed force
    ///
    /// If true, OWD will cancel the trajectory if
    /// the force/torque sensor reports a value above the threshold
    /// in the specified direction.  Use the SetForceInputThreshold service
    /// to set the threshold (default is 6 Newtons towards the palm).
    /// This option defaults to false, in which case values from the 
    /// force/torque sensor are ignored.
    bool  CancelOnForceInput;

    /// \brief Automatically cancel a trajectory based on sensed contact
    ///
    /// If true, OWD will cancel the trajectory if
    /// the tactile sensor reports values that meet the criteria set
    /// by the SetTactileInputThreshold service call.
    /// This option defaults to false, in which case values from the 
    /// force/torque sensor are ignored.
    bool  CancelOnTactileInput;

    /// \brief The pad number (0-3) used for CancelOnTactileInput
    ///
    static int tactile_pad;

    /// \brief The value at which a cell is considered pressed
    ///
    static float tactile_threshold;

    /// \brief The number of consecutive times the pad must be pressed
    ///        to stop the trajectory
    ///
    static int tactile_minimum_readings;

    R6 forcetorque;
    bool valid_ft;
    Butterworth<OWD::JointPos> tactile_filter;

    static R3 forcetorque_threshold_direction;
    static double forcetorque_threshold;

    static R3 forcetorque_torque_threshold_direction;
    static double forcetorque_torque_threshold;

    /// \brief The contructor requires a trajectory name
    ///
    /// \param name Unique identifier of the trajectory type (usually
    ///             should use the class name)
    ///
    /// \attention When subclassing the Trajectory you will have to
    /// explicitly call this contructor from your own constructor in
    /// order to set the name.
    Trajectory(std::string name, std::string identification) :
      runstate(STOP),time(0.0), id(identification), type(name),
      CancelOnStall(false),WaitForStart(false),
      Synchronize(false),
      CancelOnForceInput(false),
      CancelOnTactileInput(false),
      valid_ft(false),
      tactile_filter(3,5) // 3rd-order, 5hz
    {
      pthread_mutex_init(&mutex, NULL);
    }

    // serialization and deserialization routines
    virtual BinaryData serialize(int firstdof=0, int lastdof=-1);
    Trajectory(BinaryData &bd);

    virtual ~Trajectory(){}
    
    virtual void lock(){pthread_mutex_lock(&mutex);}
    virtual void unlock(){pthread_mutex_unlock(&mutex);}
    
    virtual void run() {
      runstate=RUN;
    }

    virtual void stop() {
      runstate=STOP;
    }

    virtual void abort() {
      runstate=ABORT;
    }

    virtual int  state() {
      return runstate;
    }

    /// The TrajControl class bundles together all the variables that
    /// the evaluate() function can modify.  
    class TrajControl {
    public:
      TrajControl(unsigned int nDOF);

      /// \brief joint positions
      ///
      /// On input, q contains the current joint positions.  On
      /// return, q should contain the target positions that the
      /// trajectory wants the controllers to hold, in radians.  If q
      /// is left unmodified, then the arm will remain free to move
      /// and the control torques will be zero.
      ///
      /// \attention vector will be of size 4 or 7 depending on the arm model
      JointPos q;

      /// \brief joint velocities
      ///
      /// On input, qd is all zeros.  On return, qd should contain the
      /// current velocity, in radians per second, that the trajectory
      /// is moving each joint.  The velocities will be passed to the
      /// dynamics model for calculating feedforward compensation
      /// torques.  If qd is left as all zeros, there will be no
      /// velocity contribution to the dynamic torques.
      ///
      /// \attention vector will be of size 4 or 7 depending on the arm model
      JointPos qd;

      /// \brief joint accelerations
      ///
      /// On input, qdd is all zeros.  On return, qdd should contain
      /// the current acceleration, in radians per second per second,
      /// that the trajectory is accelerating each joint.  The
      /// accelerations will be passed to the dynamics model for
      /// calculating feedforward compensation torques.  If qdd is
      /// left as all zeros, there will be no acceleration
      /// contribution to the dynamic torques.
      ///
      /// \attention vector will be of size 4 or 7 depending on the arm model
      JointPos qdd;

      /// \brief additional joint torques
      ///
      /// On input, t is all zeros.  On return, t should contain the
      /// pure torques that the trajectory wants applied to each
      /// joint, in Newton-meters.  These torques will be added to the
      /// PID torques (calculated from any changes made to q) and the
      /// dynamic torques (calculated from any changes made to qd and
      /// qdd) before being sent to the motors.
      ///
      /// \attention vector will be of size 4 or 7 depending on the arm model
      JointPos t;
    };

    // static WAM *wam;

    /// \brief Calculate and return the current target values
    ///
    /// When the trajectory is active, OWD will call evaluate at the
    /// control frequency (typically 500Hz) to get position, velocity,
    /// acceleration, and torque updates.  If all members of
    /// trajcontrol are left unmodified by evaluate(), no torques will
    /// be applied to the arm beyond gravity compensation, and the arm
    /// will be "free".  The evaluate() function should change
    /// runstate to DONE when the trajectory is completed.  
    ///  \param[in,out] trajcontrol On input, contains current position.
    ///      On return, should contain the desired position, velocity,
    ///      acceleration, and additional torque for each joint.
    ///  \param[in] dt Time that has elapsed since the previous call
    ///         to evaluate().  The evaluate() function should increment
    ///         the ::time variable by dt.
    virtual void evaluate(TrajControl &trajcontrol, double dt);
    // note: the default evaluate function is really just here for backwards
    // compatibility, but new Trajectory classes should focus on implementing
    // evaluate_abs(), which is required for multi-controller synchronization.

    virtual void evaluate_abs(TrajControl &trajcontrol, double t) = 0;

    inline virtual const JointPos &endPosition() const {return end_position;}
  
    virtual bool log(const char* fname);
  
    virtual void reset(double _time) {time=_time;}

    inline virtual double curtime() const {return time;}

    virtual void ForceFeedback(double ft[]);

    virtual void TactileFeedback(float tactile[], int repetitions);

    std::string static random_id();

    friend class WamDriver;
    
    // define types for each of the trajectories with serialization support
    typedef enum {
      TRAJTYPE_MACJOINTTRAJ
    } TRAJTYPES;

  };

}; // namespace OWD

#endif // TRAJECTORY_HH

