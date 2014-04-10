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

#ifndef OWD_PLUGIN_HH
#define OWD_PLUGIN_HH

#include <vector>
#include <string>
#include <stdint.h>
#include "Link.hh"
#include "../openmath/R6.hh"
#include "TrajType.hh"
#include <owd_msgs/JointTraj.h>

class WAM;

namespace OWD {

  class WamDriver;
  class Trajectory;

  /// Base class for OWD user-defined plugins.  Users can use the
  /// interfaces in this class for interacting with the WAM from
  /// within their own OWD runtime-loadable plugin.  This class
  /// can also be subclassed for users who want to publish their
  /// own data in sync with OWD's message publishing.
  class Plugin {
  public:
    
    Plugin();
    ~Plugin();

    /// \brief Publish user data
    ///
    /// If you subclass the Plugin class and override this function,
    /// OWD will automatically call it at the rate set by the ROS param
    /// <b>~/publish_frequency</b>.
    virtual void Publish();

    /// \brief Add a trajectory to the OWD trajectory queue
    ///
    /// \param traj A pointer to a Trajectory instance
    /// \param failure_reason A string describing why the trajectory
    ///                       was rejected (if it was)
    /// \returns True on success, False on failure
    static bool AddTrajectory(Trajectory *traj,
				  std::string &failure_reason);

    /// \brief Move the hand
    ///
    /// Send a position movement command to the model 280 hand (if present)
    ///
    /// \param p The four target positions, in radians
    /// \returns True on success, false otherwise
    static bool hand_move(std::vector<double> p);

    /// \brief Set hand velocity
    ///
    /// Send a velocity command to the model 280 hand (if present)
    ///
    /// \param v The four velocities, in radians per second
    /// \returns True on success, false otherwise
    static bool hand_velocity(std::vector<double> v);

    /// \brief Set hand torque
    ///
    /// Send a raw torque command to the model 280 hand (if present)
    ///
    /// \param t The four torques, in puck units (approx mA)
    /// \returns True on success, false otherwise
    static bool hand_torque(std::vector<double> t);

    /// \brief Get the handstate
    ///
    /// Copies the CANbus::handstate values.  These values will be one
    /// of the HANDSTATE_* enumeration values in CANdefs.hh
    ///
    /// \param t An array into which the handstate values are copied.
    /// \returns True on success, false otherwise
    static bool hand_get_state(int state[4]);

    /// \brief Set the movement speed for all fingers and the spread
    ///
    /// Sets the finger/spread movement speed using CANbus::hand_set_speed
    ///
    /// \param t Velocity (in radians/sec?) for all fingers and the spread
    /// \returns True on success, false otherwise
    static bool hand_set_speed(const double v);

    /// \brief Sets the speed for each finger and the spread
    ///
    /// Sets the finger/spread movement speed using CANbus::hand_set_speed
    ///
    /// \param t A vector of velocity values (in radians/sec?), one for each finger and the spread.
    /// \returns True on success, false otherwise
    static bool hand_set_speed(const std::vector<double> &v);

    /// \brief Tare the force/torque sensor
    ///
    /// Tare (zero-out) the force/torque sensor (if present)
    ///
    /// \returns True on success, false otherwise
    static bool ft_tare();

    /// \brief Multiply a vector by the current base-frame Jacobian
    ///
    /// \param v A JointPos (vector of doubles) containing the joint angles
    ///
    /// \returns The calculated workspace movement (translation and rotation)
    static const R6 Jacobian_times_vector(JointPos v);

    /// \brief Multiply the supplied vector by the Jacobian Transpose
    ///
    /// \param v an R6 vector of workspace positions and rotations
    /// \returns a JointPos vector of length NJOINTS
    ///
    /// This function is typically used to compute the joint torques
    /// that will yield the requested workspace forces/torques
    static const JointPos JacobianTranspose_times_vector(R6 &v);

    /// \brief Multiply the supplied vector by the Jacobian Transpose in 
    /// EE frame
    ///
    /// \param v an R6 vector of workspace positions and rotations
    /// \returns a JointPos vector of length NJOINTS
    ///
    /// This function is typically used to compute the joint torques
    /// that will yield the requested EE frame forces/torques
    static const JointPos JacobianEETranspose_times_vector(R6 &v);


    /// \brief Multiply the supplied vector by the Jacobian Pseudo_Inverse
    ///
    static const JointPos JacobianPseudoInverse_times_vector(R6 &v);

    /// \brief Project a vector of joint motions into the nullspace
    ///
    static const JointPos Nullspace_projection(JointPos v);

    /// \brief Convert a ROS JointTraj message to an OWD TrajType
    static TrajType ros2owd_traj(owd_msgs::JointTraj &jt);

    /// \brief Current position
    ///
    /// A read-only reference to the arm joint values.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &arm_position;

    /// \brief Target position for PID controllers
    ///
    /// A read-only reference to the target position
    /// that the controllers are trying to hold for each joint.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &target_arm_position;

    /// \brief Current torques from PID controllers
    ///
    /// A read-only reference to the torques calculated by the last
    /// iteration of the individual PID controllers.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &pid_torque;

    /// \brief Current torques from dynamic model
    ///
    /// A read-only reference to the torques calculated for each joint
    /// by the dynamic model.  Includes compensation for gravity and
    /// for the dynamic forces created by the specified velocity and
    /// acceleration.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &dynamic_torque;

    /// \brief Current torques from active trajectory
    ///
    /// A read-only reference to the torques most recently specified
    /// by the active trajectory.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &trajectory_torque;

    /// \brief Current forces from F/T sensor
    ///
    /// A read-only reference to the X/Y/Z forces in Newtons from the
    /// force/torque sensor
    ///
    /// \attention vector will have a size of zero if the force/torque
    /// sensor is not present
    static const std::vector<double> &ft_force;

    /// \brief Current torques from F/T sensor
    ///
    /// A read-only reference to the X/Y/Z torques in Newton-meters
    /// from the force/torque sensor
    ///
    /// \attention vector will have a size of zero if the force/torque
    /// sensor is not present
    static const std::vector<double> &ft_torque;

    /// \brief Current forces from F/T sensor
    ///
    /// A read-only reference to a filtered version of ft_force.
    ///
    static const std::vector<double> &filtered_ft_force;

    /// \brief Current torques from F/T sensor
    ///
    /// A read-only reference to a filtered version of ft_torque
    ///
    static const std::vector<double> &filtered_ft_torque;

    /// \brief Current F/T sensor saturation state bitmask
    ///
    /// A read-only reference to an integer bitmask
    ///
    static const int &ft_saturation_state;

    /// \brief Current position of the hand joints
    ///
    /// A read-only reference to the position of the four hand joints
    /// for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &hand_position;

    /// \brief Target position of the hand joints
    ///
    /// A read-only reference to the target position of the four hand
    /// joints for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &target_hand_position;

    /// \brief Current strain gauge readings from the hand
    ///
    /// A read-only reference to the straingauge readings from the
    /// three fingers for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &strain;

    /// \brief Finger 1 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 1
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f1;

    /// \brief Finger 2 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 2
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f2;

    /// \brief Finger 3 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 3
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f3;

    /// \brief Palm tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on the palm
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_palm;

    /// \brief The pose of the WAM end effector (link 7 origin)
    static const SE3 &endpoint;

    /// \brief The acceleration due to gravity, in m/s^2.
    ///
    /// The gravitational force will be multiplied by each of the
    /// link masses to compute the gravity-compensation torque to
    /// be sent to each joint.  Default is 9.81; gravity compensation
    /// can be disabled by setting it to zero.
    static double gravity;

    /// \brief Whether we're running owd (false) or owdsim (true)
    static const bool simulation;

    /// \brief The lower limit of each joint (in radians)
    static const std::vector<double> &lower_jlimit;

    /// \brief The upper limit of each joint (in radians)
    static const std::vector<double> &upper_jlimit;

    /// \brief The absolute max velocity for each joint (radians/s)
    static const std::vector<double> &max_joint_vel;

    /// \brief The current velocity limit for each joint (radians/s)
    static const std::vector<double> &joint_vel;

    /// \brief The current acceleration limit for each joint (radians/s/s)
    static const std::vector<double> &joint_accel;

    /// \brief The absolute max jerk (radians/s/s/s)
    static const double &max_jerk;

    /// \brief The max speed in cartesian coordinates, measured at both the
    ///       elbow and the hand (m/s)
    static const double &max_cartesian_velocity;

    /// \brief The filtered velocity of each arm joint (in radians/s)
    static const std::vector<double> &arm_velocity;

    static const bool &holdpos;

    // Set the links in the plugin, so to be accesible to the user
    static bool set_links(OWD::Link* links);

    // Computes the end-effector pose (forward kinematics) at the given joint configuration
    static SE3 get_endeffector_pose(JointPos config);

    static OWD::Trajectory *jointstraj;

  private:
    friend class ::WAM;
    friend class WamDriver;

    static void PublishAll();
    static WAM *wam;
    static WamDriver *wamdriver;

    static std::vector<double> _arm_position;
    static std::vector<double> _target_arm_position;
    static std::vector<double> _pid_torque;
    static std::vector<double> _dynamic_torque;
    static std::vector<double> _trajectory_torque;
    static std::vector<double> _ft_force;
    static std::vector<double> _ft_torque;
    static std::vector<double> _filtered_ft_force;
    static std::vector<double> _filtered_ft_torque;
    static int                 _ft_saturation_state;
    static std::vector<double> _hand_position;
    static std::vector<double> _target_hand_position;
    static std::vector<double> _strain;
    static std::vector<float> _tactile_f1;
    static std::vector<float> _tactile_f2;
    static std::vector<float> _tactile_f3;
    static std::vector<float> _tactile_palm;
    static std::vector<double> _lower_jlimit;
    static std::vector<double> _upper_jlimit;
    static std::vector<double> _max_joint_vel;
    static std::vector<double> _joint_vel;
    static std::vector<double> _joint_accel;
    static double _max_jerk;
    static double _max_cartesian_velocity;
    static std::vector<double> _arm_velocity;
    static SE3 _endpoint;
    static bool _holdpos;
    static std::vector<Plugin *> children;
    static OWD::Link m_links[OWD::Link::Ln];
  };

  /// Base class for user-defined OWD controller plugins.  Define your own
  /// class subclassed from the base class, making sure to override the pure
  /// virtual members.  Then create an instance of your class in your plugin,
  /// passing the name of your choice to the base class constructor.  The
  /// instance will register its name with the base class, and thereafter
  /// you can instruct OWD to start using it via the SetController service call.
  /// OWD will also report the name of the current controller in the WAMState
  /// messages.
  class JSController {
  public:
    
    /// Base class constructor requires an identifying name for each instance
    JSController(std::string name);
    ~JSController();
    
    /// \brief Compute control torques
    ///
    /// The main function.  Override this function in your class in order
    /// to return your computed joint torques.
    ///
    /// \param q_target The vector of target joint positions (0-based)
    /// \param q The vector of actual joint positions
    /// \param dt The elapsed time (in seconds) since the previous call
    /// \returns A vector of joint torques
    virtual std::vector<double> evaluate(const std::vector<double> q_target,
					 const std::vector<double> q,
					 const double dt) =0;

    /// \brief Adjust the controller gains
    ///
    /// \param joint The index of the joint to change (0-based)
    /// \param gains A vector of new gains (length depends on the controller)
    /// \returns True on success, false otherwise (bad joint index or
    ///          unreasonable gain values
    virtual bool set_gains(unsigned int joint, std::vector<double> gains) =0;

    /// \brief Get the current gains
    ///
    /// \param joint The index of the joint to get (0-based)
    /// \returns A vector of the current gain values (controller dependent)
    virtual std::vector<double> get_gains(unsigned int joint) =0;

    /// \brief The number of joints being controlled
    ///
    /// \returns The number of joints
    virtual int DOF() =0;

    /// \brief Reset an individual joint controller to the initial state
    virtual void reset(unsigned int j) =0;

    /// \brief Prepare an individual controller to run
    ///
    /// \note If the controller is not able to run in the current state
    ///       it can return false or throw a string describing the problem.
    virtual bool run(unsigned int j) throw (const char *) =0;

    /// \brief Stop a controller.
    ///
    /// When a controller has been stopped the evaluate function should
    /// return zero torque for that joint
    virtual void stop(unsigned int j) =0;
    
    /// \brief Whether the controller is active or not
    virtual bool active(unsigned int j) =0;

    static JSController *find_controller(std::string name);
    const std::string &name;

  private:
    std::string _name;
    static std::vector<JSController *> children;

  };



};

#endif // OWD_PLUGIN_HH
