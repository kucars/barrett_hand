if(DEFINED CATKIN_TOPLEVEL)


cmake_minimum_required(VERSION 2.8.3)
project(owd_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
add_message_files(
  DIRECTORY msg
  FILES
    ApplyForceDebug.msg
    BHState.msg
    BHTactile.msg
    ForceRead.msg
    IndexedJointValues.msg
    Jacobian.msg
    Joints.msg
    JointTraj.msg
    MassProperties.msg
    PIDgains.msg
    Servo.msg
    TrajInfo.msg
    WAMInternals.msg
    WamSetupSeaCtrl.msg
    WAMState.msg
)
add_service_files(
  DIRECTORY srv
  FILES
    AddTimedTrajectory.srv
    AddTrajectory.srv
    AddWSTraj.srv
    ApplyEEForceTorque.srv
    ApplyForce.srv
    CalibrateJoints.srv
    CancelAllTrajectories.srv
    DeleteTrajectory.srv
    GetDOF.srv
    GetHandProperty.srv
    GetSpeed.srv
    GetStallSensitivity.srv
    GetStiffness.srv
    InsertKey.srv
    MoveHand.srv
    OpenDoor.srv
    PauseTrajectory.srv
    RelaxHand.srv
    ReplaceTrajectory.srv
    ResetFinger.srv
    ResetHand.srv
    Reset.srv
    SetController.srv
    SetExtraMass.srv
    SetForceInputThreshold.srv
    SetGains.srv
    SetHandProperty.srv
    SetHandTorque.srv
    SetJointOffsets.srv
    SetJointStiffness.srv
    SetSpeed.srv
    SetStallSensitivity.srv
    SetStiffness.srv
    SetTactileInputThreshold.srv
    SetTorqueLimits.srv
    StartHelixTraj.srv
    StartMoveDirection.srv
    StartTeleop.srv
    StepJoint.srv
    StopForce.srv
    StopHelixTraj.srv
    StopMoveDirection.srv
    WamRequestSeaCtrlKd.srv
    WamRequestSeaCtrlKi.srv
    WamRequestSeaCtrlKp.srv
    WamRequestSeaCtrlTorqLimit.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)


else(DEFINED CATKIN_TOPLEVEL)


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()


endif(DEFINED CATKIN_TOPLEVEL)
