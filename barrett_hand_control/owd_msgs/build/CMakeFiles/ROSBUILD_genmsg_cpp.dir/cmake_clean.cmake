FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/owd_msgs/msg"
  "../src/owd_msgs/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/owd_msgs/BHTactile.h"
  "../msg_gen/cpp/include/owd_msgs/Servo.h"
  "../msg_gen/cpp/include/owd_msgs/IndexedJointValues.h"
  "../msg_gen/cpp/include/owd_msgs/WamSetupSeaCtrl.h"
  "../msg_gen/cpp/include/owd_msgs/JointTraj.h"
  "../msg_gen/cpp/include/owd_msgs/PIDgains.h"
  "../msg_gen/cpp/include/owd_msgs/ForceRead.h"
  "../msg_gen/cpp/include/owd_msgs/TrajInfo.h"
  "../msg_gen/cpp/include/owd_msgs/WAMInternals.h"
  "../msg_gen/cpp/include/owd_msgs/ForceState.h"
  "../msg_gen/cpp/include/owd_msgs/MassProperties.h"
  "../msg_gen/cpp/include/owd_msgs/BHState.h"
  "../msg_gen/cpp/include/owd_msgs/ApplyForceDebug.h"
  "../msg_gen/cpp/include/owd_msgs/WAMState.h"
  "../msg_gen/cpp/include/owd_msgs/Jacobian.h"
  "../msg_gen/cpp/include/owd_msgs/Joints.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
