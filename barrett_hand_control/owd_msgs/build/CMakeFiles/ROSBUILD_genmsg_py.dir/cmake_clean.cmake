FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/owd_msgs/msg"
  "../src/owd_msgs/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/owd_msgs/msg/__init__.py"
  "../src/owd_msgs/msg/_BHTactile.py"
  "../src/owd_msgs/msg/_Servo.py"
  "../src/owd_msgs/msg/_IndexedJointValues.py"
  "../src/owd_msgs/msg/_WamSetupSeaCtrl.py"
  "../src/owd_msgs/msg/_JointTraj.py"
  "../src/owd_msgs/msg/_PIDgains.py"
  "../src/owd_msgs/msg/_ForceRead.py"
  "../src/owd_msgs/msg/_TrajInfo.py"
  "../src/owd_msgs/msg/_WAMInternals.py"
  "../src/owd_msgs/msg/_ForceState.py"
  "../src/owd_msgs/msg/_MassProperties.py"
  "../src/owd_msgs/msg/_BHState.py"
  "../src/owd_msgs/msg/_ApplyForceDebug.py"
  "../src/owd_msgs/msg/_WAMState.py"
  "../src/owd_msgs/msg/_Jacobian.py"
  "../src/owd_msgs/msg/_Joints.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
