; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude ForceState.msg.html

(cl:defclass <ForceState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (saturated_axes
    :reader saturated_axes
    :initarg :saturated_axes
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ForceState (<ForceState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ForceState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ForceState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<ForceState> is deprecated: use owd_msgs-msg:ForceState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ForceState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:header-val is deprecated.  Use owd_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <ForceState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:wrench-val is deprecated.  Use owd_msgs-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'saturated_axes-val :lambda-list '(m))
(cl:defmethod saturated_axes-val ((m <ForceState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:saturated_axes-val is deprecated.  Use owd_msgs-msg:saturated_axes instead.")
  (saturated_axes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ForceState>) ostream)
  "Serializes a message object of type '<ForceState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'saturated_axes)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ForceState>) istream)
  "Deserializes a message object of type '<ForceState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'saturated_axes)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ForceState>)))
  "Returns string type for a message object of type '<ForceState>"
  "owd_msgs/ForceState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ForceState)))
  "Returns string type for a message object of type 'ForceState"
  "owd_msgs/ForceState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ForceState>)))
  "Returns md5sum for a message object of type '<ForceState>"
  "f1410afad95483f36c6e47a83ba05017")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ForceState)))
  "Returns md5sum for a message object of type 'ForceState"
  "f1410afad95483f36c6e47a83ba05017")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ForceState>)))
  "Returns full string definition for message of type '<ForceState>"
  (cl:format cl:nil "#Message to contain the state of the force torque data~%Header header~%geometry_msgs/Wrench wrench~%uint8 saturated_axes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ForceState)))
  "Returns full string definition for message of type 'ForceState"
  (cl:format cl:nil "#Message to contain the state of the force torque data~%Header header~%geometry_msgs/Wrench wrench~%uint8 saturated_axes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ForceState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ForceState>))
  "Converts a ROS message object to a list"
  (cl:list 'ForceState
    (cl:cons ':header (header msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':saturated_axes (saturated_axes msg))
))
