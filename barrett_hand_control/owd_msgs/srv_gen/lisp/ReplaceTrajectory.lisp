; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude ReplaceTrajectory-request.msg.html

(cl:defclass <ReplaceTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((traj
    :reader traj
    :initarg :traj
    :type owd_msgs-msg:JointTraj
    :initform (cl:make-instance 'owd_msgs-msg:JointTraj))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass ReplaceTrajectory-request (<ReplaceTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReplaceTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReplaceTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ReplaceTrajectory-request> is deprecated: use owd_msgs-srv:ReplaceTrajectory-request instead.")))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <ReplaceTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:traj-val is deprecated.  Use owd_msgs-srv:traj instead.")
  (traj m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ReplaceTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReplaceTrajectory-request>) ostream)
  "Serializes a message object of type '<ReplaceTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReplaceTrajectory-request>) istream)
  "Deserializes a message object of type '<ReplaceTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReplaceTrajectory-request>)))
  "Returns string type for a service object of type '<ReplaceTrajectory-request>"
  "owd_msgs/ReplaceTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReplaceTrajectory-request)))
  "Returns string type for a service object of type 'ReplaceTrajectory-request"
  "owd_msgs/ReplaceTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReplaceTrajectory-request>)))
  "Returns md5sum for a message object of type '<ReplaceTrajectory-request>"
  "cc1272a1b0295224dc0b1b55f99d0d36")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReplaceTrajectory-request)))
  "Returns md5sum for a message object of type 'ReplaceTrajectory-request"
  "cc1272a1b0295224dc0b1b55f99d0d36")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReplaceTrajectory-request>)))
  "Returns full string definition for message of type '<ReplaceTrajectory-request>"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%uint32 id~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReplaceTrajectory-request)))
  "Returns full string definition for message of type 'ReplaceTrajectory-request"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%uint32 id~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReplaceTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReplaceTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReplaceTrajectory-request
    (cl:cons ':traj (traj msg))
    (cl:cons ':id (id msg))
))
;//! \htmlinclude ReplaceTrajectory-response.msg.html

(cl:defclass <ReplaceTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass ReplaceTrajectory-response (<ReplaceTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReplaceTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReplaceTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ReplaceTrajectory-response> is deprecated: use owd_msgs-srv:ReplaceTrajectory-response instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ReplaceTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReplaceTrajectory-response>) ostream)
  "Serializes a message object of type '<ReplaceTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReplaceTrajectory-response>) istream)
  "Deserializes a message object of type '<ReplaceTrajectory-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReplaceTrajectory-response>)))
  "Returns string type for a service object of type '<ReplaceTrajectory-response>"
  "owd_msgs/ReplaceTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReplaceTrajectory-response)))
  "Returns string type for a service object of type 'ReplaceTrajectory-response"
  "owd_msgs/ReplaceTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReplaceTrajectory-response>)))
  "Returns md5sum for a message object of type '<ReplaceTrajectory-response>"
  "cc1272a1b0295224dc0b1b55f99d0d36")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReplaceTrajectory-response)))
  "Returns md5sum for a message object of type 'ReplaceTrajectory-response"
  "cc1272a1b0295224dc0b1b55f99d0d36")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReplaceTrajectory-response>)))
  "Returns full string definition for message of type '<ReplaceTrajectory-response>"
  (cl:format cl:nil "uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReplaceTrajectory-response)))
  "Returns full string definition for message of type 'ReplaceTrajectory-response"
  (cl:format cl:nil "uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReplaceTrajectory-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReplaceTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReplaceTrajectory-response
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReplaceTrajectory)))
  'ReplaceTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReplaceTrajectory)))
  'ReplaceTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReplaceTrajectory)))
  "Returns string type for a service object of type '<ReplaceTrajectory>"
  "owd_msgs/ReplaceTrajectory")