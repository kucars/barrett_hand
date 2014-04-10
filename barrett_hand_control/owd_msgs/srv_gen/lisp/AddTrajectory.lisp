; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude AddTrajectory-request.msg.html

(cl:defclass <AddTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((traj
    :reader traj
    :initarg :traj
    :type owd_msgs-msg:JointTraj
    :initform (cl:make-instance 'owd_msgs-msg:JointTraj)))
)

(cl:defclass AddTrajectory-request (<AddTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<AddTrajectory-request> is deprecated: use owd_msgs-srv:AddTrajectory-request instead.")))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <AddTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:traj-val is deprecated.  Use owd_msgs-srv:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTrajectory-request>) ostream)
  "Serializes a message object of type '<AddTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTrajectory-request>) istream)
  "Deserializes a message object of type '<AddTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTrajectory-request>)))
  "Returns string type for a service object of type '<AddTrajectory-request>"
  "owd_msgs/AddTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTrajectory-request)))
  "Returns string type for a service object of type 'AddTrajectory-request"
  "owd_msgs/AddTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTrajectory-request>)))
  "Returns md5sum for a message object of type '<AddTrajectory-request>"
  "b0fb00f22f7cc2db3b94dfce7a138eaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTrajectory-request)))
  "Returns md5sum for a message object of type 'AddTrajectory-request"
  "b0fb00f22f7cc2db3b94dfce7a138eaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTrajectory-request>)))
  "Returns full string definition for message of type '<AddTrajectory-request>"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTrajectory-request)))
  "Returns full string definition for message of type 'AddTrajectory-request"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTrajectory-request
    (cl:cons ':traj (traj msg))
))
;//! \htmlinclude AddTrajectory-response.msg.html

(cl:defclass <AddTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (time_added
    :reader time_added
    :initarg :time_added
    :type cl:real
    :initform 0))
)

(cl:defclass AddTrajectory-response (<AddTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<AddTrajectory-response> is deprecated: use owd_msgs-srv:AddTrajectory-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <AddTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <AddTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AddTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'time_added-val :lambda-list '(m))
(cl:defmethod time_added-val ((m <AddTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:time_added-val is deprecated.  Use owd_msgs-srv:time_added instead.")
  (time_added m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTrajectory-response>) ostream)
  "Serializes a message object of type '<AddTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_added)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_added) (cl:floor (cl:slot-value msg 'time_added)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTrajectory-response>) istream)
  "Deserializes a message object of type '<AddTrajectory-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_added) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTrajectory-response>)))
  "Returns string type for a service object of type '<AddTrajectory-response>"
  "owd_msgs/AddTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTrajectory-response)))
  "Returns string type for a service object of type 'AddTrajectory-response"
  "owd_msgs/AddTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTrajectory-response>)))
  "Returns md5sum for a message object of type '<AddTrajectory-response>"
  "b0fb00f22f7cc2db3b94dfce7a138eaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTrajectory-response)))
  "Returns md5sum for a message object of type 'AddTrajectory-response"
  "b0fb00f22f7cc2db3b94dfce7a138eaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTrajectory-response>)))
  "Returns full string definition for message of type '<AddTrajectory-response>"
  (cl:format cl:nil "bool ok~%string reason~%string id~%time time_added~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTrajectory-response)))
  "Returns full string definition for message of type 'AddTrajectory-response"
  (cl:format cl:nil "bool ok~%string reason~%string id~%time time_added~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTrajectory-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4 (cl:length (cl:slot-value msg 'id))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTrajectory-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
    (cl:cons ':time_added (time_added msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddTrajectory)))
  'AddTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddTrajectory)))
  'AddTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTrajectory)))
  "Returns string type for a service object of type '<AddTrajectory>"
  "owd_msgs/AddTrajectory")