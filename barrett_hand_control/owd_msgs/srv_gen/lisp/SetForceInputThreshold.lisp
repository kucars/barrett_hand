; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetForceInputThreshold-request.msg.html

(cl:defclass <SetForceInputThreshold-request> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0)
   (torques
    :reader torques
    :initarg :torques
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass SetForceInputThreshold-request (<SetForceInputThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceInputThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceInputThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetForceInputThreshold-request> is deprecated: use owd_msgs-srv:SetForceInputThreshold-request instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <SetForceInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:direction-val is deprecated.  Use owd_msgs-srv:direction instead.")
  (direction m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <SetForceInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:force-val is deprecated.  Use owd_msgs-srv:force instead.")
  (force m))

(cl:ensure-generic-function 'torques-val :lambda-list '(m))
(cl:defmethod torques-val ((m <SetForceInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:torques-val is deprecated.  Use owd_msgs-srv:torques instead.")
  (torques m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceInputThreshold-request>) ostream)
  "Serializes a message object of type '<SetForceInputThreshold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'direction) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'torques) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceInputThreshold-request>) istream)
  "Deserializes a message object of type '<SetForceInputThreshold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'direction) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'torques) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceInputThreshold-request>)))
  "Returns string type for a service object of type '<SetForceInputThreshold-request>"
  "owd_msgs/SetForceInputThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceInputThreshold-request)))
  "Returns string type for a service object of type 'SetForceInputThreshold-request"
  "owd_msgs/SetForceInputThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceInputThreshold-request>)))
  "Returns md5sum for a message object of type '<SetForceInputThreshold-request>"
  "bc61c9402d5e0ad2a8b687c96a82cc21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceInputThreshold-request)))
  "Returns md5sum for a message object of type 'SetForceInputThreshold-request"
  "bc61c9402d5e0ad2a8b687c96a82cc21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceInputThreshold-request>)))
  "Returns full string definition for message of type '<SetForceInputThreshold-request>"
  (cl:format cl:nil "geometry_msgs/Vector3 direction~%float64 force~%geometry_msgs/Vector3 torques~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceInputThreshold-request)))
  "Returns full string definition for message of type 'SetForceInputThreshold-request"
  (cl:format cl:nil "geometry_msgs/Vector3 direction~%float64 force~%geometry_msgs/Vector3 torques~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceInputThreshold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'direction))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'torques))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceInputThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceInputThreshold-request
    (cl:cons ':direction (direction msg))
    (cl:cons ':force (force msg))
    (cl:cons ':torques (torques msg))
))
;//! \htmlinclude SetForceInputThreshold-response.msg.html

(cl:defclass <SetForceInputThreshold-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass SetForceInputThreshold-response (<SetForceInputThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceInputThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceInputThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetForceInputThreshold-response> is deprecated: use owd_msgs-srv:SetForceInputThreshold-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetForceInputThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetForceInputThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceInputThreshold-response>) ostream)
  "Serializes a message object of type '<SetForceInputThreshold-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceInputThreshold-response>) istream)
  "Deserializes a message object of type '<SetForceInputThreshold-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceInputThreshold-response>)))
  "Returns string type for a service object of type '<SetForceInputThreshold-response>"
  "owd_msgs/SetForceInputThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceInputThreshold-response)))
  "Returns string type for a service object of type 'SetForceInputThreshold-response"
  "owd_msgs/SetForceInputThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceInputThreshold-response>)))
  "Returns md5sum for a message object of type '<SetForceInputThreshold-response>"
  "bc61c9402d5e0ad2a8b687c96a82cc21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceInputThreshold-response)))
  "Returns md5sum for a message object of type 'SetForceInputThreshold-response"
  "bc61c9402d5e0ad2a8b687c96a82cc21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceInputThreshold-response>)))
  "Returns full string definition for message of type '<SetForceInputThreshold-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceInputThreshold-response)))
  "Returns full string definition for message of type 'SetForceInputThreshold-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceInputThreshold-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceInputThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceInputThreshold-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetForceInputThreshold)))
  'SetForceInputThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetForceInputThreshold)))
  'SetForceInputThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceInputThreshold)))
  "Returns string type for a service object of type '<SetForceInputThreshold>"
  "owd_msgs/SetForceInputThreshold")