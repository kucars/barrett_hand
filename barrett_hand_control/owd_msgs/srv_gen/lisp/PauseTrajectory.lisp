; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude PauseTrajectory-request.msg.html

(cl:defclass <PauseTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((pause
    :reader pause
    :initarg :pause
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PauseTrajectory-request (<PauseTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<PauseTrajectory-request> is deprecated: use owd_msgs-srv:PauseTrajectory-request instead.")))

(cl:ensure-generic-function 'pause-val :lambda-list '(m))
(cl:defmethod pause-val ((m <PauseTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:pause-val is deprecated.  Use owd_msgs-srv:pause instead.")
  (pause m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseTrajectory-request>) ostream)
  "Serializes a message object of type '<PauseTrajectory-request>"
  (cl:let* ((signed (cl:slot-value msg 'pause)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseTrajectory-request>) istream)
  "Deserializes a message object of type '<PauseTrajectory-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pause) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseTrajectory-request>)))
  "Returns string type for a service object of type '<PauseTrajectory-request>"
  "owd_msgs/PauseTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseTrajectory-request)))
  "Returns string type for a service object of type 'PauseTrajectory-request"
  "owd_msgs/PauseTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseTrajectory-request>)))
  "Returns md5sum for a message object of type '<PauseTrajectory-request>"
  "a759747906bdd01f886076169018d26e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseTrajectory-request)))
  "Returns md5sum for a message object of type 'PauseTrajectory-request"
  "a759747906bdd01f886076169018d26e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseTrajectory-request>)))
  "Returns full string definition for message of type '<PauseTrajectory-request>"
  (cl:format cl:nil "int8 pause~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseTrajectory-request)))
  "Returns full string definition for message of type 'PauseTrajectory-request"
  (cl:format cl:nil "int8 pause~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseTrajectory-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseTrajectory-request
    (cl:cons ':pause (pause msg))
))
;//! \htmlinclude PauseTrajectory-response.msg.html

(cl:defclass <PauseTrajectory-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass PauseTrajectory-response (<PauseTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PauseTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PauseTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<PauseTrajectory-response> is deprecated: use owd_msgs-srv:PauseTrajectory-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <PauseTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <PauseTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PauseTrajectory-response>) ostream)
  "Serializes a message object of type '<PauseTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PauseTrajectory-response>) istream)
  "Deserializes a message object of type '<PauseTrajectory-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PauseTrajectory-response>)))
  "Returns string type for a service object of type '<PauseTrajectory-response>"
  "owd_msgs/PauseTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseTrajectory-response)))
  "Returns string type for a service object of type 'PauseTrajectory-response"
  "owd_msgs/PauseTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PauseTrajectory-response>)))
  "Returns md5sum for a message object of type '<PauseTrajectory-response>"
  "a759747906bdd01f886076169018d26e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PauseTrajectory-response)))
  "Returns md5sum for a message object of type 'PauseTrajectory-response"
  "a759747906bdd01f886076169018d26e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PauseTrajectory-response>)))
  "Returns full string definition for message of type '<PauseTrajectory-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PauseTrajectory-response)))
  "Returns full string definition for message of type 'PauseTrajectory-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PauseTrajectory-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PauseTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PauseTrajectory-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PauseTrajectory)))
  'PauseTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PauseTrajectory)))
  'PauseTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PauseTrajectory)))
  "Returns string type for a service object of type '<PauseTrajectory>"
  "owd_msgs/PauseTrajectory")