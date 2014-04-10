; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude GetStallSensitivity-request.msg.html

(cl:defclass <GetStallSensitivity-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetStallSensitivity-request (<GetStallSensitivity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStallSensitivity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStallSensitivity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetStallSensitivity-request> is deprecated: use owd_msgs-srv:GetStallSensitivity-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStallSensitivity-request>) ostream)
  "Serializes a message object of type '<GetStallSensitivity-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStallSensitivity-request>) istream)
  "Deserializes a message object of type '<GetStallSensitivity-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStallSensitivity-request>)))
  "Returns string type for a service object of type '<GetStallSensitivity-request>"
  "owd_msgs/GetStallSensitivityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStallSensitivity-request)))
  "Returns string type for a service object of type 'GetStallSensitivity-request"
  "owd_msgs/GetStallSensitivityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStallSensitivity-request>)))
  "Returns md5sum for a message object of type '<GetStallSensitivity-request>"
  "480b3e33acc20b8f29c6011b379fbc8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStallSensitivity-request)))
  "Returns md5sum for a message object of type 'GetStallSensitivity-request"
  "480b3e33acc20b8f29c6011b379fbc8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStallSensitivity-request>)))
  "Returns full string definition for message of type '<GetStallSensitivity-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStallSensitivity-request)))
  "Returns full string definition for message of type 'GetStallSensitivity-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStallSensitivity-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStallSensitivity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStallSensitivity-request
))
;//! \htmlinclude GetStallSensitivity-response.msg.html

(cl:defclass <GetStallSensitivity-response> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetStallSensitivity-response (<GetStallSensitivity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStallSensitivity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStallSensitivity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetStallSensitivity-response> is deprecated: use owd_msgs-srv:GetStallSensitivity-response instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <GetStallSensitivity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:level-val is deprecated.  Use owd_msgs-srv:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStallSensitivity-response>) ostream)
  "Serializes a message object of type '<GetStallSensitivity-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'level))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStallSensitivity-response>) istream)
  "Deserializes a message object of type '<GetStallSensitivity-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'level) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStallSensitivity-response>)))
  "Returns string type for a service object of type '<GetStallSensitivity-response>"
  "owd_msgs/GetStallSensitivityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStallSensitivity-response)))
  "Returns string type for a service object of type 'GetStallSensitivity-response"
  "owd_msgs/GetStallSensitivityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStallSensitivity-response>)))
  "Returns md5sum for a message object of type '<GetStallSensitivity-response>"
  "480b3e33acc20b8f29c6011b379fbc8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStallSensitivity-response)))
  "Returns md5sum for a message object of type 'GetStallSensitivity-response"
  "480b3e33acc20b8f29c6011b379fbc8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStallSensitivity-response>)))
  "Returns full string definition for message of type '<GetStallSensitivity-response>"
  (cl:format cl:nil "float32 level~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStallSensitivity-response)))
  "Returns full string definition for message of type 'GetStallSensitivity-response"
  (cl:format cl:nil "float32 level~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStallSensitivity-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStallSensitivity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStallSensitivity-response
    (cl:cons ':level (level msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetStallSensitivity)))
  'GetStallSensitivity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetStallSensitivity)))
  'GetStallSensitivity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStallSensitivity)))
  "Returns string type for a service object of type '<GetStallSensitivity>"
  "owd_msgs/GetStallSensitivity")