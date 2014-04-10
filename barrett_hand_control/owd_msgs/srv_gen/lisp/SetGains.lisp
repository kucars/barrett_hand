; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetGains-request.msg.html

(cl:defclass <SetGains-request> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type cl:fixnum
    :initform 0)
   (gains
    :reader gains
    :initarg :gains
    :type owd_msgs-msg:PIDgains
    :initform (cl:make-instance 'owd_msgs-msg:PIDgains)))
)

(cl:defclass SetGains-request (<SetGains-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGains-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGains-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetGains-request> is deprecated: use owd_msgs-srv:SetGains-request instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <SetGains-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:joint-val is deprecated.  Use owd_msgs-srv:joint instead.")
  (joint m))

(cl:ensure-generic-function 'gains-val :lambda-list '(m))
(cl:defmethod gains-val ((m <SetGains-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:gains-val is deprecated.  Use owd_msgs-srv:gains instead.")
  (gains m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGains-request>) ostream)
  "Serializes a message object of type '<SetGains-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gains) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGains-request>) istream)
  "Deserializes a message object of type '<SetGains-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gains) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGains-request>)))
  "Returns string type for a service object of type '<SetGains-request>"
  "owd_msgs/SetGainsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains-request)))
  "Returns string type for a service object of type 'SetGains-request"
  "owd_msgs/SetGainsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGains-request>)))
  "Returns md5sum for a message object of type '<SetGains-request>"
  "8e1b236ea8d7f7d7cef0473991af0ae8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGains-request)))
  "Returns md5sum for a message object of type 'SetGains-request"
  "8e1b236ea8d7f7d7cef0473991af0ae8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGains-request>)))
  "Returns full string definition for message of type '<SetGains-request>"
  (cl:format cl:nil "uint8 joint~%owd_msgs/PIDgains gains~%~%================================================================================~%MSG: owd_msgs/PIDgains~%float64 kp~%float64 kd~%float64 ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGains-request)))
  "Returns full string definition for message of type 'SetGains-request"
  (cl:format cl:nil "uint8 joint~%owd_msgs/PIDgains gains~%~%================================================================================~%MSG: owd_msgs/PIDgains~%float64 kp~%float64 kd~%float64 ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGains-request>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gains))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGains-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGains-request
    (cl:cons ':joint (joint msg))
    (cl:cons ':gains (gains msg))
))
;//! \htmlinclude SetGains-response.msg.html

(cl:defclass <SetGains-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetGains-response (<SetGains-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGains-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGains-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetGains-response> is deprecated: use owd_msgs-srv:SetGains-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGains-response>) ostream)
  "Serializes a message object of type '<SetGains-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGains-response>) istream)
  "Deserializes a message object of type '<SetGains-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGains-response>)))
  "Returns string type for a service object of type '<SetGains-response>"
  "owd_msgs/SetGainsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains-response)))
  "Returns string type for a service object of type 'SetGains-response"
  "owd_msgs/SetGainsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGains-response>)))
  "Returns md5sum for a message object of type '<SetGains-response>"
  "8e1b236ea8d7f7d7cef0473991af0ae8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGains-response)))
  "Returns md5sum for a message object of type 'SetGains-response"
  "8e1b236ea8d7f7d7cef0473991af0ae8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGains-response>)))
  "Returns full string definition for message of type '<SetGains-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGains-response)))
  "Returns full string definition for message of type 'SetGains-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGains-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGains-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGains-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGains)))
  'SetGains-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGains)))
  'SetGains-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGains)))
  "Returns string type for a service object of type '<SetGains>"
  "owd_msgs/SetGains")