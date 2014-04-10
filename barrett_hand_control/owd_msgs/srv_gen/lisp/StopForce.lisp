; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StopForce-request.msg.html

(cl:defclass <StopForce-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopForce-request (<StopForce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopForce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopForce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopForce-request> is deprecated: use owd_msgs-srv:StopForce-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopForce-request>) ostream)
  "Serializes a message object of type '<StopForce-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopForce-request>) istream)
  "Deserializes a message object of type '<StopForce-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopForce-request>)))
  "Returns string type for a service object of type '<StopForce-request>"
  "owd_msgs/StopForceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopForce-request)))
  "Returns string type for a service object of type 'StopForce-request"
  "owd_msgs/StopForceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopForce-request>)))
  "Returns md5sum for a message object of type '<StopForce-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopForce-request)))
  "Returns md5sum for a message object of type 'StopForce-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopForce-request>)))
  "Returns full string definition for message of type '<StopForce-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopForce-request)))
  "Returns full string definition for message of type 'StopForce-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopForce-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopForce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopForce-request
))
;//! \htmlinclude StopForce-response.msg.html

(cl:defclass <StopForce-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopForce-response (<StopForce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopForce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopForce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopForce-response> is deprecated: use owd_msgs-srv:StopForce-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopForce-response>) ostream)
  "Serializes a message object of type '<StopForce-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopForce-response>) istream)
  "Deserializes a message object of type '<StopForce-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopForce-response>)))
  "Returns string type for a service object of type '<StopForce-response>"
  "owd_msgs/StopForceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopForce-response)))
  "Returns string type for a service object of type 'StopForce-response"
  "owd_msgs/StopForceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopForce-response>)))
  "Returns md5sum for a message object of type '<StopForce-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopForce-response)))
  "Returns md5sum for a message object of type 'StopForce-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopForce-response>)))
  "Returns full string definition for message of type '<StopForce-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopForce-response)))
  "Returns full string definition for message of type 'StopForce-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopForce-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopForce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopForce-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopForce)))
  'StopForce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopForce)))
  'StopForce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopForce)))
  "Returns string type for a service object of type '<StopForce>"
  "owd_msgs/StopForce")