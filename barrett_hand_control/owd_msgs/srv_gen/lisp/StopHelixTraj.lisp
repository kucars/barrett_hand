; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StopHelixTraj-request.msg.html

(cl:defclass <StopHelixTraj-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopHelixTraj-request (<StopHelixTraj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopHelixTraj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopHelixTraj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopHelixTraj-request> is deprecated: use owd_msgs-srv:StopHelixTraj-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopHelixTraj-request>) ostream)
  "Serializes a message object of type '<StopHelixTraj-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopHelixTraj-request>) istream)
  "Deserializes a message object of type '<StopHelixTraj-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopHelixTraj-request>)))
  "Returns string type for a service object of type '<StopHelixTraj-request>"
  "owd_msgs/StopHelixTrajRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopHelixTraj-request)))
  "Returns string type for a service object of type 'StopHelixTraj-request"
  "owd_msgs/StopHelixTrajRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopHelixTraj-request>)))
  "Returns md5sum for a message object of type '<StopHelixTraj-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopHelixTraj-request)))
  "Returns md5sum for a message object of type 'StopHelixTraj-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopHelixTraj-request>)))
  "Returns full string definition for message of type '<StopHelixTraj-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopHelixTraj-request)))
  "Returns full string definition for message of type 'StopHelixTraj-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopHelixTraj-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopHelixTraj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopHelixTraj-request
))
;//! \htmlinclude StopHelixTraj-response.msg.html

(cl:defclass <StopHelixTraj-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopHelixTraj-response (<StopHelixTraj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopHelixTraj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopHelixTraj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopHelixTraj-response> is deprecated: use owd_msgs-srv:StopHelixTraj-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopHelixTraj-response>) ostream)
  "Serializes a message object of type '<StopHelixTraj-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopHelixTraj-response>) istream)
  "Deserializes a message object of type '<StopHelixTraj-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopHelixTraj-response>)))
  "Returns string type for a service object of type '<StopHelixTraj-response>"
  "owd_msgs/StopHelixTrajResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopHelixTraj-response)))
  "Returns string type for a service object of type 'StopHelixTraj-response"
  "owd_msgs/StopHelixTrajResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopHelixTraj-response>)))
  "Returns md5sum for a message object of type '<StopHelixTraj-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopHelixTraj-response)))
  "Returns md5sum for a message object of type 'StopHelixTraj-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopHelixTraj-response>)))
  "Returns full string definition for message of type '<StopHelixTraj-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopHelixTraj-response)))
  "Returns full string definition for message of type 'StopHelixTraj-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopHelixTraj-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopHelixTraj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopHelixTraj-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopHelixTraj)))
  'StopHelixTraj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopHelixTraj)))
  'StopHelixTraj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopHelixTraj)))
  "Returns string type for a service object of type '<StopHelixTraj>"
  "owd_msgs/StopHelixTraj")