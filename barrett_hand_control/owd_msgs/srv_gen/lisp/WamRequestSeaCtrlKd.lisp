; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude WamRequestSeaCtrlKd-request.msg.html

(cl:defclass <WamRequestSeaCtrlKd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKd-request (<WamRequestSeaCtrlKd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKd-request> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKd-request>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKd-request>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKd-request>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKd-request>"
  "owd_msgs/WamRequestSeaCtrlKdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKd-request)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKd-request"
  "owd_msgs/WamRequestSeaCtrlKdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKd-request>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKd-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKd-request)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKd-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKd-request>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKd-request)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKd-request
))
;//! \htmlinclude WamRequestSeaCtrlKd-response.msg.html

(cl:defclass <WamRequestSeaCtrlKd-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKd-response (<WamRequestSeaCtrlKd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKd-response> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKd-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKd-response>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKd-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKd-response>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKd-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKd-response>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKd-response>"
  "owd_msgs/WamRequestSeaCtrlKdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKd-response)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKd-response"
  "owd_msgs/WamRequestSeaCtrlKdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKd-response>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKd-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKd-response)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKd-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKd-response>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKd-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKd-response)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKd-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKd-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKd-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WamRequestSeaCtrlKd)))
  'WamRequestSeaCtrlKd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WamRequestSeaCtrlKd)))
  'WamRequestSeaCtrlKd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKd)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKd>"
  "owd_msgs/WamRequestSeaCtrlKd")