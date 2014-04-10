; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude WamRequestSeaCtrlKp-request.msg.html

(cl:defclass <WamRequestSeaCtrlKp-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKp-request (<WamRequestSeaCtrlKp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKp-request> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKp-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKp-request>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKp-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKp-request>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKp-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKp-request>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKp-request>"
  "owd_msgs/WamRequestSeaCtrlKpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKp-request)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKp-request"
  "owd_msgs/WamRequestSeaCtrlKpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKp-request>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKp-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKp-request)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKp-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKp-request>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKp-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKp-request)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKp-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKp-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKp-request
))
;//! \htmlinclude WamRequestSeaCtrlKp-response.msg.html

(cl:defclass <WamRequestSeaCtrlKp-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKp-response (<WamRequestSeaCtrlKp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKp-response> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKp-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKp-response>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKp-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKp-response>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKp-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKp-response>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKp-response>"
  "owd_msgs/WamRequestSeaCtrlKpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKp-response)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKp-response"
  "owd_msgs/WamRequestSeaCtrlKpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKp-response>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKp-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKp-response)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKp-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKp-response>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKp-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKp-response)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKp-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKp-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKp-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WamRequestSeaCtrlKp)))
  'WamRequestSeaCtrlKp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WamRequestSeaCtrlKp)))
  'WamRequestSeaCtrlKp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKp)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKp>"
  "owd_msgs/WamRequestSeaCtrlKp")