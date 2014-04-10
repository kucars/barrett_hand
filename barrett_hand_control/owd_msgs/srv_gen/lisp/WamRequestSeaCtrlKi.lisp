; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude WamRequestSeaCtrlKi-request.msg.html

(cl:defclass <WamRequestSeaCtrlKi-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKi-request (<WamRequestSeaCtrlKi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKi-request> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKi-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKi-request>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKi-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKi-request>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKi-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKi-request>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKi-request>"
  "owd_msgs/WamRequestSeaCtrlKiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKi-request)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKi-request"
  "owd_msgs/WamRequestSeaCtrlKiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKi-request>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKi-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKi-request)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKi-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKi-request>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKi-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKi-request)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKi-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKi-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKi-request
))
;//! \htmlinclude WamRequestSeaCtrlKi-response.msg.html

(cl:defclass <WamRequestSeaCtrlKi-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlKi-response (<WamRequestSeaCtrlKi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlKi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlKi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlKi-response> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlKi-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlKi-response>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlKi-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlKi-response>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlKi-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlKi-response>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKi-response>"
  "owd_msgs/WamRequestSeaCtrlKiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKi-response)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlKi-response"
  "owd_msgs/WamRequestSeaCtrlKiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlKi-response>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlKi-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlKi-response)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlKi-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlKi-response>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlKi-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlKi-response)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlKi-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlKi-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlKi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlKi-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WamRequestSeaCtrlKi)))
  'WamRequestSeaCtrlKi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WamRequestSeaCtrlKi)))
  'WamRequestSeaCtrlKi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlKi)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlKi>"
  "owd_msgs/WamRequestSeaCtrlKi")