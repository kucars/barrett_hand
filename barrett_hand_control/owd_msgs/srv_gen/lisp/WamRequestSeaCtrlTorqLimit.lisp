; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude WamRequestSeaCtrlTorqLimit-request.msg.html

(cl:defclass <WamRequestSeaCtrlTorqLimit-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlTorqLimit-request (<WamRequestSeaCtrlTorqLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlTorqLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlTorqLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlTorqLimit-request> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlTorqLimit-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlTorqLimit-request>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlTorqLimit-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlTorqLimit-request>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlTorqLimit-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlTorqLimit-request>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlTorqLimit-request>"
  "owd_msgs/WamRequestSeaCtrlTorqLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlTorqLimit-request)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlTorqLimit-request"
  "owd_msgs/WamRequestSeaCtrlTorqLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlTorqLimit-request>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlTorqLimit-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlTorqLimit-request)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlTorqLimit-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlTorqLimit-request>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlTorqLimit-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlTorqLimit-request)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlTorqLimit-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlTorqLimit-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlTorqLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlTorqLimit-request
))
;//! \htmlinclude WamRequestSeaCtrlTorqLimit-response.msg.html

(cl:defclass <WamRequestSeaCtrlTorqLimit-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WamRequestSeaCtrlTorqLimit-response (<WamRequestSeaCtrlTorqLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamRequestSeaCtrlTorqLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamRequestSeaCtrlTorqLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<WamRequestSeaCtrlTorqLimit-response> is deprecated: use owd_msgs-srv:WamRequestSeaCtrlTorqLimit-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamRequestSeaCtrlTorqLimit-response>) ostream)
  "Serializes a message object of type '<WamRequestSeaCtrlTorqLimit-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamRequestSeaCtrlTorqLimit-response>) istream)
  "Deserializes a message object of type '<WamRequestSeaCtrlTorqLimit-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamRequestSeaCtrlTorqLimit-response>)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlTorqLimit-response>"
  "owd_msgs/WamRequestSeaCtrlTorqLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlTorqLimit-response)))
  "Returns string type for a service object of type 'WamRequestSeaCtrlTorqLimit-response"
  "owd_msgs/WamRequestSeaCtrlTorqLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamRequestSeaCtrlTorqLimit-response>)))
  "Returns md5sum for a message object of type '<WamRequestSeaCtrlTorqLimit-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamRequestSeaCtrlTorqLimit-response)))
  "Returns md5sum for a message object of type 'WamRequestSeaCtrlTorqLimit-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamRequestSeaCtrlTorqLimit-response>)))
  "Returns full string definition for message of type '<WamRequestSeaCtrlTorqLimit-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamRequestSeaCtrlTorqLimit-response)))
  "Returns full string definition for message of type 'WamRequestSeaCtrlTorqLimit-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamRequestSeaCtrlTorqLimit-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamRequestSeaCtrlTorqLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WamRequestSeaCtrlTorqLimit-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WamRequestSeaCtrlTorqLimit)))
  'WamRequestSeaCtrlTorqLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WamRequestSeaCtrlTorqLimit)))
  'WamRequestSeaCtrlTorqLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamRequestSeaCtrlTorqLimit)))
  "Returns string type for a service object of type '<WamRequestSeaCtrlTorqLimit>"
  "owd_msgs/WamRequestSeaCtrlTorqLimit")