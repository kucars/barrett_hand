; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude CalibrateJoints-request.msg.html

(cl:defclass <CalibrateJoints-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CalibrateJoints-request (<CalibrateJoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateJoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateJoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<CalibrateJoints-request> is deprecated: use owd_msgs-srv:CalibrateJoints-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateJoints-request>) ostream)
  "Serializes a message object of type '<CalibrateJoints-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateJoints-request>) istream)
  "Deserializes a message object of type '<CalibrateJoints-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateJoints-request>)))
  "Returns string type for a service object of type '<CalibrateJoints-request>"
  "owd_msgs/CalibrateJointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateJoints-request)))
  "Returns string type for a service object of type 'CalibrateJoints-request"
  "owd_msgs/CalibrateJointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateJoints-request>)))
  "Returns md5sum for a message object of type '<CalibrateJoints-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateJoints-request)))
  "Returns md5sum for a message object of type 'CalibrateJoints-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateJoints-request>)))
  "Returns full string definition for message of type '<CalibrateJoints-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateJoints-request)))
  "Returns full string definition for message of type 'CalibrateJoints-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateJoints-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateJoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateJoints-request
))
;//! \htmlinclude CalibrateJoints-response.msg.html

(cl:defclass <CalibrateJoints-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CalibrateJoints-response (<CalibrateJoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateJoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateJoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<CalibrateJoints-response> is deprecated: use owd_msgs-srv:CalibrateJoints-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateJoints-response>) ostream)
  "Serializes a message object of type '<CalibrateJoints-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateJoints-response>) istream)
  "Deserializes a message object of type '<CalibrateJoints-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateJoints-response>)))
  "Returns string type for a service object of type '<CalibrateJoints-response>"
  "owd_msgs/CalibrateJointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateJoints-response)))
  "Returns string type for a service object of type 'CalibrateJoints-response"
  "owd_msgs/CalibrateJointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateJoints-response>)))
  "Returns md5sum for a message object of type '<CalibrateJoints-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateJoints-response)))
  "Returns md5sum for a message object of type 'CalibrateJoints-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateJoints-response>)))
  "Returns full string definition for message of type '<CalibrateJoints-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateJoints-response)))
  "Returns full string definition for message of type 'CalibrateJoints-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateJoints-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateJoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateJoints-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalibrateJoints)))
  'CalibrateJoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalibrateJoints)))
  'CalibrateJoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateJoints)))
  "Returns string type for a service object of type '<CalibrateJoints>"
  "owd_msgs/CalibrateJoints")