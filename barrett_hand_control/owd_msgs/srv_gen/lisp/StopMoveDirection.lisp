; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StopMoveDirection-request.msg.html

(cl:defclass <StopMoveDirection-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopMoveDirection-request (<StopMoveDirection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMoveDirection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMoveDirection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopMoveDirection-request> is deprecated: use owd_msgs-srv:StopMoveDirection-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMoveDirection-request>) ostream)
  "Serializes a message object of type '<StopMoveDirection-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMoveDirection-request>) istream)
  "Deserializes a message object of type '<StopMoveDirection-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMoveDirection-request>)))
  "Returns string type for a service object of type '<StopMoveDirection-request>"
  "owd_msgs/StopMoveDirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMoveDirection-request)))
  "Returns string type for a service object of type 'StopMoveDirection-request"
  "owd_msgs/StopMoveDirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMoveDirection-request>)))
  "Returns md5sum for a message object of type '<StopMoveDirection-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMoveDirection-request)))
  "Returns md5sum for a message object of type 'StopMoveDirection-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMoveDirection-request>)))
  "Returns full string definition for message of type '<StopMoveDirection-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMoveDirection-request)))
  "Returns full string definition for message of type 'StopMoveDirection-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMoveDirection-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMoveDirection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMoveDirection-request
))
;//! \htmlinclude StopMoveDirection-response.msg.html

(cl:defclass <StopMoveDirection-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopMoveDirection-response (<StopMoveDirection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMoveDirection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMoveDirection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StopMoveDirection-response> is deprecated: use owd_msgs-srv:StopMoveDirection-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMoveDirection-response>) ostream)
  "Serializes a message object of type '<StopMoveDirection-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMoveDirection-response>) istream)
  "Deserializes a message object of type '<StopMoveDirection-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMoveDirection-response>)))
  "Returns string type for a service object of type '<StopMoveDirection-response>"
  "owd_msgs/StopMoveDirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMoveDirection-response)))
  "Returns string type for a service object of type 'StopMoveDirection-response"
  "owd_msgs/StopMoveDirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMoveDirection-response>)))
  "Returns md5sum for a message object of type '<StopMoveDirection-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMoveDirection-response)))
  "Returns md5sum for a message object of type 'StopMoveDirection-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMoveDirection-response>)))
  "Returns full string definition for message of type '<StopMoveDirection-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMoveDirection-response)))
  "Returns full string definition for message of type 'StopMoveDirection-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMoveDirection-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMoveDirection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMoveDirection-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopMoveDirection)))
  'StopMoveDirection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopMoveDirection)))
  'StopMoveDirection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMoveDirection)))
  "Returns string type for a service object of type '<StopMoveDirection>"
  "owd_msgs/StopMoveDirection")