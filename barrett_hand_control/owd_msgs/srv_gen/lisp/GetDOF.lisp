; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude GetDOF-request.msg.html

(cl:defclass <GetDOF-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDOF-request (<GetDOF-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDOF-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDOF-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetDOF-request> is deprecated: use owd_msgs-srv:GetDOF-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDOF-request>) ostream)
  "Serializes a message object of type '<GetDOF-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDOF-request>) istream)
  "Deserializes a message object of type '<GetDOF-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDOF-request>)))
  "Returns string type for a service object of type '<GetDOF-request>"
  "owd_msgs/GetDOFRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDOF-request)))
  "Returns string type for a service object of type 'GetDOF-request"
  "owd_msgs/GetDOFRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDOF-request>)))
  "Returns md5sum for a message object of type '<GetDOF-request>"
  "28965d9e0d5ec6fa5ecf9e0da0bee01d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDOF-request)))
  "Returns md5sum for a message object of type 'GetDOF-request"
  "28965d9e0d5ec6fa5ecf9e0da0bee01d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDOF-request>)))
  "Returns full string definition for message of type '<GetDOF-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDOF-request)))
  "Returns full string definition for message of type 'GetDOF-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDOF-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDOF-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDOF-request
))
;//! \htmlinclude GetDOF-response.msg.html

(cl:defclass <GetDOF-response> (roslisp-msg-protocol:ros-message)
  ((nDOF
    :reader nDOF
    :initarg :nDOF
    :type cl:integer
    :initform 0))
)

(cl:defclass GetDOF-response (<GetDOF-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDOF-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDOF-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetDOF-response> is deprecated: use owd_msgs-srv:GetDOF-response instead.")))

(cl:ensure-generic-function 'nDOF-val :lambda-list '(m))
(cl:defmethod nDOF-val ((m <GetDOF-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:nDOF-val is deprecated.  Use owd_msgs-srv:nDOF instead.")
  (nDOF m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDOF-response>) ostream)
  "Serializes a message object of type '<GetDOF-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nDOF)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'nDOF)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'nDOF)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'nDOF)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDOF-response>) istream)
  "Deserializes a message object of type '<GetDOF-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nDOF)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'nDOF)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'nDOF)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'nDOF)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDOF-response>)))
  "Returns string type for a service object of type '<GetDOF-response>"
  "owd_msgs/GetDOFResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDOF-response)))
  "Returns string type for a service object of type 'GetDOF-response"
  "owd_msgs/GetDOFResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDOF-response>)))
  "Returns md5sum for a message object of type '<GetDOF-response>"
  "28965d9e0d5ec6fa5ecf9e0da0bee01d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDOF-response)))
  "Returns md5sum for a message object of type 'GetDOF-response"
  "28965d9e0d5ec6fa5ecf9e0da0bee01d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDOF-response>)))
  "Returns full string definition for message of type '<GetDOF-response>"
  (cl:format cl:nil "uint32 nDOF~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDOF-response)))
  "Returns full string definition for message of type 'GetDOF-response"
  (cl:format cl:nil "uint32 nDOF~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDOF-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDOF-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDOF-response
    (cl:cons ':nDOF (nDOF msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDOF)))
  'GetDOF-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDOF)))
  'GetDOF-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDOF)))
  "Returns string type for a service object of type '<GetDOF>"
  "owd_msgs/GetDOF")