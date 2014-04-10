; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StepJoint-request.msg.html

(cl:defclass <StepJoint-request> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type cl:fixnum
    :initform 0)
   (radians
    :reader radians
    :initarg :radians
    :type cl:float
    :initform 0.0))
)

(cl:defclass StepJoint-request (<StepJoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StepJoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StepJoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StepJoint-request> is deprecated: use owd_msgs-srv:StepJoint-request instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <StepJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:joint-val is deprecated.  Use owd_msgs-srv:joint instead.")
  (joint m))

(cl:ensure-generic-function 'radians-val :lambda-list '(m))
(cl:defmethod radians-val ((m <StepJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:radians-val is deprecated.  Use owd_msgs-srv:radians instead.")
  (radians m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StepJoint-request>) ostream)
  "Serializes a message object of type '<StepJoint-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StepJoint-request>) istream)
  "Deserializes a message object of type '<StepJoint-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'joint)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radians) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StepJoint-request>)))
  "Returns string type for a service object of type '<StepJoint-request>"
  "owd_msgs/StepJointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StepJoint-request)))
  "Returns string type for a service object of type 'StepJoint-request"
  "owd_msgs/StepJointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StepJoint-request>)))
  "Returns md5sum for a message object of type '<StepJoint-request>"
  "e76fd844c151f2b85b89aba56e105bdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StepJoint-request)))
  "Returns md5sum for a message object of type 'StepJoint-request"
  "e76fd844c151f2b85b89aba56e105bdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StepJoint-request>)))
  "Returns full string definition for message of type '<StepJoint-request>"
  (cl:format cl:nil "uint8 joint~%float64 radians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StepJoint-request)))
  "Returns full string definition for message of type 'StepJoint-request"
  (cl:format cl:nil "uint8 joint~%float64 radians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StepJoint-request>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StepJoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StepJoint-request
    (cl:cons ':joint (joint msg))
    (cl:cons ':radians (radians msg))
))
;//! \htmlinclude StepJoint-response.msg.html

(cl:defclass <StepJoint-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StepJoint-response (<StepJoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StepJoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StepJoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StepJoint-response> is deprecated: use owd_msgs-srv:StepJoint-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StepJoint-response>) ostream)
  "Serializes a message object of type '<StepJoint-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StepJoint-response>) istream)
  "Deserializes a message object of type '<StepJoint-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StepJoint-response>)))
  "Returns string type for a service object of type '<StepJoint-response>"
  "owd_msgs/StepJointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StepJoint-response)))
  "Returns string type for a service object of type 'StepJoint-response"
  "owd_msgs/StepJointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StepJoint-response>)))
  "Returns md5sum for a message object of type '<StepJoint-response>"
  "e76fd844c151f2b85b89aba56e105bdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StepJoint-response)))
  "Returns md5sum for a message object of type 'StepJoint-response"
  "e76fd844c151f2b85b89aba56e105bdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StepJoint-response>)))
  "Returns full string definition for message of type '<StepJoint-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StepJoint-response)))
  "Returns full string definition for message of type 'StepJoint-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StepJoint-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StepJoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StepJoint-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StepJoint)))
  'StepJoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StepJoint)))
  'StepJoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StepJoint)))
  "Returns string type for a service object of type '<StepJoint>"
  "owd_msgs/StepJoint")