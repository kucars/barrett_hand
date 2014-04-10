; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude PIDgains.msg.html

(cl:defclass <PIDgains> (roslisp-msg-protocol:ros-message)
  ((kp
    :reader kp
    :initarg :kp
    :type cl:float
    :initform 0.0)
   (kd
    :reader kd
    :initarg :kd
    :type cl:float
    :initform 0.0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:float
    :initform 0.0))
)

(cl:defclass PIDgains (<PIDgains>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PIDgains>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PIDgains)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<PIDgains> is deprecated: use owd_msgs-msg:PIDgains instead.")))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <PIDgains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:kp-val is deprecated.  Use owd_msgs-msg:kp instead.")
  (kp m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <PIDgains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:kd-val is deprecated.  Use owd_msgs-msg:kd instead.")
  (kd m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <PIDgains>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:ki-val is deprecated.  Use owd_msgs-msg:ki instead.")
  (ki m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PIDgains>) ostream)
  "Serializes a message object of type '<PIDgains>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ki))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PIDgains>) istream)
  "Deserializes a message object of type '<PIDgains>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ki) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PIDgains>)))
  "Returns string type for a message object of type '<PIDgains>"
  "owd_msgs/PIDgains")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PIDgains)))
  "Returns string type for a message object of type 'PIDgains"
  "owd_msgs/PIDgains")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PIDgains>)))
  "Returns md5sum for a message object of type '<PIDgains>"
  "e87782bd839ad4d19186c03a29ec1538")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PIDgains)))
  "Returns md5sum for a message object of type 'PIDgains"
  "e87782bd839ad4d19186c03a29ec1538")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PIDgains>)))
  "Returns full string definition for message of type '<PIDgains>"
  (cl:format cl:nil "float64 kp~%float64 kd~%float64 ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PIDgains)))
  "Returns full string definition for message of type 'PIDgains"
  (cl:format cl:nil "float64 kp~%float64 kd~%float64 ki~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PIDgains>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PIDgains>))
  "Converts a ROS message object to a list"
  (cl:list 'PIDgains
    (cl:cons ':kp (kp msg))
    (cl:cons ':kd (kd msg))
    (cl:cons ':ki (ki msg))
))
