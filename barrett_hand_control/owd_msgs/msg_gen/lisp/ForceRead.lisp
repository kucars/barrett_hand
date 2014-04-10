; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude ForceRead.msg.html

(cl:defclass <ForceRead> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ForceRead (<ForceRead>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ForceRead>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ForceRead)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<ForceRead> is deprecated: use owd_msgs-msg:ForceRead instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <ForceRead>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:force-val is deprecated.  Use owd_msgs-msg:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ForceRead>) ostream)
  "Serializes a message object of type '<ForceRead>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'force))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ForceRead>) istream)
  "Deserializes a message object of type '<ForceRead>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'force) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'force)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ForceRead>)))
  "Returns string type for a message object of type '<ForceRead>"
  "owd_msgs/ForceRead")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ForceRead)))
  "Returns string type for a message object of type 'ForceRead"
  "owd_msgs/ForceRead")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ForceRead>)))
  "Returns md5sum for a message object of type '<ForceRead>"
  "23399487a2048efabaa375690609f3b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ForceRead)))
  "Returns md5sum for a message object of type 'ForceRead"
  "23399487a2048efabaa375690609f3b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ForceRead>)))
  "Returns full string definition for message of type '<ForceRead>"
  (cl:format cl:nil "float64[] force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ForceRead)))
  "Returns full string definition for message of type 'ForceRead"
  (cl:format cl:nil "float64[] force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ForceRead>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'force) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ForceRead>))
  "Converts a ROS message object to a list"
  (cl:list 'ForceRead
    (cl:cons ':force (force msg))
))
