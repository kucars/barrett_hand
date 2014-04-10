; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude WamSetupSeaCtrl.msg.html

(cl:defclass <WamSetupSeaCtrl> (roslisp-msg-protocol:ros-message)
  ((jointIndices
    :reader jointIndices
    :initarg :jointIndices
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (values
    :reader values
    :initarg :values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass WamSetupSeaCtrl (<WamSetupSeaCtrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WamSetupSeaCtrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WamSetupSeaCtrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<WamSetupSeaCtrl> is deprecated: use owd_msgs-msg:WamSetupSeaCtrl instead.")))

(cl:ensure-generic-function 'jointIndices-val :lambda-list '(m))
(cl:defmethod jointIndices-val ((m <WamSetupSeaCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:jointIndices-val is deprecated.  Use owd_msgs-msg:jointIndices instead.")
  (jointIndices m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <WamSetupSeaCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:values-val is deprecated.  Use owd_msgs-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WamSetupSeaCtrl>)))
    "Constants for message type '<WamSetupSeaCtrl>"
  '((:RETAINARMSTATE . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WamSetupSeaCtrl)))
    "Constants for message type 'WamSetupSeaCtrl"
  '((:RETAINARMSTATE . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WamSetupSeaCtrl>) ostream)
  "Serializes a message object of type '<WamSetupSeaCtrl>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jointIndices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'jointIndices))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
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
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WamSetupSeaCtrl>) istream)
  "Deserializes a message object of type '<WamSetupSeaCtrl>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jointIndices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jointIndices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WamSetupSeaCtrl>)))
  "Returns string type for a message object of type '<WamSetupSeaCtrl>"
  "owd_msgs/WamSetupSeaCtrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WamSetupSeaCtrl)))
  "Returns string type for a message object of type 'WamSetupSeaCtrl"
  "owd_msgs/WamSetupSeaCtrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WamSetupSeaCtrl>)))
  "Returns md5sum for a message object of type '<WamSetupSeaCtrl>"
  "6714f77d58bb6129af5ebca9f942b31e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WamSetupSeaCtrl)))
  "Returns md5sum for a message object of type 'WamSetupSeaCtrl"
  "6714f77d58bb6129af5ebca9f942b31e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WamSetupSeaCtrl>)))
  "Returns full string definition for message of type '<WamSetupSeaCtrl>"
  (cl:format cl:nil "uint8 retainArmState = 0~%int32[] jointIndices~%float64[] values ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WamSetupSeaCtrl)))
  "Returns full string definition for message of type 'WamSetupSeaCtrl"
  (cl:format cl:nil "uint8 retainArmState = 0~%int32[] jointIndices~%float64[] values ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WamSetupSeaCtrl>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointIndices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WamSetupSeaCtrl>))
  "Converts a ROS message object to a list"
  (cl:list 'WamSetupSeaCtrl
    (cl:cons ':jointIndices (jointIndices msg))
    (cl:cons ':values (values msg))
))
