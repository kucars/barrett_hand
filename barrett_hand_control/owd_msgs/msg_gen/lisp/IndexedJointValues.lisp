; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude IndexedJointValues.msg.html

(cl:defclass <IndexedJointValues> (roslisp-msg-protocol:ros-message)
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

(cl:defclass IndexedJointValues (<IndexedJointValues>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IndexedJointValues>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IndexedJointValues)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<IndexedJointValues> is deprecated: use owd_msgs-msg:IndexedJointValues instead.")))

(cl:ensure-generic-function 'jointIndices-val :lambda-list '(m))
(cl:defmethod jointIndices-val ((m <IndexedJointValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:jointIndices-val is deprecated.  Use owd_msgs-msg:jointIndices instead.")
  (jointIndices m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <IndexedJointValues>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:values-val is deprecated.  Use owd_msgs-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IndexedJointValues>) ostream)
  "Serializes a message object of type '<IndexedJointValues>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IndexedJointValues>) istream)
  "Deserializes a message object of type '<IndexedJointValues>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IndexedJointValues>)))
  "Returns string type for a message object of type '<IndexedJointValues>"
  "owd_msgs/IndexedJointValues")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IndexedJointValues)))
  "Returns string type for a message object of type 'IndexedJointValues"
  "owd_msgs/IndexedJointValues")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IndexedJointValues>)))
  "Returns md5sum for a message object of type '<IndexedJointValues>"
  "02a472aa60e521a148bf4000eda4e325")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IndexedJointValues)))
  "Returns md5sum for a message object of type 'IndexedJointValues"
  "02a472aa60e521a148bf4000eda4e325")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IndexedJointValues>)))
  "Returns full string definition for message of type '<IndexedJointValues>"
  (cl:format cl:nil "# jointIndices start at 1~%int32[] jointIndices~%float64[] values ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IndexedJointValues)))
  "Returns full string definition for message of type 'IndexedJointValues"
  (cl:format cl:nil "# jointIndices start at 1~%int32[] jointIndices~%float64[] values ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IndexedJointValues>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointIndices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IndexedJointValues>))
  "Converts a ROS message object to a list"
  (cl:list 'IndexedJointValues
    (cl:cons ':jointIndices (jointIndices msg))
    (cl:cons ':values (values msg))
))
