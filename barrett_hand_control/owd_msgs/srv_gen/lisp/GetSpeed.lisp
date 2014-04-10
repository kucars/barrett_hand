; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude GetSpeed-request.msg.html

(cl:defclass <GetSpeed-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetSpeed-request (<GetSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetSpeed-request> is deprecated: use owd_msgs-srv:GetSpeed-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSpeed-request>) ostream)
  "Serializes a message object of type '<GetSpeed-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSpeed-request>) istream)
  "Deserializes a message object of type '<GetSpeed-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSpeed-request>)))
  "Returns string type for a service object of type '<GetSpeed-request>"
  "owd_msgs/GetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed-request)))
  "Returns string type for a service object of type 'GetSpeed-request"
  "owd_msgs/GetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSpeed-request>)))
  "Returns md5sum for a message object of type '<GetSpeed-request>"
  "804e50610c7476a6ce4d10a645d1448f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSpeed-request)))
  "Returns md5sum for a message object of type 'GetSpeed-request"
  "804e50610c7476a6ce4d10a645d1448f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSpeed-request>)))
  "Returns full string definition for message of type '<GetSpeed-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSpeed-request)))
  "Returns full string definition for message of type 'GetSpeed-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSpeed-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSpeed-request
))
;//! \htmlinclude GetSpeed-response.msg.html

(cl:defclass <GetSpeed-response> (roslisp-msg-protocol:ros-message)
  ((max_velocity
    :reader max_velocity
    :initarg :max_velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (max_acceleration
    :reader max_acceleration
    :initarg :max_acceleration
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (max_jerk
    :reader max_jerk
    :initarg :max_jerk
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetSpeed-response (<GetSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetSpeed-response> is deprecated: use owd_msgs-srv:GetSpeed-response instead.")))

(cl:ensure-generic-function 'max_velocity-val :lambda-list '(m))
(cl:defmethod max_velocity-val ((m <GetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:max_velocity-val is deprecated.  Use owd_msgs-srv:max_velocity instead.")
  (max_velocity m))

(cl:ensure-generic-function 'max_acceleration-val :lambda-list '(m))
(cl:defmethod max_acceleration-val ((m <GetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:max_acceleration-val is deprecated.  Use owd_msgs-srv:max_acceleration instead.")
  (max_acceleration m))

(cl:ensure-generic-function 'max_jerk-val :lambda-list '(m))
(cl:defmethod max_jerk-val ((m <GetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:max_jerk-val is deprecated.  Use owd_msgs-srv:max_jerk instead.")
  (max_jerk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSpeed-response>) ostream)
  "Serializes a message object of type '<GetSpeed-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'max_velocity))))
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
   (cl:slot-value msg 'max_velocity))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'max_acceleration))))
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
   (cl:slot-value msg 'max_acceleration))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_jerk))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSpeed-response>) istream)
  "Deserializes a message object of type '<GetSpeed-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'max_velocity) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'max_velocity)))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'max_acceleration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'max_acceleration)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_jerk) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSpeed-response>)))
  "Returns string type for a service object of type '<GetSpeed-response>"
  "owd_msgs/GetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed-response)))
  "Returns string type for a service object of type 'GetSpeed-response"
  "owd_msgs/GetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSpeed-response>)))
  "Returns md5sum for a message object of type '<GetSpeed-response>"
  "804e50610c7476a6ce4d10a645d1448f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSpeed-response)))
  "Returns md5sum for a message object of type 'GetSpeed-response"
  "804e50610c7476a6ce4d10a645d1448f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSpeed-response>)))
  "Returns full string definition for message of type '<GetSpeed-response>"
  (cl:format cl:nil "float64[] max_velocity~%float64[] max_acceleration~%float64 max_jerk~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSpeed-response)))
  "Returns full string definition for message of type 'GetSpeed-response"
  (cl:format cl:nil "float64[] max_velocity~%float64[] max_acceleration~%float64 max_jerk~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSpeed-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'max_velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'max_acceleration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSpeed-response
    (cl:cons ':max_velocity (max_velocity msg))
    (cl:cons ':max_acceleration (max_acceleration msg))
    (cl:cons ':max_jerk (max_jerk msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSpeed)))
  'GetSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSpeed)))
  'GetSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSpeed)))
  "Returns string type for a service object of type '<GetSpeed>"
  "owd_msgs/GetSpeed")