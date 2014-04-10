; Auto-generated. Do not edit!


(cl:in-package owd_msgs-msg)


;//! \htmlinclude JointTraj.msg.html

(cl:defclass <JointTraj> (roslisp-msg-protocol:ros-message)
  ((positions
    :reader positions
    :initarg :positions
    :type (cl:vector owd_msgs-msg:Joints)
   :initform (cl:make-array 0 :element-type 'owd_msgs-msg:Joints :initial-element (cl:make-instance 'owd_msgs-msg:Joints)))
   (blend_radius
    :reader blend_radius
    :initarg :blend_radius
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (options
    :reader options
    :initarg :options
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass JointTraj (<JointTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-msg:<JointTraj> is deprecated: use owd_msgs-msg:JointTraj instead.")))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <JointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:positions-val is deprecated.  Use owd_msgs-msg:positions instead.")
  (positions m))

(cl:ensure-generic-function 'blend_radius-val :lambda-list '(m))
(cl:defmethod blend_radius-val ((m <JointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:blend_radius-val is deprecated.  Use owd_msgs-msg:blend_radius instead.")
  (blend_radius m))

(cl:ensure-generic-function 'options-val :lambda-list '(m))
(cl:defmethod options-val ((m <JointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:options-val is deprecated.  Use owd_msgs-msg:options instead.")
  (options m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <JointTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-msg:id-val is deprecated.  Use owd_msgs-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JointTraj>)))
    "Constants for message type '<JointTraj>"
  '((:OPT_WAITFORSTART . 1)
    (:OPT_CANCELONSTALL . 2)
    (:OPT_CANCELONFORCEINPUT . 4)
    (:OPT_CANCELONTACTILEINPUT . 8)
    (:OPT_SYNCHRONIZE . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JointTraj)))
    "Constants for message type 'JointTraj"
  '((:OPT_WAITFORSTART . 1)
    (:OPT_CANCELONSTALL . 2)
    (:OPT_CANCELONFORCEINPUT . 4)
    (:OPT_CANCELONTACTILEINPUT . 8)
    (:OPT_SYNCHRONIZE . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTraj>) ostream)
  "Serializes a message object of type '<JointTraj>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'positions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blend_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'blend_radius))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'options)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTraj>) istream)
  "Deserializes a message object of type '<JointTraj>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'owd_msgs-msg:Joints))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blend_radius) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blend_radius)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTraj>)))
  "Returns string type for a message object of type '<JointTraj>"
  "owd_msgs/JointTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTraj)))
  "Returns string type for a message object of type 'JointTraj"
  "owd_msgs/JointTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTraj>)))
  "Returns md5sum for a message object of type '<JointTraj>"
  "c624daf7add91fa456a8f89affd11db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTraj)))
  "Returns md5sum for a message object of type 'JointTraj"
  "c624daf7add91fa456a8f89affd11db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTraj>)))
  "Returns full string definition for message of type '<JointTraj>"
  (cl:format cl:nil "owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTraj)))
  "Returns full string definition for message of type 'JointTraj"
  (cl:format cl:nil "owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTraj>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blend_radius) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTraj
    (cl:cons ':positions (positions msg))
    (cl:cons ':blend_radius (blend_radius msg))
    (cl:cons ':options (options msg))
    (cl:cons ':id (id msg))
))
