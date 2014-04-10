; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetJointOffsets-request.msg.html

(cl:defclass <SetJointOffsets-request> (roslisp-msg-protocol:ros-message)
  ((offset
    :reader offset
    :initarg :offset
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetJointOffsets-request (<SetJointOffsets-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointOffsets-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointOffsets-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetJointOffsets-request> is deprecated: use owd_msgs-srv:SetJointOffsets-request instead.")))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <SetJointOffsets-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:offset-val is deprecated.  Use owd_msgs-srv:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointOffsets-request>) ostream)
  "Serializes a message object of type '<SetJointOffsets-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'offset))))
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
   (cl:slot-value msg 'offset))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointOffsets-request>) istream)
  "Deserializes a message object of type '<SetJointOffsets-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'offset) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'offset)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointOffsets-request>)))
  "Returns string type for a service object of type '<SetJointOffsets-request>"
  "owd_msgs/SetJointOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointOffsets-request)))
  "Returns string type for a service object of type 'SetJointOffsets-request"
  "owd_msgs/SetJointOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointOffsets-request>)))
  "Returns md5sum for a message object of type '<SetJointOffsets-request>"
  "7a8655e3c68fc4de24d05c081bb29ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointOffsets-request)))
  "Returns md5sum for a message object of type 'SetJointOffsets-request"
  "7a8655e3c68fc4de24d05c081bb29ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointOffsets-request>)))
  "Returns full string definition for message of type '<SetJointOffsets-request>"
  (cl:format cl:nil "float64[] offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointOffsets-request)))
  "Returns full string definition for message of type 'SetJointOffsets-request"
  (cl:format cl:nil "float64[] offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointOffsets-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'offset) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointOffsets-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointOffsets-request
    (cl:cons ':offset (offset msg))
))
;//! \htmlinclude SetJointOffsets-response.msg.html

(cl:defclass <SetJointOffsets-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass SetJointOffsets-response (<SetJointOffsets-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointOffsets-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointOffsets-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetJointOffsets-response> is deprecated: use owd_msgs-srv:SetJointOffsets-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetJointOffsets-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetJointOffsets-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointOffsets-response>) ostream)
  "Serializes a message object of type '<SetJointOffsets-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointOffsets-response>) istream)
  "Deserializes a message object of type '<SetJointOffsets-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointOffsets-response>)))
  "Returns string type for a service object of type '<SetJointOffsets-response>"
  "owd_msgs/SetJointOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointOffsets-response)))
  "Returns string type for a service object of type 'SetJointOffsets-response"
  "owd_msgs/SetJointOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointOffsets-response>)))
  "Returns md5sum for a message object of type '<SetJointOffsets-response>"
  "7a8655e3c68fc4de24d05c081bb29ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointOffsets-response)))
  "Returns md5sum for a message object of type 'SetJointOffsets-response"
  "7a8655e3c68fc4de24d05c081bb29ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointOffsets-response>)))
  "Returns full string definition for message of type '<SetJointOffsets-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointOffsets-response)))
  "Returns full string definition for message of type 'SetJointOffsets-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointOffsets-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointOffsets-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointOffsets-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointOffsets)))
  'SetJointOffsets-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointOffsets)))
  'SetJointOffsets-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointOffsets)))
  "Returns string type for a service object of type '<SetJointOffsets>"
  "owd_msgs/SetJointOffsets")