; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetTorqueLimits-request.msg.html

(cl:defclass <SetTorqueLimits-request> (roslisp-msg-protocol:ros-message)
  ((limit
    :reader limit
    :initarg :limit
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass SetTorqueLimits-request (<SetTorqueLimits-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueLimits-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueLimits-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetTorqueLimits-request> is deprecated: use owd_msgs-srv:SetTorqueLimits-request instead.")))

(cl:ensure-generic-function 'limit-val :lambda-list '(m))
(cl:defmethod limit-val ((m <SetTorqueLimits-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:limit-val is deprecated.  Use owd_msgs-srv:limit instead.")
  (limit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueLimits-request>) ostream)
  "Serializes a message object of type '<SetTorqueLimits-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limit))))
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
   (cl:slot-value msg 'limit))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueLimits-request>) istream)
  "Deserializes a message object of type '<SetTorqueLimits-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limit) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limit)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueLimits-request>)))
  "Returns string type for a service object of type '<SetTorqueLimits-request>"
  "owd_msgs/SetTorqueLimitsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimits-request)))
  "Returns string type for a service object of type 'SetTorqueLimits-request"
  "owd_msgs/SetTorqueLimitsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueLimits-request>)))
  "Returns md5sum for a message object of type '<SetTorqueLimits-request>"
  "fefdf1bcc63e207d3eeb94abd226a47f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueLimits-request)))
  "Returns md5sum for a message object of type 'SetTorqueLimits-request"
  "fefdf1bcc63e207d3eeb94abd226a47f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueLimits-request>)))
  "Returns full string definition for message of type '<SetTorqueLimits-request>"
  (cl:format cl:nil "int32[] limit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueLimits-request)))
  "Returns full string definition for message of type 'SetTorqueLimits-request"
  (cl:format cl:nil "int32[] limit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueLimits-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limit) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueLimits-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueLimits-request
    (cl:cons ':limit (limit msg))
))
;//! \htmlinclude SetTorqueLimits-response.msg.html

(cl:defclass <SetTorqueLimits-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetTorqueLimits-response (<SetTorqueLimits-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueLimits-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueLimits-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetTorqueLimits-response> is deprecated: use owd_msgs-srv:SetTorqueLimits-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetTorqueLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetTorqueLimits-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueLimits-response>) ostream)
  "Serializes a message object of type '<SetTorqueLimits-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueLimits-response>) istream)
  "Deserializes a message object of type '<SetTorqueLimits-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueLimits-response>)))
  "Returns string type for a service object of type '<SetTorqueLimits-response>"
  "owd_msgs/SetTorqueLimitsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimits-response)))
  "Returns string type for a service object of type 'SetTorqueLimits-response"
  "owd_msgs/SetTorqueLimitsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueLimits-response>)))
  "Returns md5sum for a message object of type '<SetTorqueLimits-response>"
  "fefdf1bcc63e207d3eeb94abd226a47f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueLimits-response)))
  "Returns md5sum for a message object of type 'SetTorqueLimits-response"
  "fefdf1bcc63e207d3eeb94abd226a47f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueLimits-response>)))
  "Returns full string definition for message of type '<SetTorqueLimits-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueLimits-response)))
  "Returns full string definition for message of type 'SetTorqueLimits-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueLimits-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueLimits-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueLimits-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTorqueLimits)))
  'SetTorqueLimits-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTorqueLimits)))
  'SetTorqueLimits-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimits)))
  "Returns string type for a service object of type '<SetTorqueLimits>"
  "owd_msgs/SetTorqueLimits")