; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetStiffness-request.msg.html

(cl:defclass <SetStiffness-request> (roslisp-msg-protocol:ros-message)
  ((stiffness
    :reader stiffness
    :initarg :stiffness
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetStiffness-request (<SetStiffness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStiffness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStiffness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetStiffness-request> is deprecated: use owd_msgs-srv:SetStiffness-request instead.")))

(cl:ensure-generic-function 'stiffness-val :lambda-list '(m))
(cl:defmethod stiffness-val ((m <SetStiffness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:stiffness-val is deprecated.  Use owd_msgs-srv:stiffness instead.")
  (stiffness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStiffness-request>) ostream)
  "Serializes a message object of type '<SetStiffness-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stiffness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStiffness-request>) istream)
  "Deserializes a message object of type '<SetStiffness-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stiffness) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStiffness-request>)))
  "Returns string type for a service object of type '<SetStiffness-request>"
  "owd_msgs/SetStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffness-request)))
  "Returns string type for a service object of type 'SetStiffness-request"
  "owd_msgs/SetStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStiffness-request>)))
  "Returns md5sum for a message object of type '<SetStiffness-request>"
  "aaa90fa03da8b85daed40a3e55f51144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStiffness-request)))
  "Returns md5sum for a message object of type 'SetStiffness-request"
  "aaa90fa03da8b85daed40a3e55f51144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStiffness-request>)))
  "Returns full string definition for message of type '<SetStiffness-request>"
  (cl:format cl:nil "float32 stiffness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStiffness-request)))
  "Returns full string definition for message of type 'SetStiffness-request"
  (cl:format cl:nil "float32 stiffness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStiffness-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStiffness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStiffness-request
    (cl:cons ':stiffness (stiffness msg))
))
;//! \htmlinclude SetStiffness-response.msg.html

(cl:defclass <SetStiffness-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetStiffness-response (<SetStiffness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStiffness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStiffness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetStiffness-response> is deprecated: use owd_msgs-srv:SetStiffness-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStiffness-response>) ostream)
  "Serializes a message object of type '<SetStiffness-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStiffness-response>) istream)
  "Deserializes a message object of type '<SetStiffness-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStiffness-response>)))
  "Returns string type for a service object of type '<SetStiffness-response>"
  "owd_msgs/SetStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffness-response)))
  "Returns string type for a service object of type 'SetStiffness-response"
  "owd_msgs/SetStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStiffness-response>)))
  "Returns md5sum for a message object of type '<SetStiffness-response>"
  "aaa90fa03da8b85daed40a3e55f51144")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStiffness-response)))
  "Returns md5sum for a message object of type 'SetStiffness-response"
  "aaa90fa03da8b85daed40a3e55f51144")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStiffness-response>)))
  "Returns full string definition for message of type '<SetStiffness-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStiffness-response)))
  "Returns full string definition for message of type 'SetStiffness-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStiffness-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStiffness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStiffness-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStiffness)))
  'SetStiffness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStiffness)))
  'SetStiffness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffness)))
  "Returns string type for a service object of type '<SetStiffness>"
  "owd_msgs/SetStiffness")