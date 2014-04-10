; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetTactileInputThreshold-request.msg.html

(cl:defclass <SetTactileInputThreshold-request> (roslisp-msg-protocol:ros-message)
  ((pad_number
    :reader pad_number
    :initarg :pad_number
    :type cl:integer
    :initform 0)
   (threshold
    :reader threshold
    :initarg :threshold
    :type cl:float
    :initform 0.0)
   (minimum_readings
    :reader minimum_readings
    :initarg :minimum_readings
    :type cl:integer
    :initform 0))
)

(cl:defclass SetTactileInputThreshold-request (<SetTactileInputThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTactileInputThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTactileInputThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetTactileInputThreshold-request> is deprecated: use owd_msgs-srv:SetTactileInputThreshold-request instead.")))

(cl:ensure-generic-function 'pad_number-val :lambda-list '(m))
(cl:defmethod pad_number-val ((m <SetTactileInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:pad_number-val is deprecated.  Use owd_msgs-srv:pad_number instead.")
  (pad_number m))

(cl:ensure-generic-function 'threshold-val :lambda-list '(m))
(cl:defmethod threshold-val ((m <SetTactileInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:threshold-val is deprecated.  Use owd_msgs-srv:threshold instead.")
  (threshold m))

(cl:ensure-generic-function 'minimum_readings-val :lambda-list '(m))
(cl:defmethod minimum_readings-val ((m <SetTactileInputThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:minimum_readings-val is deprecated.  Use owd_msgs-srv:minimum_readings instead.")
  (minimum_readings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTactileInputThreshold-request>) ostream)
  "Serializes a message object of type '<SetTactileInputThreshold-request>"
  (cl:let* ((signed (cl:slot-value msg 'pad_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'minimum_readings)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTactileInputThreshold-request>) istream)
  "Deserializes a message object of type '<SetTactileInputThreshold-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pad_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'minimum_readings) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTactileInputThreshold-request>)))
  "Returns string type for a service object of type '<SetTactileInputThreshold-request>"
  "owd_msgs/SetTactileInputThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileInputThreshold-request)))
  "Returns string type for a service object of type 'SetTactileInputThreshold-request"
  "owd_msgs/SetTactileInputThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTactileInputThreshold-request>)))
  "Returns md5sum for a message object of type '<SetTactileInputThreshold-request>"
  "7205417324d66b6ed0188e5a03a48908")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTactileInputThreshold-request)))
  "Returns md5sum for a message object of type 'SetTactileInputThreshold-request"
  "7205417324d66b6ed0188e5a03a48908")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTactileInputThreshold-request>)))
  "Returns full string definition for message of type '<SetTactileInputThreshold-request>"
  (cl:format cl:nil "int32 pad_number~%float32 threshold~%int32 minimum_readings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTactileInputThreshold-request)))
  "Returns full string definition for message of type 'SetTactileInputThreshold-request"
  (cl:format cl:nil "int32 pad_number~%float32 threshold~%int32 minimum_readings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTactileInputThreshold-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTactileInputThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTactileInputThreshold-request
    (cl:cons ':pad_number (pad_number msg))
    (cl:cons ':threshold (threshold msg))
    (cl:cons ':minimum_readings (minimum_readings msg))
))
;//! \htmlinclude SetTactileInputThreshold-response.msg.html

(cl:defclass <SetTactileInputThreshold-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetTactileInputThreshold-response (<SetTactileInputThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTactileInputThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTactileInputThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetTactileInputThreshold-response> is deprecated: use owd_msgs-srv:SetTactileInputThreshold-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetTactileInputThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetTactileInputThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTactileInputThreshold-response>) ostream)
  "Serializes a message object of type '<SetTactileInputThreshold-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTactileInputThreshold-response>) istream)
  "Deserializes a message object of type '<SetTactileInputThreshold-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTactileInputThreshold-response>)))
  "Returns string type for a service object of type '<SetTactileInputThreshold-response>"
  "owd_msgs/SetTactileInputThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileInputThreshold-response)))
  "Returns string type for a service object of type 'SetTactileInputThreshold-response"
  "owd_msgs/SetTactileInputThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTactileInputThreshold-response>)))
  "Returns md5sum for a message object of type '<SetTactileInputThreshold-response>"
  "7205417324d66b6ed0188e5a03a48908")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTactileInputThreshold-response)))
  "Returns md5sum for a message object of type 'SetTactileInputThreshold-response"
  "7205417324d66b6ed0188e5a03a48908")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTactileInputThreshold-response>)))
  "Returns full string definition for message of type '<SetTactileInputThreshold-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTactileInputThreshold-response)))
  "Returns full string definition for message of type 'SetTactileInputThreshold-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTactileInputThreshold-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTactileInputThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTactileInputThreshold-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTactileInputThreshold)))
  'SetTactileInputThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTactileInputThreshold)))
  'SetTactileInputThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTactileInputThreshold)))
  "Returns string type for a service object of type '<SetTactileInputThreshold>"
  "owd_msgs/SetTactileInputThreshold")