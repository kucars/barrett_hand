; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetHandProperty-request.msg.html

(cl:defclass <SetHandProperty-request> (roslisp-msg-protocol:ros-message)
  ((nodeid
    :reader nodeid
    :initarg :nodeid
    :type cl:integer
    :initform 0)
   (property
    :reader property
    :initarg :property
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass SetHandProperty-request (<SetHandProperty-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHandProperty-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHandProperty-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetHandProperty-request> is deprecated: use owd_msgs-srv:SetHandProperty-request instead.")))

(cl:ensure-generic-function 'nodeid-val :lambda-list '(m))
(cl:defmethod nodeid-val ((m <SetHandProperty-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:nodeid-val is deprecated.  Use owd_msgs-srv:nodeid instead.")
  (nodeid m))

(cl:ensure-generic-function 'property-val :lambda-list '(m))
(cl:defmethod property-val ((m <SetHandProperty-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:property-val is deprecated.  Use owd_msgs-srv:property instead.")
  (property m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetHandProperty-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:value-val is deprecated.  Use owd_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHandProperty-request>) ostream)
  "Serializes a message object of type '<SetHandProperty-request>"
  (cl:let* ((signed (cl:slot-value msg 'nodeid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'property)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHandProperty-request>) istream)
  "Deserializes a message object of type '<SetHandProperty-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeid) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'property) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHandProperty-request>)))
  "Returns string type for a service object of type '<SetHandProperty-request>"
  "owd_msgs/SetHandPropertyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandProperty-request)))
  "Returns string type for a service object of type 'SetHandProperty-request"
  "owd_msgs/SetHandPropertyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHandProperty-request>)))
  "Returns md5sum for a message object of type '<SetHandProperty-request>"
  "24011110d79668edbbae5577142fb799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHandProperty-request)))
  "Returns md5sum for a message object of type 'SetHandProperty-request"
  "24011110d79668edbbae5577142fb799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHandProperty-request>)))
  "Returns full string definition for message of type '<SetHandProperty-request>"
  (cl:format cl:nil "int32 nodeid~%int32 property~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHandProperty-request)))
  "Returns full string definition for message of type 'SetHandProperty-request"
  (cl:format cl:nil "int32 nodeid~%int32 property~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHandProperty-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHandProperty-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHandProperty-request
    (cl:cons ':nodeid (nodeid msg))
    (cl:cons ':property (property msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetHandProperty-response.msg.html

(cl:defclass <SetHandProperty-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetHandProperty-response (<SetHandProperty-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHandProperty-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHandProperty-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetHandProperty-response> is deprecated: use owd_msgs-srv:SetHandProperty-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetHandProperty-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetHandProperty-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHandProperty-response>) ostream)
  "Serializes a message object of type '<SetHandProperty-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHandProperty-response>) istream)
  "Deserializes a message object of type '<SetHandProperty-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHandProperty-response>)))
  "Returns string type for a service object of type '<SetHandProperty-response>"
  "owd_msgs/SetHandPropertyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandProperty-response)))
  "Returns string type for a service object of type 'SetHandProperty-response"
  "owd_msgs/SetHandPropertyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHandProperty-response>)))
  "Returns md5sum for a message object of type '<SetHandProperty-response>"
  "24011110d79668edbbae5577142fb799")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHandProperty-response)))
  "Returns md5sum for a message object of type 'SetHandProperty-response"
  "24011110d79668edbbae5577142fb799")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHandProperty-response>)))
  "Returns full string definition for message of type '<SetHandProperty-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHandProperty-response)))
  "Returns full string definition for message of type 'SetHandProperty-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHandProperty-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHandProperty-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHandProperty-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHandProperty)))
  'SetHandProperty-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHandProperty)))
  'SetHandProperty-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandProperty)))
  "Returns string type for a service object of type '<SetHandProperty>"
  "owd_msgs/SetHandProperty")