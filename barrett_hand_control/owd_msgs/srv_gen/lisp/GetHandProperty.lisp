; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude GetHandProperty-request.msg.html

(cl:defclass <GetHandProperty-request> (roslisp-msg-protocol:ros-message)
  ((nodeid
    :reader nodeid
    :initarg :nodeid
    :type cl:integer
    :initform 0)
   (property
    :reader property
    :initarg :property
    :type cl:integer
    :initform 0))
)

(cl:defclass GetHandProperty-request (<GetHandProperty-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHandProperty-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHandProperty-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetHandProperty-request> is deprecated: use owd_msgs-srv:GetHandProperty-request instead.")))

(cl:ensure-generic-function 'nodeid-val :lambda-list '(m))
(cl:defmethod nodeid-val ((m <GetHandProperty-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:nodeid-val is deprecated.  Use owd_msgs-srv:nodeid instead.")
  (nodeid m))

(cl:ensure-generic-function 'property-val :lambda-list '(m))
(cl:defmethod property-val ((m <GetHandProperty-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:property-val is deprecated.  Use owd_msgs-srv:property instead.")
  (property m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHandProperty-request>) ostream)
  "Serializes a message object of type '<GetHandProperty-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHandProperty-request>) istream)
  "Deserializes a message object of type '<GetHandProperty-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHandProperty-request>)))
  "Returns string type for a service object of type '<GetHandProperty-request>"
  "owd_msgs/GetHandPropertyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandProperty-request)))
  "Returns string type for a service object of type 'GetHandProperty-request"
  "owd_msgs/GetHandPropertyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHandProperty-request>)))
  "Returns md5sum for a message object of type '<GetHandProperty-request>"
  "2dcf1331b4912a45435689f10658fe42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHandProperty-request)))
  "Returns md5sum for a message object of type 'GetHandProperty-request"
  "2dcf1331b4912a45435689f10658fe42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHandProperty-request>)))
  "Returns full string definition for message of type '<GetHandProperty-request>"
  (cl:format cl:nil "int32 nodeid~%int32 property~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHandProperty-request)))
  "Returns full string definition for message of type 'GetHandProperty-request"
  (cl:format cl:nil "int32 nodeid~%int32 property~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHandProperty-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHandProperty-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHandProperty-request
    (cl:cons ':nodeid (nodeid msg))
    (cl:cons ':property (property msg))
))
;//! \htmlinclude GetHandProperty-response.msg.html

(cl:defclass <GetHandProperty-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0)
   (ok
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

(cl:defclass GetHandProperty-response (<GetHandProperty-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHandProperty-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHandProperty-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetHandProperty-response> is deprecated: use owd_msgs-srv:GetHandProperty-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetHandProperty-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:value-val is deprecated.  Use owd_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <GetHandProperty-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <GetHandProperty-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHandProperty-response>) ostream)
  "Serializes a message object of type '<GetHandProperty-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHandProperty-response>) istream)
  "Deserializes a message object of type '<GetHandProperty-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHandProperty-response>)))
  "Returns string type for a service object of type '<GetHandProperty-response>"
  "owd_msgs/GetHandPropertyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandProperty-response)))
  "Returns string type for a service object of type 'GetHandProperty-response"
  "owd_msgs/GetHandPropertyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHandProperty-response>)))
  "Returns md5sum for a message object of type '<GetHandProperty-response>"
  "2dcf1331b4912a45435689f10658fe42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHandProperty-response)))
  "Returns md5sum for a message object of type 'GetHandProperty-response"
  "2dcf1331b4912a45435689f10658fe42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHandProperty-response>)))
  "Returns full string definition for message of type '<GetHandProperty-response>"
  (cl:format cl:nil "int32 value~%bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHandProperty-response)))
  "Returns full string definition for message of type 'GetHandProperty-response"
  (cl:format cl:nil "int32 value~%bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHandProperty-response>))
  (cl:+ 0
     4
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHandProperty-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHandProperty-response
    (cl:cons ':value (value msg))
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHandProperty)))
  'GetHandProperty-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHandProperty)))
  'GetHandProperty-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHandProperty)))
  "Returns string type for a service object of type '<GetHandProperty>"
  "owd_msgs/GetHandProperty")