; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetHandTorque-request.msg.html

(cl:defclass <SetHandTorque-request> (roslisp-msg-protocol:ros-message)
  ((initial
    :reader initial
    :initarg :initial
    :type cl:integer
    :initform 0)
   (sustained
    :reader sustained
    :initarg :sustained
    :type cl:integer
    :initform 0))
)

(cl:defclass SetHandTorque-request (<SetHandTorque-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHandTorque-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHandTorque-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetHandTorque-request> is deprecated: use owd_msgs-srv:SetHandTorque-request instead.")))

(cl:ensure-generic-function 'initial-val :lambda-list '(m))
(cl:defmethod initial-val ((m <SetHandTorque-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:initial-val is deprecated.  Use owd_msgs-srv:initial instead.")
  (initial m))

(cl:ensure-generic-function 'sustained-val :lambda-list '(m))
(cl:defmethod sustained-val ((m <SetHandTorque-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:sustained-val is deprecated.  Use owd_msgs-srv:sustained instead.")
  (sustained m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHandTorque-request>) ostream)
  "Serializes a message object of type '<SetHandTorque-request>"
  (cl:let* ((signed (cl:slot-value msg 'initial)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sustained)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHandTorque-request>) istream)
  "Deserializes a message object of type '<SetHandTorque-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'initial) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sustained) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHandTorque-request>)))
  "Returns string type for a service object of type '<SetHandTorque-request>"
  "owd_msgs/SetHandTorqueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandTorque-request)))
  "Returns string type for a service object of type 'SetHandTorque-request"
  "owd_msgs/SetHandTorqueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHandTorque-request>)))
  "Returns md5sum for a message object of type '<SetHandTorque-request>"
  "25f05b3c85731e4fc054b0dcb3c9b8a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHandTorque-request)))
  "Returns md5sum for a message object of type 'SetHandTorque-request"
  "25f05b3c85731e4fc054b0dcb3c9b8a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHandTorque-request>)))
  "Returns full string definition for message of type '<SetHandTorque-request>"
  (cl:format cl:nil "int32 initial~%int32 sustained~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHandTorque-request)))
  "Returns full string definition for message of type 'SetHandTorque-request"
  (cl:format cl:nil "int32 initial~%int32 sustained~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHandTorque-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHandTorque-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHandTorque-request
    (cl:cons ':initial (initial msg))
    (cl:cons ':sustained (sustained msg))
))
;//! \htmlinclude SetHandTorque-response.msg.html

(cl:defclass <SetHandTorque-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetHandTorque-response (<SetHandTorque-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHandTorque-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHandTorque-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetHandTorque-response> is deprecated: use owd_msgs-srv:SetHandTorque-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetHandTorque-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetHandTorque-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHandTorque-response>) ostream)
  "Serializes a message object of type '<SetHandTorque-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHandTorque-response>) istream)
  "Deserializes a message object of type '<SetHandTorque-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHandTorque-response>)))
  "Returns string type for a service object of type '<SetHandTorque-response>"
  "owd_msgs/SetHandTorqueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandTorque-response)))
  "Returns string type for a service object of type 'SetHandTorque-response"
  "owd_msgs/SetHandTorqueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHandTorque-response>)))
  "Returns md5sum for a message object of type '<SetHandTorque-response>"
  "25f05b3c85731e4fc054b0dcb3c9b8a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHandTorque-response)))
  "Returns md5sum for a message object of type 'SetHandTorque-response"
  "25f05b3c85731e4fc054b0dcb3c9b8a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHandTorque-response>)))
  "Returns full string definition for message of type '<SetHandTorque-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHandTorque-response)))
  "Returns full string definition for message of type 'SetHandTorque-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHandTorque-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHandTorque-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHandTorque-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHandTorque)))
  'SetHandTorque-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHandTorque)))
  'SetHandTorque-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHandTorque)))
  "Returns string type for a service object of type '<SetHandTorque>"
  "owd_msgs/SetHandTorque")