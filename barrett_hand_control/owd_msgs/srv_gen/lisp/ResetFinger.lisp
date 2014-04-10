; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude ResetFinger-request.msg.html

(cl:defclass <ResetFinger-request> (roslisp-msg-protocol:ros-message)
  ((finger
    :reader finger
    :initarg :finger
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ResetFinger-request (<ResetFinger-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetFinger-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetFinger-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ResetFinger-request> is deprecated: use owd_msgs-srv:ResetFinger-request instead.")))

(cl:ensure-generic-function 'finger-val :lambda-list '(m))
(cl:defmethod finger-val ((m <ResetFinger-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:finger-val is deprecated.  Use owd_msgs-srv:finger instead.")
  (finger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetFinger-request>) ostream)
  "Serializes a message object of type '<ResetFinger-request>"
  (cl:let* ((signed (cl:slot-value msg 'finger)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetFinger-request>) istream)
  "Deserializes a message object of type '<ResetFinger-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'finger) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetFinger-request>)))
  "Returns string type for a service object of type '<ResetFinger-request>"
  "owd_msgs/ResetFingerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetFinger-request)))
  "Returns string type for a service object of type 'ResetFinger-request"
  "owd_msgs/ResetFingerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetFinger-request>)))
  "Returns md5sum for a message object of type '<ResetFinger-request>"
  "0e29cc56e438836f4dcc31646927be30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetFinger-request)))
  "Returns md5sum for a message object of type 'ResetFinger-request"
  "0e29cc56e438836f4dcc31646927be30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetFinger-request>)))
  "Returns full string definition for message of type '<ResetFinger-request>"
  (cl:format cl:nil "int8 finger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetFinger-request)))
  "Returns full string definition for message of type 'ResetFinger-request"
  (cl:format cl:nil "int8 finger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetFinger-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetFinger-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetFinger-request
    (cl:cons ':finger (finger msg))
))
;//! \htmlinclude ResetFinger-response.msg.html

(cl:defclass <ResetFinger-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ResetFinger-response (<ResetFinger-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetFinger-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetFinger-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ResetFinger-response> is deprecated: use owd_msgs-srv:ResetFinger-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <ResetFinger-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <ResetFinger-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetFinger-response>) ostream)
  "Serializes a message object of type '<ResetFinger-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetFinger-response>) istream)
  "Deserializes a message object of type '<ResetFinger-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetFinger-response>)))
  "Returns string type for a service object of type '<ResetFinger-response>"
  "owd_msgs/ResetFingerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetFinger-response)))
  "Returns string type for a service object of type 'ResetFinger-response"
  "owd_msgs/ResetFingerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetFinger-response>)))
  "Returns md5sum for a message object of type '<ResetFinger-response>"
  "0e29cc56e438836f4dcc31646927be30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetFinger-response)))
  "Returns md5sum for a message object of type 'ResetFinger-response"
  "0e29cc56e438836f4dcc31646927be30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetFinger-response>)))
  "Returns full string definition for message of type '<ResetFinger-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetFinger-response)))
  "Returns full string definition for message of type 'ResetFinger-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetFinger-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetFinger-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetFinger-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetFinger)))
  'ResetFinger-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetFinger)))
  'ResetFinger-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetFinger)))
  "Returns string type for a service object of type '<ResetFinger>"
  "owd_msgs/ResetFinger")