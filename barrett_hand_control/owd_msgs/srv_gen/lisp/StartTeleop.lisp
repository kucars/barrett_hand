; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StartTeleop-request.msg.html

(cl:defclass <StartTeleop-request> (roslisp-msg-protocol:ros-message)
  ((input_topic
    :reader input_topic
    :initarg :input_topic
    :type cl:string
    :initform ""))
)

(cl:defclass StartTeleop-request (<StartTeleop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTeleop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTeleop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartTeleop-request> is deprecated: use owd_msgs-srv:StartTeleop-request instead.")))

(cl:ensure-generic-function 'input_topic-val :lambda-list '(m))
(cl:defmethod input_topic-val ((m <StartTeleop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:input_topic-val is deprecated.  Use owd_msgs-srv:input_topic instead.")
  (input_topic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTeleop-request>) ostream)
  "Serializes a message object of type '<StartTeleop-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'input_topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'input_topic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTeleop-request>) istream)
  "Deserializes a message object of type '<StartTeleop-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input_topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'input_topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTeleop-request>)))
  "Returns string type for a service object of type '<StartTeleop-request>"
  "owd_msgs/StartTeleopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeleop-request)))
  "Returns string type for a service object of type 'StartTeleop-request"
  "owd_msgs/StartTeleopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTeleop-request>)))
  "Returns md5sum for a message object of type '<StartTeleop-request>"
  "e3c64a01f46498f5454eb9dea3b978d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTeleop-request)))
  "Returns md5sum for a message object of type 'StartTeleop-request"
  "e3c64a01f46498f5454eb9dea3b978d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTeleop-request>)))
  "Returns full string definition for message of type '<StartTeleop-request>"
  (cl:format cl:nil "~%~%string input_topic~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTeleop-request)))
  "Returns full string definition for message of type 'StartTeleop-request"
  (cl:format cl:nil "~%~%string input_topic~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTeleop-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'input_topic))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTeleop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTeleop-request
    (cl:cons ':input_topic (input_topic msg))
))
;//! \htmlinclude StartTeleop-response.msg.html

(cl:defclass <StartTeleop-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass StartTeleop-response (<StartTeleop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartTeleop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartTeleop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartTeleop-response> is deprecated: use owd_msgs-srv:StartTeleop-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <StartTeleop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <StartTeleop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <StartTeleop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartTeleop-response>) ostream)
  "Serializes a message object of type '<StartTeleop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartTeleop-response>) istream)
  "Deserializes a message object of type '<StartTeleop-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartTeleop-response>)))
  "Returns string type for a service object of type '<StartTeleop-response>"
  "owd_msgs/StartTeleopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeleop-response)))
  "Returns string type for a service object of type 'StartTeleop-response"
  "owd_msgs/StartTeleopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartTeleop-response>)))
  "Returns md5sum for a message object of type '<StartTeleop-response>"
  "e3c64a01f46498f5454eb9dea3b978d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartTeleop-response)))
  "Returns md5sum for a message object of type 'StartTeleop-response"
  "e3c64a01f46498f5454eb9dea3b978d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartTeleop-response>)))
  "Returns full string definition for message of type '<StartTeleop-response>"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartTeleop-response)))
  "Returns full string definition for message of type 'StartTeleop-response"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartTeleop-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartTeleop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartTeleop-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartTeleop)))
  'StartTeleop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartTeleop)))
  'StartTeleop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartTeleop)))
  "Returns string type for a service object of type '<StartTeleop>"
  "owd_msgs/StartTeleop")