; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude GetStiffness-request.msg.html

(cl:defclass <GetStiffness-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetStiffness-request (<GetStiffness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStiffness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStiffness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetStiffness-request> is deprecated: use owd_msgs-srv:GetStiffness-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStiffness-request>) ostream)
  "Serializes a message object of type '<GetStiffness-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStiffness-request>) istream)
  "Deserializes a message object of type '<GetStiffness-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStiffness-request>)))
  "Returns string type for a service object of type '<GetStiffness-request>"
  "owd_msgs/GetStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStiffness-request)))
  "Returns string type for a service object of type 'GetStiffness-request"
  "owd_msgs/GetStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStiffness-request>)))
  "Returns md5sum for a message object of type '<GetStiffness-request>"
  "369c1f1022dd58bd3c86927727e1ddb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStiffness-request)))
  "Returns md5sum for a message object of type 'GetStiffness-request"
  "369c1f1022dd58bd3c86927727e1ddb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStiffness-request>)))
  "Returns full string definition for message of type '<GetStiffness-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStiffness-request)))
  "Returns full string definition for message of type 'GetStiffness-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStiffness-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStiffness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStiffness-request
))
;//! \htmlinclude GetStiffness-response.msg.html

(cl:defclass <GetStiffness-response> (roslisp-msg-protocol:ros-message)
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
   (stiffness
    :reader stiffness
    :initarg :stiffness
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetStiffness-response (<GetStiffness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStiffness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStiffness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<GetStiffness-response> is deprecated: use owd_msgs-srv:GetStiffness-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <GetStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <GetStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'stiffness-val :lambda-list '(m))
(cl:defmethod stiffness-val ((m <GetStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:stiffness-val is deprecated.  Use owd_msgs-srv:stiffness instead.")
  (stiffness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStiffness-response>) ostream)
  "Serializes a message object of type '<GetStiffness-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stiffness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStiffness-response>) istream)
  "Deserializes a message object of type '<GetStiffness-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stiffness) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStiffness-response>)))
  "Returns string type for a service object of type '<GetStiffness-response>"
  "owd_msgs/GetStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStiffness-response)))
  "Returns string type for a service object of type 'GetStiffness-response"
  "owd_msgs/GetStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStiffness-response>)))
  "Returns md5sum for a message object of type '<GetStiffness-response>"
  "369c1f1022dd58bd3c86927727e1ddb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStiffness-response)))
  "Returns md5sum for a message object of type 'GetStiffness-response"
  "369c1f1022dd58bd3c86927727e1ddb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStiffness-response>)))
  "Returns full string definition for message of type '<GetStiffness-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%float32 stiffness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStiffness-response)))
  "Returns full string definition for message of type 'GetStiffness-response"
  (cl:format cl:nil "bool ok~%string reason~%~%float32 stiffness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStiffness-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStiffness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStiffness-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':stiffness (stiffness msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetStiffness)))
  'GetStiffness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetStiffness)))
  'GetStiffness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStiffness)))
  "Returns string type for a service object of type '<GetStiffness>"
  "owd_msgs/GetStiffness")