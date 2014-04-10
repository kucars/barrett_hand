; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude ResetHand-request.msg.html

(cl:defclass <ResetHand-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetHand-request (<ResetHand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetHand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetHand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ResetHand-request> is deprecated: use owd_msgs-srv:ResetHand-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetHand-request>) ostream)
  "Serializes a message object of type '<ResetHand-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetHand-request>) istream)
  "Deserializes a message object of type '<ResetHand-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetHand-request>)))
  "Returns string type for a service object of type '<ResetHand-request>"
  "owd_msgs/ResetHandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetHand-request)))
  "Returns string type for a service object of type 'ResetHand-request"
  "owd_msgs/ResetHandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetHand-request>)))
  "Returns md5sum for a message object of type '<ResetHand-request>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetHand-request)))
  "Returns md5sum for a message object of type 'ResetHand-request"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetHand-request>)))
  "Returns full string definition for message of type '<ResetHand-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetHand-request)))
  "Returns full string definition for message of type 'ResetHand-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetHand-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetHand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetHand-request
))
;//! \htmlinclude ResetHand-response.msg.html

(cl:defclass <ResetHand-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ResetHand-response (<ResetHand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetHand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetHand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<ResetHand-response> is deprecated: use owd_msgs-srv:ResetHand-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <ResetHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <ResetHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetHand-response>) ostream)
  "Serializes a message object of type '<ResetHand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetHand-response>) istream)
  "Deserializes a message object of type '<ResetHand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetHand-response>)))
  "Returns string type for a service object of type '<ResetHand-response>"
  "owd_msgs/ResetHandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetHand-response)))
  "Returns string type for a service object of type 'ResetHand-response"
  "owd_msgs/ResetHandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetHand-response>)))
  "Returns md5sum for a message object of type '<ResetHand-response>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetHand-response)))
  "Returns md5sum for a message object of type 'ResetHand-response"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetHand-response>)))
  "Returns full string definition for message of type '<ResetHand-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetHand-response)))
  "Returns full string definition for message of type 'ResetHand-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetHand-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetHand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetHand-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetHand)))
  'ResetHand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetHand)))
  'ResetHand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetHand)))
  "Returns string type for a service object of type '<ResetHand>"
  "owd_msgs/ResetHand")