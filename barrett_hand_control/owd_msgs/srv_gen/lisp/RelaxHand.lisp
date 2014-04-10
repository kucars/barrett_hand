; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude RelaxHand-request.msg.html

(cl:defclass <RelaxHand-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RelaxHand-request (<RelaxHand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelaxHand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelaxHand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<RelaxHand-request> is deprecated: use owd_msgs-srv:RelaxHand-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelaxHand-request>) ostream)
  "Serializes a message object of type '<RelaxHand-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelaxHand-request>) istream)
  "Deserializes a message object of type '<RelaxHand-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelaxHand-request>)))
  "Returns string type for a service object of type '<RelaxHand-request>"
  "owd_msgs/RelaxHandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelaxHand-request)))
  "Returns string type for a service object of type 'RelaxHand-request"
  "owd_msgs/RelaxHandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelaxHand-request>)))
  "Returns md5sum for a message object of type '<RelaxHand-request>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelaxHand-request)))
  "Returns md5sum for a message object of type 'RelaxHand-request"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelaxHand-request>)))
  "Returns full string definition for message of type '<RelaxHand-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelaxHand-request)))
  "Returns full string definition for message of type 'RelaxHand-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelaxHand-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelaxHand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RelaxHand-request
))
;//! \htmlinclude RelaxHand-response.msg.html

(cl:defclass <RelaxHand-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass RelaxHand-response (<RelaxHand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelaxHand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelaxHand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<RelaxHand-response> is deprecated: use owd_msgs-srv:RelaxHand-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <RelaxHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <RelaxHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelaxHand-response>) ostream)
  "Serializes a message object of type '<RelaxHand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelaxHand-response>) istream)
  "Deserializes a message object of type '<RelaxHand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelaxHand-response>)))
  "Returns string type for a service object of type '<RelaxHand-response>"
  "owd_msgs/RelaxHandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelaxHand-response)))
  "Returns string type for a service object of type 'RelaxHand-response"
  "owd_msgs/RelaxHandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelaxHand-response>)))
  "Returns md5sum for a message object of type '<RelaxHand-response>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelaxHand-response)))
  "Returns md5sum for a message object of type 'RelaxHand-response"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelaxHand-response>)))
  "Returns full string definition for message of type '<RelaxHand-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelaxHand-response)))
  "Returns full string definition for message of type 'RelaxHand-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelaxHand-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelaxHand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RelaxHand-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RelaxHand)))
  'RelaxHand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RelaxHand)))
  'RelaxHand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelaxHand)))
  "Returns string type for a service object of type '<RelaxHand>"
  "owd_msgs/RelaxHand")