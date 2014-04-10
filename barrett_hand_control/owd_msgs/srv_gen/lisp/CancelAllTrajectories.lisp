; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude CancelAllTrajectories-request.msg.html

(cl:defclass <CancelAllTrajectories-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CancelAllTrajectories-request (<CancelAllTrajectories-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CancelAllTrajectories-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CancelAllTrajectories-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<CancelAllTrajectories-request> is deprecated: use owd_msgs-srv:CancelAllTrajectories-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CancelAllTrajectories-request>) ostream)
  "Serializes a message object of type '<CancelAllTrajectories-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CancelAllTrajectories-request>) istream)
  "Deserializes a message object of type '<CancelAllTrajectories-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CancelAllTrajectories-request>)))
  "Returns string type for a service object of type '<CancelAllTrajectories-request>"
  "owd_msgs/CancelAllTrajectoriesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CancelAllTrajectories-request)))
  "Returns string type for a service object of type 'CancelAllTrajectories-request"
  "owd_msgs/CancelAllTrajectoriesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CancelAllTrajectories-request>)))
  "Returns md5sum for a message object of type '<CancelAllTrajectories-request>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CancelAllTrajectories-request)))
  "Returns md5sum for a message object of type 'CancelAllTrajectories-request"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CancelAllTrajectories-request>)))
  "Returns full string definition for message of type '<CancelAllTrajectories-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CancelAllTrajectories-request)))
  "Returns full string definition for message of type 'CancelAllTrajectories-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CancelAllTrajectories-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CancelAllTrajectories-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CancelAllTrajectories-request
))
;//! \htmlinclude CancelAllTrajectories-response.msg.html

(cl:defclass <CancelAllTrajectories-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass CancelAllTrajectories-response (<CancelAllTrajectories-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CancelAllTrajectories-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CancelAllTrajectories-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<CancelAllTrajectories-response> is deprecated: use owd_msgs-srv:CancelAllTrajectories-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <CancelAllTrajectories-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <CancelAllTrajectories-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CancelAllTrajectories-response>) ostream)
  "Serializes a message object of type '<CancelAllTrajectories-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CancelAllTrajectories-response>) istream)
  "Deserializes a message object of type '<CancelAllTrajectories-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CancelAllTrajectories-response>)))
  "Returns string type for a service object of type '<CancelAllTrajectories-response>"
  "owd_msgs/CancelAllTrajectoriesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CancelAllTrajectories-response)))
  "Returns string type for a service object of type 'CancelAllTrajectories-response"
  "owd_msgs/CancelAllTrajectoriesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CancelAllTrajectories-response>)))
  "Returns md5sum for a message object of type '<CancelAllTrajectories-response>"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CancelAllTrajectories-response)))
  "Returns md5sum for a message object of type 'CancelAllTrajectories-response"
  "4679398f882e7cbdea165980d3ec2888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CancelAllTrajectories-response>)))
  "Returns full string definition for message of type '<CancelAllTrajectories-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CancelAllTrajectories-response)))
  "Returns full string definition for message of type 'CancelAllTrajectories-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CancelAllTrajectories-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CancelAllTrajectories-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CancelAllTrajectories-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CancelAllTrajectories)))
  'CancelAllTrajectories-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CancelAllTrajectories)))
  'CancelAllTrajectories-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CancelAllTrajectories)))
  "Returns string type for a service object of type '<CancelAllTrajectories>"
  "owd_msgs/CancelAllTrajectories")