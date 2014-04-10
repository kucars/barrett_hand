; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetController-request.msg.html

(cl:defclass <SetController-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SetController-request (<SetController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetController-request> is deprecated: use owd_msgs-srv:SetController-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SetController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:name-val is deprecated.  Use owd_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetController-request>) ostream)
  "Serializes a message object of type '<SetController-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetController-request>) istream)
  "Deserializes a message object of type '<SetController-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetController-request>)))
  "Returns string type for a service object of type '<SetController-request>"
  "owd_msgs/SetControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetController-request)))
  "Returns string type for a service object of type 'SetController-request"
  "owd_msgs/SetControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetController-request>)))
  "Returns md5sum for a message object of type '<SetController-request>"
  "1e590c5944817d436cfbcaa99fe1b98a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetController-request)))
  "Returns md5sum for a message object of type 'SetController-request"
  "1e590c5944817d436cfbcaa99fe1b98a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetController-request>)))
  "Returns full string definition for message of type '<SetController-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetController-request)))
  "Returns full string definition for message of type 'SetController-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetController-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetController-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude SetController-response.msg.html

(cl:defclass <SetController-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetController-response (<SetController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetController-response> is deprecated: use owd_msgs-srv:SetController-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetController-response>) ostream)
  "Serializes a message object of type '<SetController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetController-response>) istream)
  "Deserializes a message object of type '<SetController-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetController-response>)))
  "Returns string type for a service object of type '<SetController-response>"
  "owd_msgs/SetControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetController-response)))
  "Returns string type for a service object of type 'SetController-response"
  "owd_msgs/SetControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetController-response>)))
  "Returns md5sum for a message object of type '<SetController-response>"
  "1e590c5944817d436cfbcaa99fe1b98a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetController-response)))
  "Returns md5sum for a message object of type 'SetController-response"
  "1e590c5944817d436cfbcaa99fe1b98a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetController-response>)))
  "Returns full string definition for message of type '<SetController-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetController-response)))
  "Returns full string definition for message of type 'SetController-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetController-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetController-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetController)))
  'SetController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetController)))
  'SetController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetController)))
  "Returns string type for a service object of type '<SetController>"
  "owd_msgs/SetController")