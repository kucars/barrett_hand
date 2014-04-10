; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StartHelixTraj-request.msg.html

(cl:defclass <StartHelixTraj-request> (roslisp-msg-protocol:ros-message)
  ((amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0))
)

(cl:defclass StartHelixTraj-request (<StartHelixTraj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartHelixTraj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartHelixTraj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartHelixTraj-request> is deprecated: use owd_msgs-srv:StartHelixTraj-request instead.")))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <StartHelixTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:amplitude-val is deprecated.  Use owd_msgs-srv:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <StartHelixTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:pitch-val is deprecated.  Use owd_msgs-srv:pitch instead.")
  (pitch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartHelixTraj-request>) ostream)
  "Serializes a message object of type '<StartHelixTraj-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartHelixTraj-request>) istream)
  "Deserializes a message object of type '<StartHelixTraj-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartHelixTraj-request>)))
  "Returns string type for a service object of type '<StartHelixTraj-request>"
  "owd_msgs/StartHelixTrajRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartHelixTraj-request)))
  "Returns string type for a service object of type 'StartHelixTraj-request"
  "owd_msgs/StartHelixTrajRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartHelixTraj-request>)))
  "Returns md5sum for a message object of type '<StartHelixTraj-request>"
  "9ea831417e899e1b9c045fa6a6a89bd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartHelixTraj-request)))
  "Returns md5sum for a message object of type 'StartHelixTraj-request"
  "9ea831417e899e1b9c045fa6a6a89bd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartHelixTraj-request>)))
  "Returns full string definition for message of type '<StartHelixTraj-request>"
  (cl:format cl:nil "float32 amplitude~%float32 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartHelixTraj-request)))
  "Returns full string definition for message of type 'StartHelixTraj-request"
  (cl:format cl:nil "float32 amplitude~%float32 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartHelixTraj-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartHelixTraj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartHelixTraj-request
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':pitch (pitch msg))
))
;//! \htmlinclude StartHelixTraj-response.msg.html

(cl:defclass <StartHelixTraj-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass StartHelixTraj-response (<StartHelixTraj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartHelixTraj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartHelixTraj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartHelixTraj-response> is deprecated: use owd_msgs-srv:StartHelixTraj-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <StartHelixTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <StartHelixTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <StartHelixTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartHelixTraj-response>) ostream)
  "Serializes a message object of type '<StartHelixTraj-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartHelixTraj-response>) istream)
  "Deserializes a message object of type '<StartHelixTraj-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartHelixTraj-response>)))
  "Returns string type for a service object of type '<StartHelixTraj-response>"
  "owd_msgs/StartHelixTrajResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartHelixTraj-response)))
  "Returns string type for a service object of type 'StartHelixTraj-response"
  "owd_msgs/StartHelixTrajResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartHelixTraj-response>)))
  "Returns md5sum for a message object of type '<StartHelixTraj-response>"
  "9ea831417e899e1b9c045fa6a6a89bd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartHelixTraj-response)))
  "Returns md5sum for a message object of type 'StartHelixTraj-response"
  "9ea831417e899e1b9c045fa6a6a89bd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartHelixTraj-response>)))
  "Returns full string definition for message of type '<StartHelixTraj-response>"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartHelixTraj-response)))
  "Returns full string definition for message of type 'StartHelixTraj-response"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartHelixTraj-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartHelixTraj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartHelixTraj-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartHelixTraj)))
  'StartHelixTraj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartHelixTraj)))
  'StartHelixTraj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartHelixTraj)))
  "Returns string type for a service object of type '<StartHelixTraj>"
  "owd_msgs/StartHelixTraj")