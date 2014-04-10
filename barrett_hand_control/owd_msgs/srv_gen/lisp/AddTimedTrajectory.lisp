; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude AddTimedTrajectory-request.msg.html

(cl:defclass <AddTimedTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((SerializedTrajectory
    :reader SerializedTrajectory
    :initarg :SerializedTrajectory
    :type cl:string
    :initform "")
   (options
    :reader options
    :initarg :options
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass AddTimedTrajectory-request (<AddTimedTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTimedTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTimedTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<AddTimedTrajectory-request> is deprecated: use owd_msgs-srv:AddTimedTrajectory-request instead.")))

(cl:ensure-generic-function 'SerializedTrajectory-val :lambda-list '(m))
(cl:defmethod SerializedTrajectory-val ((m <AddTimedTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:SerializedTrajectory-val is deprecated.  Use owd_msgs-srv:SerializedTrajectory instead.")
  (SerializedTrajectory m))

(cl:ensure-generic-function 'options-val :lambda-list '(m))
(cl:defmethod options-val ((m <AddTimedTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:options-val is deprecated.  Use owd_msgs-srv:options instead.")
  (options m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AddTimedTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTimedTrajectory-request>) ostream)
  "Serializes a message object of type '<AddTimedTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'SerializedTrajectory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'SerializedTrajectory))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'options)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'options)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTimedTrajectory-request>) istream)
  "Deserializes a message object of type '<AddTimedTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'SerializedTrajectory) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'SerializedTrajectory) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'options)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTimedTrajectory-request>)))
  "Returns string type for a service object of type '<AddTimedTrajectory-request>"
  "owd_msgs/AddTimedTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTimedTrajectory-request)))
  "Returns string type for a service object of type 'AddTimedTrajectory-request"
  "owd_msgs/AddTimedTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTimedTrajectory-request>)))
  "Returns md5sum for a message object of type '<AddTimedTrajectory-request>"
  "cef48710cbca1959e13f1a927d0adf9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTimedTrajectory-request)))
  "Returns md5sum for a message object of type 'AddTimedTrajectory-request"
  "cef48710cbca1959e13f1a927d0adf9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTimedTrajectory-request>)))
  "Returns full string definition for message of type '<AddTimedTrajectory-request>"
  (cl:format cl:nil "~%~%string SerializedTrajectory~%~%~%uint32 options~%~%~%~%string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTimedTrajectory-request)))
  "Returns full string definition for message of type 'AddTimedTrajectory-request"
  (cl:format cl:nil "~%~%string SerializedTrajectory~%~%~%uint32 options~%~%~%~%string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTimedTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'SerializedTrajectory))
     4
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTimedTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTimedTrajectory-request
    (cl:cons ':SerializedTrajectory (SerializedTrajectory msg))
    (cl:cons ':options (options msg))
    (cl:cons ':id (id msg))
))
;//! \htmlinclude AddTimedTrajectory-response.msg.html

(cl:defclass <AddTimedTrajectory-response> (roslisp-msg-protocol:ros-message)
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
    :type cl:string
    :initform "")
   (time_added
    :reader time_added
    :initarg :time_added
    :type cl:real
    :initform 0))
)

(cl:defclass AddTimedTrajectory-response (<AddTimedTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTimedTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTimedTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<AddTimedTrajectory-response> is deprecated: use owd_msgs-srv:AddTimedTrajectory-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <AddTimedTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <AddTimedTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AddTimedTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'time_added-val :lambda-list '(m))
(cl:defmethod time_added-val ((m <AddTimedTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:time_added-val is deprecated.  Use owd_msgs-srv:time_added instead.")
  (time_added m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTimedTrajectory-response>) ostream)
  "Serializes a message object of type '<AddTimedTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_added)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_added) (cl:floor (cl:slot-value msg 'time_added)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTimedTrajectory-response>) istream)
  "Deserializes a message object of type '<AddTimedTrajectory-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_added) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTimedTrajectory-response>)))
  "Returns string type for a service object of type '<AddTimedTrajectory-response>"
  "owd_msgs/AddTimedTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTimedTrajectory-response)))
  "Returns string type for a service object of type 'AddTimedTrajectory-response"
  "owd_msgs/AddTimedTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTimedTrajectory-response>)))
  "Returns md5sum for a message object of type '<AddTimedTrajectory-response>"
  "cef48710cbca1959e13f1a927d0adf9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTimedTrajectory-response)))
  "Returns md5sum for a message object of type 'AddTimedTrajectory-response"
  "cef48710cbca1959e13f1a927d0adf9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTimedTrajectory-response>)))
  "Returns full string definition for message of type '<AddTimedTrajectory-response>"
  (cl:format cl:nil "bool ok~%string reason~%string id~%time time_added~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTimedTrajectory-response)))
  "Returns full string definition for message of type 'AddTimedTrajectory-response"
  (cl:format cl:nil "bool ok~%string reason~%string id~%time time_added~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTimedTrajectory-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4 (cl:length (cl:slot-value msg 'id))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTimedTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTimedTrajectory-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
    (cl:cons ':time_added (time_added msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddTimedTrajectory)))
  'AddTimedTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddTimedTrajectory)))
  'AddTimedTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTimedTrajectory)))
  "Returns string type for a service object of type '<AddTimedTrajectory>"
  "owd_msgs/AddTimedTrajectory")