; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude StartMoveDirection-request.msg.html

(cl:defclass <StartMoveDirection-request> (roslisp-msg-protocol:ros-message)
  ((direction_x
    :reader direction_x
    :initarg :direction_x
    :type cl:float
    :initform 0.0)
   (direction_y
    :reader direction_y
    :initarg :direction_y
    :type cl:float
    :initform 0.0)
   (direction_z
    :reader direction_z
    :initarg :direction_z
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (compliance
    :reader compliance
    :initarg :compliance
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartMoveDirection-request (<StartMoveDirection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartMoveDirection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartMoveDirection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartMoveDirection-request> is deprecated: use owd_msgs-srv:StartMoveDirection-request instead.")))

(cl:ensure-generic-function 'direction_x-val :lambda-list '(m))
(cl:defmethod direction_x-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:direction_x-val is deprecated.  Use owd_msgs-srv:direction_x instead.")
  (direction_x m))

(cl:ensure-generic-function 'direction_y-val :lambda-list '(m))
(cl:defmethod direction_y-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:direction_y-val is deprecated.  Use owd_msgs-srv:direction_y instead.")
  (direction_y m))

(cl:ensure-generic-function 'direction_z-val :lambda-list '(m))
(cl:defmethod direction_z-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:direction_z-val is deprecated.  Use owd_msgs-srv:direction_z instead.")
  (direction_z m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:distance-val is deprecated.  Use owd_msgs-srv:distance instead.")
  (distance m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:velocity-val is deprecated.  Use owd_msgs-srv:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'compliance-val :lambda-list '(m))
(cl:defmethod compliance-val ((m <StartMoveDirection-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:compliance-val is deprecated.  Use owd_msgs-srv:compliance instead.")
  (compliance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartMoveDirection-request>) ostream)
  "Serializes a message object of type '<StartMoveDirection-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'direction_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'direction_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'direction_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'compliance) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartMoveDirection-request>) istream)
  "Deserializes a message object of type '<StartMoveDirection-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'compliance) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartMoveDirection-request>)))
  "Returns string type for a service object of type '<StartMoveDirection-request>"
  "owd_msgs/StartMoveDirectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMoveDirection-request)))
  "Returns string type for a service object of type 'StartMoveDirection-request"
  "owd_msgs/StartMoveDirectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartMoveDirection-request>)))
  "Returns md5sum for a message object of type '<StartMoveDirection-request>"
  "592d0394dec93f8d4a007e711a4711de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartMoveDirection-request)))
  "Returns md5sum for a message object of type 'StartMoveDirection-request"
  "592d0394dec93f8d4a007e711a4711de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartMoveDirection-request>)))
  "Returns full string definition for message of type '<StartMoveDirection-request>"
  (cl:format cl:nil "float32 direction_x~%float32 direction_y~%float32 direction_z~%float32 distance~%float32 velocity~%bool compliance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartMoveDirection-request)))
  "Returns full string definition for message of type 'StartMoveDirection-request"
  (cl:format cl:nil "float32 direction_x~%float32 direction_y~%float32 direction_z~%float32 distance~%float32 velocity~%bool compliance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartMoveDirection-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartMoveDirection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartMoveDirection-request
    (cl:cons ':direction_x (direction_x msg))
    (cl:cons ':direction_y (direction_y msg))
    (cl:cons ':direction_z (direction_z msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':compliance (compliance msg))
))
;//! \htmlinclude StartMoveDirection-response.msg.html

(cl:defclass <StartMoveDirection-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass StartMoveDirection-response (<StartMoveDirection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartMoveDirection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartMoveDirection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<StartMoveDirection-response> is deprecated: use owd_msgs-srv:StartMoveDirection-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <StartMoveDirection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <StartMoveDirection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <StartMoveDirection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartMoveDirection-response>) ostream)
  "Serializes a message object of type '<StartMoveDirection-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartMoveDirection-response>) istream)
  "Deserializes a message object of type '<StartMoveDirection-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartMoveDirection-response>)))
  "Returns string type for a service object of type '<StartMoveDirection-response>"
  "owd_msgs/StartMoveDirectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMoveDirection-response)))
  "Returns string type for a service object of type 'StartMoveDirection-response"
  "owd_msgs/StartMoveDirectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartMoveDirection-response>)))
  "Returns md5sum for a message object of type '<StartMoveDirection-response>"
  "592d0394dec93f8d4a007e711a4711de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartMoveDirection-response)))
  "Returns md5sum for a message object of type 'StartMoveDirection-response"
  "592d0394dec93f8d4a007e711a4711de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartMoveDirection-response>)))
  "Returns full string definition for message of type '<StartMoveDirection-response>"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartMoveDirection-response)))
  "Returns full string definition for message of type 'StartMoveDirection-response"
  (cl:format cl:nil "bool ok~%string reason~%uint32 id~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartMoveDirection-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartMoveDirection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartMoveDirection-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartMoveDirection)))
  'StartMoveDirection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartMoveDirection)))
  'StartMoveDirection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartMoveDirection)))
  "Returns string type for a service object of type '<StartMoveDirection>"
  "owd_msgs/StartMoveDirection")