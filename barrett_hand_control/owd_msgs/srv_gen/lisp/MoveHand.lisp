; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude MoveHand-request.msg.html

(cl:defclass <MoveHand-request> (roslisp-msg-protocol:ros-message)
  ((movetype
    :reader movetype
    :initarg :movetype
    :type cl:fixnum
    :initform 0)
   (positions
    :reader positions
    :initarg :positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MoveHand-request (<MoveHand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveHand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveHand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<MoveHand-request> is deprecated: use owd_msgs-srv:MoveHand-request instead.")))

(cl:ensure-generic-function 'movetype-val :lambda-list '(m))
(cl:defmethod movetype-val ((m <MoveHand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:movetype-val is deprecated.  Use owd_msgs-srv:movetype instead.")
  (movetype m))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <MoveHand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:positions-val is deprecated.  Use owd_msgs-srv:positions instead.")
  (positions m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MoveHand-request>)))
    "Constants for message type '<MoveHand-request>"
  '((:MOVETYPE_POSITION . 1)
    (:MOVETYPE_VELOCITY . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MoveHand-request)))
    "Constants for message type 'MoveHand-request"
  '((:MOVETYPE_POSITION . 1)
    (:MOVETYPE_VELOCITY . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveHand-request>) ostream)
  "Serializes a message object of type '<MoveHand-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'movetype)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'positions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveHand-request>) istream)
  "Deserializes a message object of type '<MoveHand-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'movetype)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveHand-request>)))
  "Returns string type for a service object of type '<MoveHand-request>"
  "owd_msgs/MoveHandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHand-request)))
  "Returns string type for a service object of type 'MoveHand-request"
  "owd_msgs/MoveHandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveHand-request>)))
  "Returns md5sum for a message object of type '<MoveHand-request>"
  "f6025280010064fd2f0d4ea3199d410b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveHand-request)))
  "Returns md5sum for a message object of type 'MoveHand-request"
  "f6025280010064fd2f0d4ea3199d410b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveHand-request>)))
  "Returns full string definition for message of type '<MoveHand-request>"
  (cl:format cl:nil "uint8 movetype~%float64[] positions~%uint8 movetype_position=1~%uint8 movetype_velocity=2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveHand-request)))
  "Returns full string definition for message of type 'MoveHand-request"
  (cl:format cl:nil "uint8 movetype~%float64[] positions~%uint8 movetype_position=1~%uint8 movetype_velocity=2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveHand-request>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveHand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveHand-request
    (cl:cons ':movetype (movetype msg))
    (cl:cons ':positions (positions msg))
))
;//! \htmlinclude MoveHand-response.msg.html

(cl:defclass <MoveHand-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass MoveHand-response (<MoveHand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveHand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveHand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<MoveHand-response> is deprecated: use owd_msgs-srv:MoveHand-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <MoveHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <MoveHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveHand-response>) ostream)
  "Serializes a message object of type '<MoveHand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveHand-response>) istream)
  "Deserializes a message object of type '<MoveHand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveHand-response>)))
  "Returns string type for a service object of type '<MoveHand-response>"
  "owd_msgs/MoveHandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHand-response)))
  "Returns string type for a service object of type 'MoveHand-response"
  "owd_msgs/MoveHandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveHand-response>)))
  "Returns md5sum for a message object of type '<MoveHand-response>"
  "f6025280010064fd2f0d4ea3199d410b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveHand-response)))
  "Returns md5sum for a message object of type 'MoveHand-response"
  "f6025280010064fd2f0d4ea3199d410b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveHand-response>)))
  "Returns full string definition for message of type '<MoveHand-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveHand-response)))
  "Returns full string definition for message of type 'MoveHand-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveHand-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveHand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveHand-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveHand)))
  'MoveHand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveHand)))
  'MoveHand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHand)))
  "Returns string type for a service object of type '<MoveHand>"
  "owd_msgs/MoveHand")