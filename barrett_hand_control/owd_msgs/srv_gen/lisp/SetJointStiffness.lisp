; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetJointStiffness-request.msg.html

(cl:defclass <SetJointStiffness-request> (roslisp-msg-protocol:ros-message)
  ((stiffness
    :reader stiffness
    :initarg :stiffness
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetJointStiffness-request (<SetJointStiffness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointStiffness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointStiffness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetJointStiffness-request> is deprecated: use owd_msgs-srv:SetJointStiffness-request instead.")))

(cl:ensure-generic-function 'stiffness-val :lambda-list '(m))
(cl:defmethod stiffness-val ((m <SetJointStiffness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:stiffness-val is deprecated.  Use owd_msgs-srv:stiffness instead.")
  (stiffness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointStiffness-request>) ostream)
  "Serializes a message object of type '<SetJointStiffness-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stiffness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'stiffness))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointStiffness-request>) istream)
  "Deserializes a message object of type '<SetJointStiffness-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stiffness) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stiffness)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointStiffness-request>)))
  "Returns string type for a service object of type '<SetJointStiffness-request>"
  "owd_msgs/SetJointStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointStiffness-request)))
  "Returns string type for a service object of type 'SetJointStiffness-request"
  "owd_msgs/SetJointStiffnessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointStiffness-request>)))
  "Returns md5sum for a message object of type '<SetJointStiffness-request>"
  "e192d1f69eccc586e7f2b6325b1ee3d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointStiffness-request)))
  "Returns md5sum for a message object of type 'SetJointStiffness-request"
  "e192d1f69eccc586e7f2b6325b1ee3d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointStiffness-request>)))
  "Returns full string definition for message of type '<SetJointStiffness-request>"
  (cl:format cl:nil "float32[] stiffness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointStiffness-request)))
  "Returns full string definition for message of type 'SetJointStiffness-request"
  (cl:format cl:nil "float32[] stiffness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointStiffness-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stiffness) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointStiffness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointStiffness-request
    (cl:cons ':stiffness (stiffness msg))
))
;//! \htmlinclude SetJointStiffness-response.msg.html

(cl:defclass <SetJointStiffness-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetJointStiffness-response (<SetJointStiffness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointStiffness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointStiffness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetJointStiffness-response> is deprecated: use owd_msgs-srv:SetJointStiffness-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetJointStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetJointStiffness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointStiffness-response>) ostream)
  "Serializes a message object of type '<SetJointStiffness-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointStiffness-response>) istream)
  "Deserializes a message object of type '<SetJointStiffness-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointStiffness-response>)))
  "Returns string type for a service object of type '<SetJointStiffness-response>"
  "owd_msgs/SetJointStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointStiffness-response)))
  "Returns string type for a service object of type 'SetJointStiffness-response"
  "owd_msgs/SetJointStiffnessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointStiffness-response>)))
  "Returns md5sum for a message object of type '<SetJointStiffness-response>"
  "e192d1f69eccc586e7f2b6325b1ee3d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointStiffness-response)))
  "Returns md5sum for a message object of type 'SetJointStiffness-response"
  "e192d1f69eccc586e7f2b6325b1ee3d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointStiffness-response>)))
  "Returns full string definition for message of type '<SetJointStiffness-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointStiffness-response)))
  "Returns full string definition for message of type 'SetJointStiffness-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointStiffness-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointStiffness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointStiffness-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointStiffness)))
  'SetJointStiffness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointStiffness)))
  'SetJointStiffness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointStiffness)))
  "Returns string type for a service object of type '<SetJointStiffness>"
  "owd_msgs/SetJointStiffness")