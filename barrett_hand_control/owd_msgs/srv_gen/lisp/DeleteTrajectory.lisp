; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude DeleteTrajectory-request.msg.html

(cl:defclass <DeleteTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass DeleteTrajectory-request (<DeleteTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<DeleteTrajectory-request> is deprecated: use owd_msgs-srv:DeleteTrajectory-request instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <DeleteTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ids-val is deprecated.  Use owd_msgs-srv:ids instead.")
  (ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteTrajectory-request>) ostream)
  "Serializes a message object of type '<DeleteTrajectory-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteTrajectory-request>) istream)
  "Deserializes a message object of type '<DeleteTrajectory-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteTrajectory-request>)))
  "Returns string type for a service object of type '<DeleteTrajectory-request>"
  "owd_msgs/DeleteTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrajectory-request)))
  "Returns string type for a service object of type 'DeleteTrajectory-request"
  "owd_msgs/DeleteTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteTrajectory-request>)))
  "Returns md5sum for a message object of type '<DeleteTrajectory-request>"
  "ae5f0ecb1e1d1f21c66a723bc36404cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteTrajectory-request)))
  "Returns md5sum for a message object of type 'DeleteTrajectory-request"
  "ae5f0ecb1e1d1f21c66a723bc36404cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteTrajectory-request>)))
  "Returns full string definition for message of type '<DeleteTrajectory-request>"
  (cl:format cl:nil "string[] ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteTrajectory-request)))
  "Returns full string definition for message of type 'DeleteTrajectory-request"
  (cl:format cl:nil "string[] ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteTrajectory-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteTrajectory-request
    (cl:cons ':ids (ids msg))
))
;//! \htmlinclude DeleteTrajectory-response.msg.html

(cl:defclass <DeleteTrajectory-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass DeleteTrajectory-response (<DeleteTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeleteTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeleteTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<DeleteTrajectory-response> is deprecated: use owd_msgs-srv:DeleteTrajectory-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <DeleteTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <DeleteTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeleteTrajectory-response>) ostream)
  "Serializes a message object of type '<DeleteTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeleteTrajectory-response>) istream)
  "Deserializes a message object of type '<DeleteTrajectory-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeleteTrajectory-response>)))
  "Returns string type for a service object of type '<DeleteTrajectory-response>"
  "owd_msgs/DeleteTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrajectory-response)))
  "Returns string type for a service object of type 'DeleteTrajectory-response"
  "owd_msgs/DeleteTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeleteTrajectory-response>)))
  "Returns md5sum for a message object of type '<DeleteTrajectory-response>"
  "ae5f0ecb1e1d1f21c66a723bc36404cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeleteTrajectory-response)))
  "Returns md5sum for a message object of type 'DeleteTrajectory-response"
  "ae5f0ecb1e1d1f21c66a723bc36404cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeleteTrajectory-response>)))
  "Returns full string definition for message of type '<DeleteTrajectory-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeleteTrajectory-response)))
  "Returns full string definition for message of type 'DeleteTrajectory-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeleteTrajectory-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeleteTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeleteTrajectory-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeleteTrajectory)))
  'DeleteTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeleteTrajectory)))
  'DeleteTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeleteTrajectory)))
  "Returns string type for a service object of type '<DeleteTrajectory>"
  "owd_msgs/DeleteTrajectory")