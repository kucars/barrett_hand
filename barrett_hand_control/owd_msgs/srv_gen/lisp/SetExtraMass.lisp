; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetExtraMass-request.msg.html

(cl:defclass <SetExtraMass-request> (roslisp-msg-protocol:ros-message)
  ((m
    :reader m
    :initarg :m
    :type owd_msgs-msg:MassProperties
    :initform (cl:make-instance 'owd_msgs-msg:MassProperties)))
)

(cl:defclass SetExtraMass-request (<SetExtraMass-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExtraMass-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExtraMass-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetExtraMass-request> is deprecated: use owd_msgs-srv:SetExtraMass-request instead.")))

(cl:ensure-generic-function 'm-val :lambda-list '(m))
(cl:defmethod m-val ((m <SetExtraMass-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:m-val is deprecated.  Use owd_msgs-srv:m instead.")
  (m m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExtraMass-request>) ostream)
  "Serializes a message object of type '<SetExtraMass-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExtraMass-request>) istream)
  "Deserializes a message object of type '<SetExtraMass-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExtraMass-request>)))
  "Returns string type for a service object of type '<SetExtraMass-request>"
  "owd_msgs/SetExtraMassRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtraMass-request)))
  "Returns string type for a service object of type 'SetExtraMass-request"
  "owd_msgs/SetExtraMassRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExtraMass-request>)))
  "Returns md5sum for a message object of type '<SetExtraMass-request>"
  "6e56e849d39ef37144de532715dc61f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExtraMass-request)))
  "Returns md5sum for a message object of type 'SetExtraMass-request"
  "6e56e849d39ef37144de532715dc61f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExtraMass-request>)))
  "Returns full string definition for message of type '<SetExtraMass-request>"
  (cl:format cl:nil "owd_msgs/MassProperties m~%~%================================================================================~%MSG: owd_msgs/MassProperties~%uint8 link~%float64 mass~%float64 cog_x~%float64 cog_y~%float64 cog_z~%float64 inertia_xx~%float64 inertia_xy~%float64 inertia_xz~%float64 inertia_yy~%float64 inertia_yz~%float64 inertia_zz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExtraMass-request)))
  "Returns full string definition for message of type 'SetExtraMass-request"
  (cl:format cl:nil "owd_msgs/MassProperties m~%~%================================================================================~%MSG: owd_msgs/MassProperties~%uint8 link~%float64 mass~%float64 cog_x~%float64 cog_y~%float64 cog_z~%float64 inertia_xx~%float64 inertia_xy~%float64 inertia_xz~%float64 inertia_yy~%float64 inertia_yz~%float64 inertia_zz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExtraMass-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExtraMass-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExtraMass-request
    (cl:cons ':m (m msg))
))
;//! \htmlinclude SetExtraMass-response.msg.html

(cl:defclass <SetExtraMass-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetExtraMass-response (<SetExtraMass-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExtraMass-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExtraMass-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetExtraMass-response> is deprecated: use owd_msgs-srv:SetExtraMass-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetExtraMass-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetExtraMass-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExtraMass-response>) ostream)
  "Serializes a message object of type '<SetExtraMass-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExtraMass-response>) istream)
  "Deserializes a message object of type '<SetExtraMass-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExtraMass-response>)))
  "Returns string type for a service object of type '<SetExtraMass-response>"
  "owd_msgs/SetExtraMassResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtraMass-response)))
  "Returns string type for a service object of type 'SetExtraMass-response"
  "owd_msgs/SetExtraMassResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExtraMass-response>)))
  "Returns md5sum for a message object of type '<SetExtraMass-response>"
  "6e56e849d39ef37144de532715dc61f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExtraMass-response)))
  "Returns md5sum for a message object of type 'SetExtraMass-response"
  "6e56e849d39ef37144de532715dc61f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExtraMass-response>)))
  "Returns full string definition for message of type '<SetExtraMass-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExtraMass-response)))
  "Returns full string definition for message of type 'SetExtraMass-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExtraMass-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExtraMass-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExtraMass-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetExtraMass)))
  'SetExtraMass-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetExtraMass)))
  'SetExtraMass-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExtraMass)))
  "Returns string type for a service object of type '<SetExtraMass>"
  "owd_msgs/SetExtraMass")