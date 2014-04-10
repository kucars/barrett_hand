; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude SetFingerCompliant-request.msg.html

(cl:defclass <SetFingerCompliant-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil)
   (strain
    :reader strain
    :initarg :strain
    :type cl:integer
    :initform 0))
)

(cl:defclass SetFingerCompliant-request (<SetFingerCompliant-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFingerCompliant-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFingerCompliant-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetFingerCompliant-request> is deprecated: use owd_msgs-srv:SetFingerCompliant-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SetFingerCompliant-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:enable-val is deprecated.  Use owd_msgs-srv:enable instead.")
  (enable m))

(cl:ensure-generic-function 'strain-val :lambda-list '(m))
(cl:defmethod strain-val ((m <SetFingerCompliant-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:strain-val is deprecated.  Use owd_msgs-srv:strain instead.")
  (strain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFingerCompliant-request>) ostream)
  "Serializes a message object of type '<SetFingerCompliant-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'strain)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFingerCompliant-request>) istream)
  "Deserializes a message object of type '<SetFingerCompliant-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strain) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFingerCompliant-request>)))
  "Returns string type for a service object of type '<SetFingerCompliant-request>"
  "owd_msgs/SetFingerCompliantRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerCompliant-request)))
  "Returns string type for a service object of type 'SetFingerCompliant-request"
  "owd_msgs/SetFingerCompliantRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFingerCompliant-request>)))
  "Returns md5sum for a message object of type '<SetFingerCompliant-request>"
  "5dcef267ed8d1dc073136b82b069876f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFingerCompliant-request)))
  "Returns md5sum for a message object of type 'SetFingerCompliant-request"
  "5dcef267ed8d1dc073136b82b069876f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFingerCompliant-request>)))
  "Returns full string definition for message of type '<SetFingerCompliant-request>"
  (cl:format cl:nil "bool  enable~%int32 strain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFingerCompliant-request)))
  "Returns full string definition for message of type 'SetFingerCompliant-request"
  (cl:format cl:nil "bool  enable~%int32 strain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFingerCompliant-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFingerCompliant-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFingerCompliant-request
    (cl:cons ':enable (enable msg))
    (cl:cons ':strain (strain msg))
))
;//! \htmlinclude SetFingerCompliant-response.msg.html

(cl:defclass <SetFingerCompliant-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetFingerCompliant-response (<SetFingerCompliant-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFingerCompliant-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFingerCompliant-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<SetFingerCompliant-response> is deprecated: use owd_msgs-srv:SetFingerCompliant-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetFingerCompliant-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SetFingerCompliant-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFingerCompliant-response>) ostream)
  "Serializes a message object of type '<SetFingerCompliant-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFingerCompliant-response>) istream)
  "Deserializes a message object of type '<SetFingerCompliant-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFingerCompliant-response>)))
  "Returns string type for a service object of type '<SetFingerCompliant-response>"
  "owd_msgs/SetFingerCompliantResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerCompliant-response)))
  "Returns string type for a service object of type 'SetFingerCompliant-response"
  "owd_msgs/SetFingerCompliantResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFingerCompliant-response>)))
  "Returns md5sum for a message object of type '<SetFingerCompliant-response>"
  "5dcef267ed8d1dc073136b82b069876f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFingerCompliant-response)))
  "Returns md5sum for a message object of type 'SetFingerCompliant-response"
  "5dcef267ed8d1dc073136b82b069876f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFingerCompliant-response>)))
  "Returns full string definition for message of type '<SetFingerCompliant-response>"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFingerCompliant-response)))
  "Returns full string definition for message of type 'SetFingerCompliant-response"
  (cl:format cl:nil "bool ok~%string reason~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFingerCompliant-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFingerCompliant-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFingerCompliant-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFingerCompliant)))
  'SetFingerCompliant-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFingerCompliant)))
  'SetFingerCompliant-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerCompliant)))
  "Returns string type for a service object of type '<SetFingerCompliant>"
  "owd_msgs/SetFingerCompliant")