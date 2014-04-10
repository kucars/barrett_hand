; Auto-generated. Do not edit!


(cl:in-package owd_msgs-srv)


;//! \htmlinclude OpenDoor-request.msg.html

(cl:defclass <OpenDoor-request> (roslisp-msg-protocol:ros-message)
  ((traj
    :reader traj
    :initarg :traj
    :type owd_msgs-msg:JointTraj
    :initform (cl:make-instance 'owd_msgs-msg:JointTraj))
   (ee_pose
    :reader ee_pose
    :initarg :ee_pose
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (pull_direction
    :reader pull_direction
    :initarg :pull_direction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass OpenDoor-request (<OpenDoor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OpenDoor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OpenDoor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<OpenDoor-request> is deprecated: use owd_msgs-srv:OpenDoor-request instead.")))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <OpenDoor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:traj-val is deprecated.  Use owd_msgs-srv:traj instead.")
  (traj m))

(cl:ensure-generic-function 'ee_pose-val :lambda-list '(m))
(cl:defmethod ee_pose-val ((m <OpenDoor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ee_pose-val is deprecated.  Use owd_msgs-srv:ee_pose instead.")
  (ee_pose m))

(cl:ensure-generic-function 'pull_direction-val :lambda-list '(m))
(cl:defmethod pull_direction-val ((m <OpenDoor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:pull_direction-val is deprecated.  Use owd_msgs-srv:pull_direction instead.")
  (pull_direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OpenDoor-request>) ostream)
  "Serializes a message object of type '<OpenDoor-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ee_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ee_pose))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pull_direction) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OpenDoor-request>) istream)
  "Deserializes a message object of type '<OpenDoor-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ee_pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ee_pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pull_direction) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OpenDoor-request>)))
  "Returns string type for a service object of type '<OpenDoor-request>"
  "owd_msgs/OpenDoorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpenDoor-request)))
  "Returns string type for a service object of type 'OpenDoor-request"
  "owd_msgs/OpenDoorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OpenDoor-request>)))
  "Returns md5sum for a message object of type '<OpenDoor-request>"
  "f4a7c407b022295db65e5d7386039232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OpenDoor-request)))
  "Returns md5sum for a message object of type 'OpenDoor-request"
  "f4a7c407b022295db65e5d7386039232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OpenDoor-request>)))
  "Returns full string definition for message of type '<OpenDoor-request>"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%geometry_msgs/Pose[] ee_pose~%geometry_msgs/Vector3 pull_direction~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OpenDoor-request)))
  "Returns full string definition for message of type 'OpenDoor-request"
  (cl:format cl:nil "owd_msgs/JointTraj traj~%geometry_msgs/Pose[] ee_pose~%geometry_msgs/Vector3 pull_direction~%~%================================================================================~%MSG: owd_msgs/JointTraj~%owd_msgs/Joints[] positions~%float32[] blend_radius~%uint32 options~%string id~%~%# options should be powers of 2, so that they can be OR'd together~%uint32 opt_WaitForStart=1~%uint32 opt_CancelOnStall=2~%uint32 opt_CancelOnForceInput=4~%uint32 opt_CancelOnTactileInput=8~%uint32 opt_Synchronize=16~%#uint32 opt_          =32  # placeholder for next value~%~%================================================================================~%MSG: owd_msgs/Joints~%float64[] j~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OpenDoor-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ee_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pull_direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OpenDoor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OpenDoor-request
    (cl:cons ':traj (traj msg))
    (cl:cons ':ee_pose (ee_pose msg))
    (cl:cons ':pull_direction (pull_direction msg))
))
;//! \htmlinclude OpenDoor-response.msg.html

(cl:defclass <OpenDoor-response> (roslisp-msg-protocol:ros-message)
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
    :initform ""))
)

(cl:defclass OpenDoor-response (<OpenDoor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OpenDoor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OpenDoor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name owd_msgs-srv:<OpenDoor-response> is deprecated: use owd_msgs-srv:OpenDoor-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <OpenDoor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:ok-val is deprecated.  Use owd_msgs-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <OpenDoor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:reason-val is deprecated.  Use owd_msgs-srv:reason instead.")
  (reason m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <OpenDoor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader owd_msgs-srv:id-val is deprecated.  Use owd_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OpenDoor-response>) ostream)
  "Serializes a message object of type '<OpenDoor-response>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OpenDoor-response>) istream)
  "Deserializes a message object of type '<OpenDoor-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OpenDoor-response>)))
  "Returns string type for a service object of type '<OpenDoor-response>"
  "owd_msgs/OpenDoorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpenDoor-response)))
  "Returns string type for a service object of type 'OpenDoor-response"
  "owd_msgs/OpenDoorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OpenDoor-response>)))
  "Returns md5sum for a message object of type '<OpenDoor-response>"
  "f4a7c407b022295db65e5d7386039232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OpenDoor-response)))
  "Returns md5sum for a message object of type 'OpenDoor-response"
  "f4a7c407b022295db65e5d7386039232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OpenDoor-response>)))
  "Returns full string definition for message of type '<OpenDoor-response>"
  (cl:format cl:nil "bool ok~%string reason~%string id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OpenDoor-response)))
  "Returns full string definition for message of type 'OpenDoor-response"
  (cl:format cl:nil "bool ok~%string reason~%string id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OpenDoor-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'reason))
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OpenDoor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OpenDoor-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':reason (reason msg))
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OpenDoor)))
  'OpenDoor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OpenDoor)))
  'OpenDoor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpenDoor)))
  "Returns string type for a service object of type '<OpenDoor>"
  "owd_msgs/OpenDoor")