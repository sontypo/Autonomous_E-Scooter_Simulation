; Auto-generated. Do not edit!


(cl:in-package escooter_msgs-msg)


;//! \htmlinclude Joints.msg.html

(cl:defclass <Joints> (roslisp-msg-protocol:ros-message)
  ((handle_bar
    :reader handle_bar
    :initarg :handle_bar
    :type cl:float
    :initform 0.0)
   (rotational_plate
    :reader rotational_plate
    :initarg :rotational_plate
    :type cl:float
    :initform 0.0)
   (front_wheel
    :reader front_wheel
    :initarg :front_wheel
    :type cl:float
    :initform 0.0))
)

(cl:defclass Joints (<Joints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Joints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Joints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name escooter_msgs-msg:<Joints> is deprecated: use escooter_msgs-msg:Joints instead.")))

(cl:ensure-generic-function 'handle_bar-val :lambda-list '(m))
(cl:defmethod handle_bar-val ((m <Joints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader escooter_msgs-msg:handle_bar-val is deprecated.  Use escooter_msgs-msg:handle_bar instead.")
  (handle_bar m))

(cl:ensure-generic-function 'rotational_plate-val :lambda-list '(m))
(cl:defmethod rotational_plate-val ((m <Joints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader escooter_msgs-msg:rotational_plate-val is deprecated.  Use escooter_msgs-msg:rotational_plate instead.")
  (rotational_plate m))

(cl:ensure-generic-function 'front_wheel-val :lambda-list '(m))
(cl:defmethod front_wheel-val ((m <Joints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader escooter_msgs-msg:front_wheel-val is deprecated.  Use escooter_msgs-msg:front_wheel instead.")
  (front_wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Joints>) ostream)
  "Serializes a message object of type '<Joints>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'handle_bar))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotational_plate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'front_wheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Joints>) istream)
  "Deserializes a message object of type '<Joints>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'handle_bar) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotational_plate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_wheel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Joints>)))
  "Returns string type for a message object of type '<Joints>"
  "escooter_msgs/Joints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Joints)))
  "Returns string type for a message object of type 'Joints"
  "escooter_msgs/Joints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Joints>)))
  "Returns md5sum for a message object of type '<Joints>"
  "d3e5fbcf0da70ecde669dea5a74f6f7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Joints)))
  "Returns md5sum for a message object of type 'Joints"
  "d3e5fbcf0da70ecde669dea5a74f6f7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Joints>)))
  "Returns full string definition for message of type '<Joints>"
  (cl:format cl:nil "float64 handle_bar~%float64 rotational_plate~%float64 front_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Joints)))
  "Returns full string definition for message of type 'Joints"
  (cl:format cl:nil "float64 handle_bar~%float64 rotational_plate~%float64 front_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Joints>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Joints>))
  "Converts a ROS message object to a list"
  (cl:list 'Joints
    (cl:cons ':handle_bar (handle_bar msg))
    (cl:cons ':rotational_plate (rotational_plate msg))
    (cl:cons ':front_wheel (front_wheel msg))
))
