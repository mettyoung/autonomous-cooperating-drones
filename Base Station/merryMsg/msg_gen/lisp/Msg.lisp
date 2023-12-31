; Auto-generated. Do not edit!


(cl:in-package merryMsg-msg)


;//! \htmlinclude Msg.msg.html

(cl:defclass <Msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass Msg (<Msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name merryMsg-msg:<Msg> is deprecated: use merryMsg-msg:Msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader merryMsg-msg:header-val is deprecated.  Use merryMsg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader merryMsg-msg:data-val is deprecated.  Use merryMsg-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Msg>) ostream)
  "Serializes a message object of type '<Msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Msg>) istream)
  "Deserializes a message object of type '<Msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Msg>)))
  "Returns string type for a message object of type '<Msg>"
  "merryMsg/Msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Msg)))
  "Returns string type for a message object of type 'Msg"
  "merryMsg/Msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Msg>)))
  "Returns md5sum for a message object of type '<Msg>"
  "ef848af8cf12f6df11682cc76fba477b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Msg)))
  "Returns md5sum for a message object of type 'Msg"
  "ef848af8cf12f6df11682cc76fba477b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Msg>)))
  "Returns full string definition for message of type '<Msg>"
  (cl:format cl:nil "Header header~%float32 data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Msg)))
  "Returns full string definition for message of type 'Msg"
  (cl:format cl:nil "Header header~%float32 data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Msg
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
