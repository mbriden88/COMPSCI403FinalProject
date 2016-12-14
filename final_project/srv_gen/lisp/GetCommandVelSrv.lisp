; Auto-generated. Do not edit!


(cl:in-package final_project-srv)


;//! \htmlinclude GetCommandVelSrv-request.msg.html

(cl:defclass <GetCommandVelSrv-request> (roslisp-msg-protocol:ros-message)
  ((Image
    :reader Image
    :initarg :Image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (v0
    :reader v0
    :initarg :v0
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (w0
    :reader w0
    :initarg :w0
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass GetCommandVelSrv-request (<GetCommandVelSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandVelSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandVelSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name final_project-srv:<GetCommandVelSrv-request> is deprecated: use final_project-srv:GetCommandVelSrv-request instead.")))

(cl:ensure-generic-function 'Image-val :lambda-list '(m))
(cl:defmethod Image-val ((m <GetCommandVelSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader final_project-srv:Image-val is deprecated.  Use final_project-srv:Image instead.")
  (Image m))

(cl:ensure-generic-function 'v0-val :lambda-list '(m))
(cl:defmethod v0-val ((m <GetCommandVelSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader final_project-srv:v0-val is deprecated.  Use final_project-srv:v0 instead.")
  (v0 m))

(cl:ensure-generic-function 'w0-val :lambda-list '(m))
(cl:defmethod w0-val ((m <GetCommandVelSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader final_project-srv:w0-val is deprecated.  Use final_project-srv:w0 instead.")
  (w0 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandVelSrv-request>) ostream)
  "Serializes a message object of type '<GetCommandVelSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'w0) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandVelSrv-request>) istream)
  "Deserializes a message object of type '<GetCommandVelSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'w0) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandVelSrv-request>)))
  "Returns string type for a service object of type '<GetCommandVelSrv-request>"
  "final_project/GetCommandVelSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv-request)))
  "Returns string type for a service object of type 'GetCommandVelSrv-request"
  "final_project/GetCommandVelSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandVelSrv-request>)))
  "Returns md5sum for a message object of type '<GetCommandVelSrv-request>"
  "f3ec071038723cac47826a5f79b9a4b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandVelSrv-request)))
  "Returns md5sum for a message object of type 'GetCommandVelSrv-request"
  "f3ec071038723cac47826a5f79b9a4b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandVelSrv-request>)))
  "Returns full string definition for message of type '<GetCommandVelSrv-request>"
  (cl:format cl:nil "sensor_msgs/Image Image~%geometry_msgs/Point32 v0~%geometry_msgs/Point32 w0~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandVelSrv-request)))
  "Returns full string definition for message of type 'GetCommandVelSrv-request"
  (cl:format cl:nil "sensor_msgs/Image Image~%geometry_msgs/Point32 v0~%geometry_msgs/Point32 w0~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandVelSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'w0))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandVelSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandVelSrv-request
    (cl:cons ':Image (Image msg))
    (cl:cons ':v0 (v0 msg))
    (cl:cons ':w0 (w0 msg))
))
;//! \htmlinclude GetCommandVelSrv-response.msg.html

(cl:defclass <GetCommandVelSrv-response> (roslisp-msg-protocol:ros-message)
  ((Cv
    :reader Cv
    :initarg :Cv
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (Cw
    :reader Cw
    :initarg :Cw
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass GetCommandVelSrv-response (<GetCommandVelSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCommandVelSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCommandVelSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name final_project-srv:<GetCommandVelSrv-response> is deprecated: use final_project-srv:GetCommandVelSrv-response instead.")))

(cl:ensure-generic-function 'Cv-val :lambda-list '(m))
(cl:defmethod Cv-val ((m <GetCommandVelSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader final_project-srv:Cv-val is deprecated.  Use final_project-srv:Cv instead.")
  (Cv m))

(cl:ensure-generic-function 'Cw-val :lambda-list '(m))
(cl:defmethod Cw-val ((m <GetCommandVelSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader final_project-srv:Cw-val is deprecated.  Use final_project-srv:Cw instead.")
  (Cw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCommandVelSrv-response>) ostream)
  "Serializes a message object of type '<GetCommandVelSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Cv) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Cw) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCommandVelSrv-response>) istream)
  "Deserializes a message object of type '<GetCommandVelSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Cv) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Cw) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCommandVelSrv-response>)))
  "Returns string type for a service object of type '<GetCommandVelSrv-response>"
  "final_project/GetCommandVelSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv-response)))
  "Returns string type for a service object of type 'GetCommandVelSrv-response"
  "final_project/GetCommandVelSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCommandVelSrv-response>)))
  "Returns md5sum for a message object of type '<GetCommandVelSrv-response>"
  "f3ec071038723cac47826a5f79b9a4b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCommandVelSrv-response)))
  "Returns md5sum for a message object of type 'GetCommandVelSrv-response"
  "f3ec071038723cac47826a5f79b9a4b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCommandVelSrv-response>)))
  "Returns full string definition for message of type '<GetCommandVelSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32 Cv~%geometry_msgs/Point32 Cw~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCommandVelSrv-response)))
  "Returns full string definition for message of type 'GetCommandVelSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32 Cv~%geometry_msgs/Point32 Cw~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCommandVelSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Cv))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Cw))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCommandVelSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCommandVelSrv-response
    (cl:cons ':Cv (Cv msg))
    (cl:cons ':Cw (Cw msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCommandVelSrv)))
  'GetCommandVelSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCommandVelSrv)))
  'GetCommandVelSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCommandVelSrv)))
  "Returns string type for a service object of type '<GetCommandVelSrv>"
  "final_project/GetCommandVelSrv")