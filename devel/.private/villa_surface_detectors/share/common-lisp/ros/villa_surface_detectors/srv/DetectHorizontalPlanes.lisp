; Auto-generated. Do not edit!


(cl:in-package villa_surface_detectors-srv)


;//! \htmlinclude DetectHorizontalPlanes-request.msg.html

(cl:defclass <DetectHorizontalPlanes-request> (roslisp-msg-protocol:ros-message)
  ((cloud_input
    :reader cloud_input
    :initarg :cloud_input
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass DetectHorizontalPlanes-request (<DetectHorizontalPlanes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectHorizontalPlanes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectHorizontalPlanes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name villa_surface_detectors-srv:<DetectHorizontalPlanes-request> is deprecated: use villa_surface_detectors-srv:DetectHorizontalPlanes-request instead.")))

(cl:ensure-generic-function 'cloud_input-val :lambda-list '(m))
(cl:defmethod cloud_input-val ((m <DetectHorizontalPlanes-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader villa_surface_detectors-srv:cloud_input-val is deprecated.  Use villa_surface_detectors-srv:cloud_input instead.")
  (cloud_input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectHorizontalPlanes-request>) ostream)
  "Serializes a message object of type '<DetectHorizontalPlanes-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectHorizontalPlanes-request>) istream)
  "Deserializes a message object of type '<DetectHorizontalPlanes-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectHorizontalPlanes-request>)))
  "Returns string type for a service object of type '<DetectHorizontalPlanes-request>"
  "villa_surface_detectors/DetectHorizontalPlanesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectHorizontalPlanes-request)))
  "Returns string type for a service object of type 'DetectHorizontalPlanes-request"
  "villa_surface_detectors/DetectHorizontalPlanesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectHorizontalPlanes-request>)))
  "Returns md5sum for a message object of type '<DetectHorizontalPlanes-request>"
  "a0f448ffc3cd900d5281565c7c294544")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectHorizontalPlanes-request)))
  "Returns md5sum for a message object of type 'DetectHorizontalPlanes-request"
  "a0f448ffc3cd900d5281565c7c294544")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectHorizontalPlanes-request>)))
  "Returns full string definition for message of type '<DetectHorizontalPlanes-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_input~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectHorizontalPlanes-request)))
  "Returns full string definition for message of type 'DetectHorizontalPlanes-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_input~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectHorizontalPlanes-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectHorizontalPlanes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectHorizontalPlanes-request
    (cl:cons ':cloud_input (cloud_input msg))
))
;//! \htmlinclude DetectHorizontalPlanes-response.msg.html

(cl:defclass <DetectHorizontalPlanes-response> (roslisp-msg-protocol:ros-message)
  ((horizontal_planes
    :reader horizontal_planes
    :initarg :horizontal_planes
    :type (cl:vector sensor_msgs-msg:PointCloud2)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:PointCloud2 :initial-element (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
   (horizontal_plane_coefs
    :reader horizontal_plane_coefs
    :initarg :horizontal_plane_coefs
    :type (cl:vector geometry_msgs-msg:Quaternion)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Quaternion :initial-element (cl:make-instance 'geometry_msgs-msg:Quaternion)))
   (horizontal_plane_bounding_boxes
    :reader horizontal_plane_bounding_boxes
    :initarg :horizontal_plane_bounding_boxes
    :type (cl:vector visualization_msgs-msg:Marker)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:Marker :initial-element (cl:make-instance 'visualization_msgs-msg:Marker)))
   (horizontal_plane_AA_bounding_boxes
    :reader horizontal_plane_AA_bounding_boxes
    :initarg :horizontal_plane_AA_bounding_boxes
    :type (cl:vector visualization_msgs-msg:Marker)
   :initform (cl:make-array 0 :element-type 'visualization_msgs-msg:Marker :initial-element (cl:make-instance 'visualization_msgs-msg:Marker))))
)

(cl:defclass DetectHorizontalPlanes-response (<DetectHorizontalPlanes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectHorizontalPlanes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectHorizontalPlanes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name villa_surface_detectors-srv:<DetectHorizontalPlanes-response> is deprecated: use villa_surface_detectors-srv:DetectHorizontalPlanes-response instead.")))

(cl:ensure-generic-function 'horizontal_planes-val :lambda-list '(m))
(cl:defmethod horizontal_planes-val ((m <DetectHorizontalPlanes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader villa_surface_detectors-srv:horizontal_planes-val is deprecated.  Use villa_surface_detectors-srv:horizontal_planes instead.")
  (horizontal_planes m))

(cl:ensure-generic-function 'horizontal_plane_coefs-val :lambda-list '(m))
(cl:defmethod horizontal_plane_coefs-val ((m <DetectHorizontalPlanes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader villa_surface_detectors-srv:horizontal_plane_coefs-val is deprecated.  Use villa_surface_detectors-srv:horizontal_plane_coefs instead.")
  (horizontal_plane_coefs m))

(cl:ensure-generic-function 'horizontal_plane_bounding_boxes-val :lambda-list '(m))
(cl:defmethod horizontal_plane_bounding_boxes-val ((m <DetectHorizontalPlanes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader villa_surface_detectors-srv:horizontal_plane_bounding_boxes-val is deprecated.  Use villa_surface_detectors-srv:horizontal_plane_bounding_boxes instead.")
  (horizontal_plane_bounding_boxes m))

(cl:ensure-generic-function 'horizontal_plane_AA_bounding_boxes-val :lambda-list '(m))
(cl:defmethod horizontal_plane_AA_bounding_boxes-val ((m <DetectHorizontalPlanes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader villa_surface_detectors-srv:horizontal_plane_AA_bounding_boxes-val is deprecated.  Use villa_surface_detectors-srv:horizontal_plane_AA_bounding_boxes instead.")
  (horizontal_plane_AA_bounding_boxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectHorizontalPlanes-response>) ostream)
  "Serializes a message object of type '<DetectHorizontalPlanes-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'horizontal_planes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'horizontal_planes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'horizontal_plane_coefs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'horizontal_plane_coefs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'horizontal_plane_bounding_boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'horizontal_plane_bounding_boxes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'horizontal_plane_AA_bounding_boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'horizontal_plane_AA_bounding_boxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectHorizontalPlanes-response>) istream)
  "Deserializes a message object of type '<DetectHorizontalPlanes-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'horizontal_planes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'horizontal_planes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:PointCloud2))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'horizontal_plane_coefs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'horizontal_plane_coefs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Quaternion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'horizontal_plane_bounding_boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'horizontal_plane_bounding_boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visualization_msgs-msg:Marker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'horizontal_plane_AA_bounding_boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'horizontal_plane_AA_bounding_boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'visualization_msgs-msg:Marker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectHorizontalPlanes-response>)))
  "Returns string type for a service object of type '<DetectHorizontalPlanes-response>"
  "villa_surface_detectors/DetectHorizontalPlanesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectHorizontalPlanes-response)))
  "Returns string type for a service object of type 'DetectHorizontalPlanes-response"
  "villa_surface_detectors/DetectHorizontalPlanesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectHorizontalPlanes-response>)))
  "Returns md5sum for a message object of type '<DetectHorizontalPlanes-response>"
  "a0f448ffc3cd900d5281565c7c294544")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectHorizontalPlanes-response)))
  "Returns md5sum for a message object of type 'DetectHorizontalPlanes-response"
  "a0f448ffc3cd900d5281565c7c294544")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectHorizontalPlanes-response>)))
  "Returns full string definition for message of type '<DetectHorizontalPlanes-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2[] horizontal_planes~%geometry_msgs/Quaternion[] horizontal_plane_coefs~%visualization_msgs/Marker[] horizontal_plane_bounding_boxes~%visualization_msgs/Marker[] horizontal_plane_AA_bounding_boxes~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectHorizontalPlanes-response)))
  "Returns full string definition for message of type 'DetectHorizontalPlanes-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2[] horizontal_planes~%geometry_msgs/Quaternion[] horizontal_plane_coefs~%visualization_msgs/Marker[] horizontal_plane_bounding_boxes~%visualization_msgs/Marker[] horizontal_plane_AA_bounding_boxes~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: visualization_msgs/Marker~%# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz~%~%uint8 ARROW=0~%uint8 CUBE=1~%uint8 SPHERE=2~%uint8 CYLINDER=3~%uint8 LINE_STRIP=4~%uint8 LINE_LIST=5~%uint8 CUBE_LIST=6~%uint8 SPHERE_LIST=7~%uint8 POINTS=8~%uint8 TEXT_VIEW_FACING=9~%uint8 MESH_RESOURCE=10~%uint8 TRIANGLE_LIST=11~%~%uint8 ADD=0~%uint8 MODIFY=0~%uint8 DELETE=2~%uint8 DELETEALL=3~%~%Header header                        # header for time/frame information~%string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%int32 type 		                       # Type of object~%int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%geometry_msgs/Pose pose                 # Pose of the object~%geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)~%std_msgs/ColorRGBA color             # Color [0.0-1.0]~%duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever~%bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%geometry_msgs/Point[] points~%#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)~%#number of colors must either be 0 or equal to the number of points~%#NOTE: alpha is not yet used~%std_msgs/ColorRGBA[] colors~%~%# NOTE: only used for text markers~%string text~%~%# NOTE: only used for MESH_RESOURCE markers~%string mesh_resource~%bool mesh_use_embedded_materials~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectHorizontalPlanes-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'horizontal_planes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'horizontal_plane_coefs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'horizontal_plane_bounding_boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'horizontal_plane_AA_bounding_boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectHorizontalPlanes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectHorizontalPlanes-response
    (cl:cons ':horizontal_planes (horizontal_planes msg))
    (cl:cons ':horizontal_plane_coefs (horizontal_plane_coefs msg))
    (cl:cons ':horizontal_plane_bounding_boxes (horizontal_plane_bounding_boxes msg))
    (cl:cons ':horizontal_plane_AA_bounding_boxes (horizontal_plane_AA_bounding_boxes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectHorizontalPlanes)))
  'DetectHorizontalPlanes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectHorizontalPlanes)))
  'DetectHorizontalPlanes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectHorizontalPlanes)))
  "Returns string type for a service object of type '<DetectHorizontalPlanes>"
  "villa_surface_detectors/DetectHorizontalPlanes")