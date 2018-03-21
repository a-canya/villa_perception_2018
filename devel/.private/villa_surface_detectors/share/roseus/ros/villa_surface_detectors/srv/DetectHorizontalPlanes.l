;; Auto-generated. Do not edit!


(when (boundp 'villa_surface_detectors::DetectHorizontalPlanes)
  (if (not (find-package "VILLA_SURFACE_DETECTORS"))
    (make-package "VILLA_SURFACE_DETECTORS"))
  (shadow 'DetectHorizontalPlanes (find-package "VILLA_SURFACE_DETECTORS")))
(unless (find-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANES")
  (make-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANES"))
(unless (find-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANESREQUEST")
  (make-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANESREQUEST"))
(unless (find-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANESRESPONSE")
  (make-package "VILLA_SURFACE_DETECTORS::DETECTHORIZONTALPLANESRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "VISUALIZATION_MSGS"))
  (ros::roseus-add-msgs "visualization_msgs"))


(defclass villa_surface_detectors::DetectHorizontalPlanesRequest
  :super ros::object
  :slots (_cloud_input ))

(defmethod villa_surface_detectors::DetectHorizontalPlanesRequest
  (:init
   (&key
    ((:cloud_input __cloud_input) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _cloud_input __cloud_input)
   self)
  (:cloud_input
   (&rest __cloud_input)
   (if (keywordp (car __cloud_input))
       (send* _cloud_input __cloud_input)
     (progn
       (if __cloud_input (setq _cloud_input (car __cloud_input)))
       _cloud_input)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _cloud_input
    (send _cloud_input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _cloud_input
       (send _cloud_input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _cloud_input
     (send _cloud_input :deserialize buf ptr-) (incf ptr- (send _cloud_input :serialization-length))
   ;;
   self)
  )

(defclass villa_surface_detectors::DetectHorizontalPlanesResponse
  :super ros::object
  :slots (_horizontal_planes _horizontal_plane_coefs _horizontal_plane_bounding_boxes _horizontal_plane_AA_bounding_boxes ))

(defmethod villa_surface_detectors::DetectHorizontalPlanesResponse
  (:init
   (&key
    ((:horizontal_planes __horizontal_planes) (let (r) (dotimes (i 0) (push (instance sensor_msgs::PointCloud2 :init) r)) r))
    ((:horizontal_plane_coefs __horizontal_plane_coefs) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Quaternion :init) r)) r))
    ((:horizontal_plane_bounding_boxes __horizontal_plane_bounding_boxes) (let (r) (dotimes (i 0) (push (instance visualization_msgs::Marker :init) r)) r))
    ((:horizontal_plane_AA_bounding_boxes __horizontal_plane_AA_bounding_boxes) (let (r) (dotimes (i 0) (push (instance visualization_msgs::Marker :init) r)) r))
    )
   (send-super :init)
   (setq _horizontal_planes __horizontal_planes)
   (setq _horizontal_plane_coefs __horizontal_plane_coefs)
   (setq _horizontal_plane_bounding_boxes __horizontal_plane_bounding_boxes)
   (setq _horizontal_plane_AA_bounding_boxes __horizontal_plane_AA_bounding_boxes)
   self)
  (:horizontal_planes
   (&rest __horizontal_planes)
   (if (keywordp (car __horizontal_planes))
       (send* _horizontal_planes __horizontal_planes)
     (progn
       (if __horizontal_planes (setq _horizontal_planes (car __horizontal_planes)))
       _horizontal_planes)))
  (:horizontal_plane_coefs
   (&rest __horizontal_plane_coefs)
   (if (keywordp (car __horizontal_plane_coefs))
       (send* _horizontal_plane_coefs __horizontal_plane_coefs)
     (progn
       (if __horizontal_plane_coefs (setq _horizontal_plane_coefs (car __horizontal_plane_coefs)))
       _horizontal_plane_coefs)))
  (:horizontal_plane_bounding_boxes
   (&rest __horizontal_plane_bounding_boxes)
   (if (keywordp (car __horizontal_plane_bounding_boxes))
       (send* _horizontal_plane_bounding_boxes __horizontal_plane_bounding_boxes)
     (progn
       (if __horizontal_plane_bounding_boxes (setq _horizontal_plane_bounding_boxes (car __horizontal_plane_bounding_boxes)))
       _horizontal_plane_bounding_boxes)))
  (:horizontal_plane_AA_bounding_boxes
   (&rest __horizontal_plane_AA_bounding_boxes)
   (if (keywordp (car __horizontal_plane_AA_bounding_boxes))
       (send* _horizontal_plane_AA_bounding_boxes __horizontal_plane_AA_bounding_boxes)
     (progn
       (if __horizontal_plane_AA_bounding_boxes (setq _horizontal_plane_AA_bounding_boxes (car __horizontal_plane_AA_bounding_boxes)))
       _horizontal_plane_AA_bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2[] _horizontal_planes
    (apply #'+ (send-all _horizontal_planes :serialization-length)) 4
    ;; geometry_msgs/Quaternion[] _horizontal_plane_coefs
    (apply #'+ (send-all _horizontal_plane_coefs :serialization-length)) 4
    ;; visualization_msgs/Marker[] _horizontal_plane_bounding_boxes
    (apply #'+ (send-all _horizontal_plane_bounding_boxes :serialization-length)) 4
    ;; visualization_msgs/Marker[] _horizontal_plane_AA_bounding_boxes
    (apply #'+ (send-all _horizontal_plane_AA_bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2[] _horizontal_planes
     (write-long (length _horizontal_planes) s)
     (dolist (elem _horizontal_planes)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Quaternion[] _horizontal_plane_coefs
     (write-long (length _horizontal_plane_coefs) s)
     (dolist (elem _horizontal_plane_coefs)
       (send elem :serialize s)
       )
     ;; visualization_msgs/Marker[] _horizontal_plane_bounding_boxes
     (write-long (length _horizontal_plane_bounding_boxes) s)
     (dolist (elem _horizontal_plane_bounding_boxes)
       (send elem :serialize s)
       )
     ;; visualization_msgs/Marker[] _horizontal_plane_AA_bounding_boxes
     (write-long (length _horizontal_plane_AA_bounding_boxes) s)
     (dolist (elem _horizontal_plane_AA_bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2[] _horizontal_planes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _horizontal_planes (let (r) (dotimes (i n) (push (instance sensor_msgs::PointCloud2 :init) r)) r))
     (dolist (elem- _horizontal_planes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Quaternion[] _horizontal_plane_coefs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _horizontal_plane_coefs (let (r) (dotimes (i n) (push (instance geometry_msgs::Quaternion :init) r)) r))
     (dolist (elem- _horizontal_plane_coefs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; visualization_msgs/Marker[] _horizontal_plane_bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _horizontal_plane_bounding_boxes (let (r) (dotimes (i n) (push (instance visualization_msgs::Marker :init) r)) r))
     (dolist (elem- _horizontal_plane_bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; visualization_msgs/Marker[] _horizontal_plane_AA_bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _horizontal_plane_AA_bounding_boxes (let (r) (dotimes (i n) (push (instance visualization_msgs::Marker :init) r)) r))
     (dolist (elem- _horizontal_plane_AA_bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass villa_surface_detectors::DetectHorizontalPlanes
  :super ros::object
  :slots ())

(setf (get villa_surface_detectors::DetectHorizontalPlanes :md5sum-) "a0f448ffc3cd900d5281565c7c294544")
(setf (get villa_surface_detectors::DetectHorizontalPlanes :datatype-) "villa_surface_detectors/DetectHorizontalPlanes")
(setf (get villa_surface_detectors::DetectHorizontalPlanes :request) villa_surface_detectors::DetectHorizontalPlanesRequest)
(setf (get villa_surface_detectors::DetectHorizontalPlanes :response) villa_surface_detectors::DetectHorizontalPlanesResponse)

(defmethod villa_surface_detectors::DetectHorizontalPlanesRequest
  (:response () (instance villa_surface_detectors::DetectHorizontalPlanesResponse :init)))

(setf (get villa_surface_detectors::DetectHorizontalPlanesRequest :md5sum-) "a0f448ffc3cd900d5281565c7c294544")
(setf (get villa_surface_detectors::DetectHorizontalPlanesRequest :datatype-) "villa_surface_detectors/DetectHorizontalPlanesRequest")
(setf (get villa_surface_detectors::DetectHorizontalPlanesRequest :definition-)
      "sensor_msgs/PointCloud2 cloud_input

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
sensor_msgs/PointCloud2[] horizontal_planes
geometry_msgs/Quaternion[] horizontal_plane_coefs
visualization_msgs/Marker[] horizontal_plane_bounding_boxes
visualization_msgs/Marker[] horizontal_plane_AA_bounding_boxes


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
")

(setf (get villa_surface_detectors::DetectHorizontalPlanesResponse :md5sum-) "a0f448ffc3cd900d5281565c7c294544")
(setf (get villa_surface_detectors::DetectHorizontalPlanesResponse :datatype-) "villa_surface_detectors/DetectHorizontalPlanesResponse")
(setf (get villa_surface_detectors::DetectHorizontalPlanesResponse :definition-)
      "sensor_msgs/PointCloud2 cloud_input

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
sensor_msgs/PointCloud2[] horizontal_planes
geometry_msgs/Quaternion[] horizontal_plane_coefs
visualization_msgs/Marker[] horizontal_plane_bounding_boxes
visualization_msgs/Marker[] horizontal_plane_AA_bounding_boxes


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
")



(provide :villa_surface_detectors/DetectHorizontalPlanes "a0f448ffc3cd900d5281565c7c294544")


