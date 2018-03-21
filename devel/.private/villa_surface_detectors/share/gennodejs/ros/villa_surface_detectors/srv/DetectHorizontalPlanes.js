// Auto-generated. Do not edit!

// (in-package villa_surface_detectors.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');
let visualization_msgs = _finder('visualization_msgs');

//-----------------------------------------------------------

class DetectHorizontalPlanesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cloud_input = null;
    }
    else {
      if (initObj.hasOwnProperty('cloud_input')) {
        this.cloud_input = initObj.cloud_input
      }
      else {
        this.cloud_input = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectHorizontalPlanesRequest
    // Serialize message field [cloud_input]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectHorizontalPlanesRequest
    let len;
    let data = new DetectHorizontalPlanesRequest(null);
    // Deserialize message field [cloud_input]
    data.cloud_input = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'villa_surface_detectors/DetectHorizontalPlanesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '863a5c747de8277a3d3cbdc41ceca684';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 cloud_input
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectHorizontalPlanesRequest(null);
    if (msg.cloud_input !== undefined) {
      resolved.cloud_input = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_input)
    }
    else {
      resolved.cloud_input = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

class DetectHorizontalPlanesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.horizontal_planes = null;
      this.horizontal_plane_coefs = null;
      this.horizontal_plane_bounding_boxes = null;
      this.horizontal_plane_AA_bounding_boxes = null;
    }
    else {
      if (initObj.hasOwnProperty('horizontal_planes')) {
        this.horizontal_planes = initObj.horizontal_planes
      }
      else {
        this.horizontal_planes = [];
      }
      if (initObj.hasOwnProperty('horizontal_plane_coefs')) {
        this.horizontal_plane_coefs = initObj.horizontal_plane_coefs
      }
      else {
        this.horizontal_plane_coefs = [];
      }
      if (initObj.hasOwnProperty('horizontal_plane_bounding_boxes')) {
        this.horizontal_plane_bounding_boxes = initObj.horizontal_plane_bounding_boxes
      }
      else {
        this.horizontal_plane_bounding_boxes = [];
      }
      if (initObj.hasOwnProperty('horizontal_plane_AA_bounding_boxes')) {
        this.horizontal_plane_AA_bounding_boxes = initObj.horizontal_plane_AA_bounding_boxes
      }
      else {
        this.horizontal_plane_AA_bounding_boxes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectHorizontalPlanesResponse
    // Serialize message field [horizontal_planes]
    // Serialize the length for message field [horizontal_planes]
    bufferOffset = _serializer.uint32(obj.horizontal_planes.length, buffer, bufferOffset);
    obj.horizontal_planes.forEach((val) => {
      bufferOffset = sensor_msgs.msg.PointCloud2.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [horizontal_plane_coefs]
    // Serialize the length for message field [horizontal_plane_coefs]
    bufferOffset = _serializer.uint32(obj.horizontal_plane_coefs.length, buffer, bufferOffset);
    obj.horizontal_plane_coefs.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Quaternion.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [horizontal_plane_bounding_boxes]
    // Serialize the length for message field [horizontal_plane_bounding_boxes]
    bufferOffset = _serializer.uint32(obj.horizontal_plane_bounding_boxes.length, buffer, bufferOffset);
    obj.horizontal_plane_bounding_boxes.forEach((val) => {
      bufferOffset = visualization_msgs.msg.Marker.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [horizontal_plane_AA_bounding_boxes]
    // Serialize the length for message field [horizontal_plane_AA_bounding_boxes]
    bufferOffset = _serializer.uint32(obj.horizontal_plane_AA_bounding_boxes.length, buffer, bufferOffset);
    obj.horizontal_plane_AA_bounding_boxes.forEach((val) => {
      bufferOffset = visualization_msgs.msg.Marker.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectHorizontalPlanesResponse
    let len;
    let data = new DetectHorizontalPlanesResponse(null);
    // Deserialize message field [horizontal_planes]
    // Deserialize array length for message field [horizontal_planes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.horizontal_planes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.horizontal_planes[i] = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [horizontal_plane_coefs]
    // Deserialize array length for message field [horizontal_plane_coefs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.horizontal_plane_coefs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.horizontal_plane_coefs[i] = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [horizontal_plane_bounding_boxes]
    // Deserialize array length for message field [horizontal_plane_bounding_boxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.horizontal_plane_bounding_boxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.horizontal_plane_bounding_boxes[i] = visualization_msgs.msg.Marker.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [horizontal_plane_AA_bounding_boxes]
    // Deserialize array length for message field [horizontal_plane_AA_bounding_boxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.horizontal_plane_AA_bounding_boxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.horizontal_plane_AA_bounding_boxes[i] = visualization_msgs.msg.Marker.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.horizontal_planes.forEach((val) => {
      length += sensor_msgs.msg.PointCloud2.getMessageSize(val);
    });
    length += 32 * object.horizontal_plane_coefs.length;
    object.horizontal_plane_bounding_boxes.forEach((val) => {
      length += visualization_msgs.msg.Marker.getMessageSize(val);
    });
    object.horizontal_plane_AA_bounding_boxes.forEach((val) => {
      length += visualization_msgs.msg.Marker.getMessageSize(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'villa_surface_detectors/DetectHorizontalPlanesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7372d43733aad47072674fd9087cad3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2[] horizontal_planes
    geometry_msgs/Quaternion[] horizontal_plane_coefs
    visualization_msgs/Marker[] horizontal_plane_bounding_boxes
    visualization_msgs/Marker[] horizontal_plane_AA_bounding_boxes
    
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectHorizontalPlanesResponse(null);
    if (msg.horizontal_planes !== undefined) {
      resolved.horizontal_planes = new Array(msg.horizontal_planes.length);
      for (let i = 0; i < resolved.horizontal_planes.length; ++i) {
        resolved.horizontal_planes[i] = sensor_msgs.msg.PointCloud2.Resolve(msg.horizontal_planes[i]);
      }
    }
    else {
      resolved.horizontal_planes = []
    }

    if (msg.horizontal_plane_coefs !== undefined) {
      resolved.horizontal_plane_coefs = new Array(msg.horizontal_plane_coefs.length);
      for (let i = 0; i < resolved.horizontal_plane_coefs.length; ++i) {
        resolved.horizontal_plane_coefs[i] = geometry_msgs.msg.Quaternion.Resolve(msg.horizontal_plane_coefs[i]);
      }
    }
    else {
      resolved.horizontal_plane_coefs = []
    }

    if (msg.horizontal_plane_bounding_boxes !== undefined) {
      resolved.horizontal_plane_bounding_boxes = new Array(msg.horizontal_plane_bounding_boxes.length);
      for (let i = 0; i < resolved.horizontal_plane_bounding_boxes.length; ++i) {
        resolved.horizontal_plane_bounding_boxes[i] = visualization_msgs.msg.Marker.Resolve(msg.horizontal_plane_bounding_boxes[i]);
      }
    }
    else {
      resolved.horizontal_plane_bounding_boxes = []
    }

    if (msg.horizontal_plane_AA_bounding_boxes !== undefined) {
      resolved.horizontal_plane_AA_bounding_boxes = new Array(msg.horizontal_plane_AA_bounding_boxes.length);
      for (let i = 0; i < resolved.horizontal_plane_AA_bounding_boxes.length; ++i) {
        resolved.horizontal_plane_AA_bounding_boxes[i] = visualization_msgs.msg.Marker.Resolve(msg.horizontal_plane_AA_bounding_boxes[i]);
      }
    }
    else {
      resolved.horizontal_plane_AA_bounding_boxes = []
    }

    return resolved;
    }
};

module.exports = {
  Request: DetectHorizontalPlanesRequest,
  Response: DetectHorizontalPlanesResponse,
  md5sum() { return 'a0f448ffc3cd900d5281565c7c294544'; },
  datatype() { return 'villa_surface_detectors/DetectHorizontalPlanes'; }
};
