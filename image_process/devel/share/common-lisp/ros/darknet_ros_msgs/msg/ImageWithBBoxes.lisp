; Auto-generated. Do not edit!


(cl:in-package darknet_ros_msgs-msg)


;//! \htmlinclude ImageWithBBoxes.msg.html

(cl:defclass <ImageWithBBoxes> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (bboxes
    :reader bboxes
    :initarg :bboxes
    :type darknet_ros_msgs-msg:BoundingBoxes
    :initform (cl:make-instance 'darknet_ros_msgs-msg:BoundingBoxes)))
)

(cl:defclass ImageWithBBoxes (<ImageWithBBoxes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageWithBBoxes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageWithBBoxes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name darknet_ros_msgs-msg:<ImageWithBBoxes> is deprecated: use darknet_ros_msgs-msg:ImageWithBBoxes instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImageWithBBoxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:header-val is deprecated.  Use darknet_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <ImageWithBBoxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:image-val is deprecated.  Use darknet_ros_msgs-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'bboxes-val :lambda-list '(m))
(cl:defmethod bboxes-val ((m <ImageWithBBoxes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:bboxes-val is deprecated.  Use darknet_ros_msgs-msg:bboxes instead.")
  (bboxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageWithBBoxes>) ostream)
  "Serializes a message object of type '<ImageWithBBoxes>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bboxes) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageWithBBoxes>) istream)
  "Deserializes a message object of type '<ImageWithBBoxes>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bboxes) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageWithBBoxes>)))
  "Returns string type for a message object of type '<ImageWithBBoxes>"
  "darknet_ros_msgs/ImageWithBBoxes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageWithBBoxes)))
  "Returns string type for a message object of type 'ImageWithBBoxes"
  "darknet_ros_msgs/ImageWithBBoxes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageWithBBoxes>)))
  "Returns md5sum for a message object of type '<ImageWithBBoxes>"
  "00c5fc14f2375b31e30409a084e3c69c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageWithBBoxes)))
  "Returns md5sum for a message object of type 'ImageWithBBoxes"
  "00c5fc14f2375b31e30409a084e3c69c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageWithBBoxes>)))
  "Returns full string definition for message of type '<ImageWithBBoxes>"
  (cl:format cl:nil "Header header~%sensor_msgs/Image image~%BoundingBoxes bboxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageWithBBoxes)))
  "Returns full string definition for message of type 'ImageWithBBoxes"
  (cl:format cl:nil "Header header~%sensor_msgs/Image image~%BoundingBoxes bboxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageWithBBoxes>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bboxes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageWithBBoxes>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageWithBBoxes
    (cl:cons ':header (header msg))
    (cl:cons ':image (image msg))
    (cl:cons ':bboxes (bboxes msg))
))
