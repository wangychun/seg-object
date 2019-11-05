;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ImageWithBBoxes)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ImageWithBBoxes (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::IMAGEWITHBBOXES")
  (make-package "DARKNET_ROS_MSGS::IMAGEWITHBBOXES"))

(in-package "ROS")
;;//! \htmlinclude ImageWithBBoxes.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass darknet_ros_msgs::ImageWithBBoxes
  :super ros::object
  :slots (_header _image _bboxes ))

(defmethod darknet_ros_msgs::ImageWithBBoxes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:bboxes __bboxes) (instance darknet_ros_msgs::BoundingBoxes :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image __image)
   (setq _bboxes __bboxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:bboxes
   (&rest __bboxes)
   (if (keywordp (car __bboxes))
       (send* _bboxes __bboxes)
     (progn
       (if __bboxes (setq _bboxes (car __bboxes)))
       _bboxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; darknet_ros_msgs/BoundingBoxes _bboxes
    (send _bboxes :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; darknet_ros_msgs/BoundingBoxes _bboxes
       (send _bboxes :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; darknet_ros_msgs/BoundingBoxes _bboxes
     (send _bboxes :deserialize buf ptr-) (incf ptr- (send _bboxes :serialization-length))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ImageWithBBoxes :md5sum-) "00c5fc14f2375b31e30409a084e3c69c")
(setf (get darknet_ros_msgs::ImageWithBBoxes :datatype-) "darknet_ros_msgs/ImageWithBBoxes")
(setf (get darknet_ros_msgs::ImageWithBBoxes :definition-)
      "Header header
sensor_msgs/Image image
BoundingBoxes bboxes

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
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: darknet_ros_msgs/BoundingBoxes
Header header
Header image_header
BoundingBox[] bounding_boxes

================================================================================
MSG: darknet_ros_msgs/BoundingBox
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string Class

")



(provide :darknet_ros_msgs/ImageWithBBoxes "00c5fc14f2375b31e30409a084e3c69c")


