## oil_filler_pose

#### This is a ros package for oil filler pose detect, it could calculate the 3D pose of an oil filler.

usage:

```
roslaunch oil_filler_pose oil_filter_detect.launch
```

param note: 
 - show: choose if to show the image and point cloud
 - useExact: message_filters params
 - useCompressed: message_filters params
 - camera: camera type, Currently supports realsense, tuyang
 - oil_frame_reference: publish the reference coordinates of the oil pose
 - topicColor: rgb image topic
 - topicDepth: depth image topic
