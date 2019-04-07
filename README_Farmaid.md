## Setting up the ROS environment

---

Open a new terminal. If the Anaconda environment is activated, deactivate it by running `conda deactivate`.

Source the ROS catkin workspace by running:

```
$ cd ~/path_to_catkin_ws/
$ source devel/setup.bash
```



## USB Camera and AprilTags

---

Launch the USB camera node and AprilTag detection node by running:

```
$ roslaunch robot_launch robot.launch
```

The `robot.launch` file launches two other launch files: `camera_rect.launch` and `tag_detection.launch`. `camera_rect.launch` starts the USB camera node. In this file is where you can set the appropriate device name. For example, if I want to use the USB webcam I would set the device name as "/dev/video1". If I want to use the integrated webcam, the device name would be "/dev/video0". `tag_detection.launch` starts the AprilTag detector node. In this file is where you can set the size of each AprilTag corresponding to its ID number.

After launching the above, you can view the AprilTags being detected while the webcam is streaming by opening another terminal and running `rqt_image_view`. In the drop-down menu, select "/camera/tag_detections_image" and hold the AprilTag in front of the camera to verify that it gets detected.

To see the pose of the AprilTags detected with respect to the camera, open another terminal and run `rostopic echo /camera/tag_detections_pose`.

## Converting AprilTag measurements to robot coordinate frame

---

The AprilTag detector node publishes pose information to the "topic_name" topic. This is the pose of the tag with respect to the camera coordinate frame. The ultimate goal is to get the pose of the tag in the robot coordinate frame.

Before deriving the final results, I plan to study the RosInterface class that was provided by the Robotics Capstone course from Coursera in hopes of reusing as much of their code as possible. A brief description of their code is provided below.

## Publishing AprilTag static transform

---

The AprilTags will have a fixed position and orientation in the world map. Thus, we will have to publish their pose information with respect to the world coordinate frame. One way to publish a static transform is through a launch file. Create a new launch file and use the code below as an example:

```
<launch>
        <node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0.1 0 0.455 0 0 0 base_link camera_link 100"/>
</launch>
```

## Making a tf broadcaster

---

Create a new Python file called **cam2tag_broadcaster.py** and copy the code below.

```python
#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)


def tag_pose_callback(pose_array):

    if (len(pose_array.detections)==0):
        return

    for i in range(len(pose_array.detections)):
        pose = pose_array.detections[i].pose.pose
        tag_tf = geometry_msgs.msg.TransformStamped()
        tag_tf.header.stamp = rospy.Time.now()
        tag_tf.header.frame_id = "camera"
        tag_tf.child_frame_id = "tag" + str(i)
        tag_tf.transform.translation.x = pose.position.x
        tag_tf.transform.translation.y = pose.position.y
        tag_tf.transform.translation.z = pose.position.z
        tag_tf.transform.rotation.x = pose.orientation.x
        tag_tf.transform.rotation.y = pose.orientation.y
        tag_tf.transform.rotation.z = pose.orientation.z
        tag_tf.transform.rotation.w = pose.orientation.w
        br.sendTransform(tag_tf)

if __name__ == '__main__':
    rospy.init_node('cam2tag_broadcaster')

    sub = rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,tag_pose_callback)
    br = tf2_ros.TransformBroadcaster()

    rospy.spin()

```

Be sure to make the Python file an executable by running `chmod +x cam2tag_broadcaster.py`.

## Mount Raspbian image file in Ubuntu

I have a .img file that contains the Raspbian OS and some other files. This .img file is meant to be flashed onto the Raspberry Pi. However, I only want to access some of the files on this .img file. How do I access the contents of the .img file on my Ubuntu OS?

Check the file system table with fdisk:

```
$ fdisk -l coursera_robotics_capstone_pi.img
Disk coursera_robotics_capstone_pi.img: 6.5 GiB, 6920601600 bytes, 13516800 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x54cef515

Device                             Boot  Start      End  Sectors  Size Id Type
coursera_robotics_capstone_pi.img1        8192   131071   122880   60M  c W95 FAT32 (LBA)
coursera_robotics_capstone_pi.img2      131072 13443071 13312000  6.4G 83 Linux
```

The important parts from the above output are:

* Sector size: 512
* Start blocks for img1 (8192) and img2 (131072)

Now create a directory mount point for both images:

```
mkdir img1 img2
```

When your mount point directories are ready, mount both images with the sector size and start block information you have gathered in previous step:

```
$ sudo mount coursera_robotics_capstone_pi.img -o loop,offset=$((512*8192)) img1/
$ sudo mount coursera_robotics_capstone_pi.img -o loop,offset=$((512*131072)) img2/
```

The contents are now contained in the `img1/` and `img2/` folders.

### References

1. https://linuxconfig.org/how-to-mount-rasberry-pi-filesystem-image

## Creating a catkin package

Navigate to the `src` folder of your catkin workspace and create the package with the name and dependencies as arguments:

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg farmaid_bot std_msgs rospy roscpp
```

The general form is shown below.

```
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

To build a catkin package in the catkin workspace:

```
$ cd ~/catkin-ws
$ catkin_make
```

To add the workspace to your ROS environment you need to source the generated setup file:

```
$ . ~/catkin_ws/devel/setup.bash
```

### References

1. http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage
2. 






