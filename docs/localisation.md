## Localisation

# GNSS
- every satellite is constantly broadbasting its position *(x,y,z,t) ecef system

- gnss is not the most accurate, satellites have many sources of inaccuracy due to the operating env

## differential gnss
- uses a base station that knows its location very well, learns the difference and offsets the difference with the rover

## sensor fusion

- gnss + imu
- when u have 2 antennas on the receiver you can calculate the heading/azimuth of the car (rtk gnss)

## coordinate frames
- coordinate transform to sync the coordinates of different sensors to get homo coords
- can be represented as a 4x4 matrix
- combine transforms just need to multiply, inverse for the reverse

to achieve accuracy with gnss 
1. multiple satellite constellations
2. corrections from base station
3. fusion with imu

# ros bag files

nodes are code, bags hold data, topics are the medium
rostopic info
Publishers: 
 * /localization/novatel_oem7_localizer (http://delta-2006-04:42269/)

rosnode list
 Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /initialpose [geometry_msgs/PoseWithCovarianceStamped]
 * /novatel/oem7/bestpos [novatel_oem7_msgs/BESTPOS]
 * /novatel/oem7/inspva [novatel_oem7_msgs/INSPVA]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

the main source is the topic with the sensor name  * /novatel/oem7/bestpos [novatel_oem7_msgs/BESTPOS]
 * /novatel/oem7/inspva [novatel_oem7_msgs/INSPVA]

rostopic hz

Lesson2Finished 