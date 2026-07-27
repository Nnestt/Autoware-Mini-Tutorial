## ROS

- basically mqtt 

`rosmsg echo`
see all commands

useful commands

topic
 `rostopic list`
 `rostopic info` - sees pub sub

`rostopc hz /vehicle/vehicle_status`
 `rostopic echo vehicle/vehicle_status`

 `rqt_topic` open gui
 `rqt_graph`

node
 `rosnode list` 
 `rosnode info /control/pure/pursuit/controller`

msg
 `rosmsg list`
 list of message types

 `rosmsg show autoware_mini/VehicleStatus`

Is the topic even published? rostopic list
Is there activity in that topic? rostopic hz /topic/name
Are the contents of the topic what is expected? rostopic echo /topic/name
To skip long arrays: rostopic echo --noarr /topic/name
Who are publishing and subscribing the topic? rostopic info /topic/name
Where are the subscribers publishing? rosnode info /node/name


hi

