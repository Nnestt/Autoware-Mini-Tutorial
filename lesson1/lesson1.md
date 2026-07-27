```
student@delta-2006-04:~/ernest$ rostopic list
/message
/rosout
/rosout_agg
student@delta-2006-04:~/ernest$ rostopic info /message\
> ^C
student@delta-2006-04:~/ernest$ rostopic info /message
Type: std_msgs/String

Publishers: 
 * /publisher (http://delta-2006-04:40189/)

Subscribers: None


student@delta-2006-04:~/ernest$ rostopic echo /message
data: "Hello world!"
---
data: "Hello world!"
---
data: "Hello world!"
---
data: "Hello world!"
---
data: "Hello world!"
---
data: "Hello world!"
---
^Cstudent@delta-2006-04:~/ernest$ rostopic hz /message
subscribed to [/message]
average rate: 1.999
        min: 0.500s max: 0.500s std dev: 0.00000s window: 2
average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00020s window: 4
average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00022s window: 6
average rate: 2.000
        min: 0.500s max: 0.500s std dev: 0.00021s window: 8
^Cno new messages

```
