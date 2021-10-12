
* Module created to merge 2 laser sensor messages on one message for use 2 lidars on ROS, avoiding the use of PCL.


# How to Run

* Just clone the repository and change the names of the topic suscribed. 
* If one laser has less resolution should go into the auxiliar side.

search this lines on laser_merger.py

```
self.laser1_sub = rospy.Subscriber('/scan_main', LaserScan, self.callbackLaserMain)
self.laser2_sub = rospy.Subscriber('/scan_aux', LaserScan, self.callbackLaserAux)
```






