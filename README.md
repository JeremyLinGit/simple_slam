# simple_slam
A simple ROS slam code without optimization
system : Ubuntu 18.04 / ROS melodic    
It's the easiest and simplest slam project
Pros: The conputation is very little     
Cons: might get lost or have drift when too much noise.     
You could check out Gmapping or Hector slam if you are interested in more sophisticated ones.

The code subscribes the scan topic and odom and publish a occupancy grid map.   
It was tested on turtlebot gazebo simulation and have pretty good result. Haven't tested on real time machine.   

The result from turtlebot gazebo simulation.   
![Screenshot from 2021-03-24 15-39-55](https://user-images.githubusercontent.com/67049287/112332596-d4800280-8cf4-11eb-951d-f4c0f6497566.png).  


![Screenshot from 2021-03-24 15-38-53](https://user-images.githubusercontent.com/67049287/112332451-b61a0700-8cf4-11eb-828c-9045aa5e859a.png)
  
  
