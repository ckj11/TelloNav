# TelloNav
Tello PID controller that accepts waypoints via ROS with a queue system. Since the plugin subscribes to ROS topic, any path planning algorithm that can publish to that rostopic can be used. 
Ensure that gazebo and ROS noetic is installed. 

# Getting Started:

1. Setup Catkin Workspace
2. clone repository in the src folder
3. Run `catkin_make`
3. run `source devel/setup.bash`

# gazeboNav
This package contains the worlds and launch files. When starting the simulation with the tello drone, run `roslaunch gazeboNav environment.world.launch`.
This starts gazebo and ros with the plugin for Tello PID control. 

You can change which world is launched by editing `/(name of your workspace)/src/TelloNav/gazeboNav/launch/environment.world.launch`.
Edit the name of the world that you want to launch on line 4 in the value field. For more information about creating gazebo worlds, look here http://classic.gazebosim.org/tutorials. 

# droneNav
This package contains the plugin and PID controller for the drone. Currently, it only supports x, y, z, and yaw control, but can be modified
to do other degrees of freedom as well. The plugin is integrated with ROS and subscribes to a topic called `/Tello/target`. The controller will
attempt to get the drone as close to those specified x, y, z, and yaw as possible.

The drone has a degree of tolerance of error before moving onto the next point. It will not move onto the next queued target point until it comes within
the tolerance. This was accomplised using a custom callbackqueue method that only called one message at a time. 

# nav1 and nav2
These packages are aStar enabled directors for sending waypoints to the drones. They subscribe specificially to `/Tello1/target` and `/Tello2/target`
respectively. In order to run both of them, run `rosrun nav1 nav1` and `rosrun nav2 nav2` after launching the gazeboNav package. 

# Tello Model
Make sure to put the Tello model your system gazebo model path so that the plugin can find it. It is located in `(name of your workspace)/src/Tellonav/models`. 
