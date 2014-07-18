ROS Packages for the SR-SCARA-HX Manipulator from Sastra Robotics

NOTE: Work in progress, not complete


To run, clone this repo into your catkin worskpace 

cd catkin_ws/src

git clone https://github.com/sastrarobotics/sr_scara_hx_ros.git

now build it by running  catkin_make in your workspace directory (catkin_ws in this case)

now you can run either the simulated scara in rviz or on hardware

to run without the hardware, run

roslaunch sr_scara_hx_moveit_gen demo.launch


to run without the hardware, run

roslaunch sr_scara_hx_moveit_gen demo_controller.launch

