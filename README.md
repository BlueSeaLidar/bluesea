# bluesea
ROS driver for Lanhai USB 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build 

How to run Lanhai ros node
=====================================================================
1) sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable
2) rosrun bluesea bluesea_node _frame_id:=laser _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 
