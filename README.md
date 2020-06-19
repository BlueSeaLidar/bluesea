# bluesea
ROS driver for Lanhai USB 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build 

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable
2) rosrun bluesea bluesea_node _frame_id:=laser _port:=/dev/ttyUSB0 _baud_rate:=384000 _firmware_version:=2 _output_scan:=1 _output_cloud:=1 _mirror:=1 _from_zero:=1 _angle_patch:=1 _unit_is_mm:=1 _with_confidence:=1 _mirror:=1 _from_zero:=1 _angle_patch:=1 _unit_is_mm:=1 _with_confidence:=1
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

How to run Lanhai ros node (UDP Network Version)
=====================================================================
1) sudo ifconfig eth0:1 192.168.158.200 # add sub net
2) rosrun bluesea bluesea_node _frame_id:=map _type:=udp _dev_ip:=192.168.158.91 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 
