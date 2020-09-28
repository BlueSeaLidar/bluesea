# bluesea
ROS driver for Lanhai USB 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build 

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable
2) 
## if your lidar model is LDS-50C-2 :
* rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=1 _output_cloud:=1 _mirror:=1 _from_zero:=1 _angle_patch:=1 _unit_is_mm:=1 _with_confidence:=1
* or use roslaunch src/bluesea/launch/LDS-50C.launch
    
## if your lidar model is LDS-25C or LDS-50C:
* rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=1 _output_cloud:=1 _mirror:=1 _from_zero:=1 _angle_patch:=1 _unit_is_mm:=0 _with_confidence:=1
* or use roslaunch src/bluesea/launch/LDS-50C.launch    

3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

How to run Lanhai ros node (UDP Network Version)
=====================================================================
1) sudo ifconfig eth0:1 192.168.158.200 # add sub net
2) rosrun bluesea bluesea_node _frame_id:=map _type:=udp _dev_ip:=192.168.158.91 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

Parameters
=====================================================================
* std::string type; // LiDAR comm type, could be "uart" or "udp"
* std::string dump;	// file path of dump raw data, for debug

// for serial port comm
* std::string port; // serial port device path
* int baud_rate; // baud rate

// for network comm
* std::string dev_ip; // network 
* int udp_port, tcp_port; 
//	
* int unit_is_mm; //  0 : unit of raw data distance is CM, 1: MM
* int with_confidence; // 1: raw data with intensity, 0: no intensity
* int with_checksum; // 1 : enable packet checksum
* int output_scan; // 1: enable output angle+distance mode, 0: disable
* int output_cloud; // 1: enable output xyz format data, 0 : disable
* int output_360; // 1: collect multiple RawData packets (360 degree), then publish
				// 0: publish every RawData (36 degree)
* std::string frame_id;	// frame information, could be used for rviz
// output data format
* int mirror; // 0: clockwise, 1: counterclockwise
* int from_zero; // 1: angle range 0 - 360, 0: angle range -180 - +180
* int angle_patch; // 1: make points number of every fans to unique
