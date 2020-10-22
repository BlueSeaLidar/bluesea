
/*********************************************************************
 * This is demo for ROS refering to xv_11_laser_driver.
 * serial port version  LDS-25: 
 rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=1 _output_cloud:=0 _unit_is_mm:=0 _with_confidence:=0
 * serial port version  LDS-50: 
 rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=1 _output_cloud:=0 _unit_is_mm:=1 _with_confidence:=1
 * UDP network version like this:
 rosrun bluesea bluesea_node _frame_id:=map _type:=udp _dev_ip:=192.168.158.91 _firmware_version:=2 _output_scan:=1 _output_cloud:=1
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
//#define ROS_ERROR printf
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h> 
#include <arpa/inet.h> 
#include <stdarg.h>
#include <pthread.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>

extern "C" int change_baud(int fd, int baud);



//#include <linux/termios.h>
//
struct RawDataHdr
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
};

struct RawData
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short distance[1000];
	unsigned char confidence[1000];
};


#define PI 3.1415926

#define HDR_SIZE 6

#define BUF_SIZE 512*1024

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};

// serial port handle
int g_uart_port = -1;

// UDP socket
int g_udp_socket = -1;

// parameters for network comm
std::string dev_ip;
int udp_port, tcp_port;
	
// CRC32
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

// send pacecat command to lidar serial port
int send_cmd_uart(int fd_uart, unsigned short cmd_id, int len, const void* cmd_body)
{
	char buffer[2048] = {0};
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd_id;
	static unsigned short sn = 1;
	hdr->sn = sn++;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd_body, len);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	return write(fd_uart, buffer, len + sizeof(CmdHeader) + 4);
}

// 
int send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const char* snd_buf)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	char s[3096];
	for (int i = 0; i < len2; i++) 
		sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);
	ROS_INFO("send to %s:%d 0x%04x sn[%d] L=%d : %s", 
			dev_ip, dev_port, cmd, sn, len, s);

	return 0;
}

// 
int open_serial_port(const char* port, int baudrate) 
{
       	int fd = open(port,  O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) { 
		ROS_ERROR("open %s error", port); 
		return -1; 
       	}
	
	int   		ret; 
	struct termios	attrs;

	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if(ret < 0) {
		ROS_ERROR("get attrs failed");
		return -1;
	}

	/* set speed */
	int speed = B230400;
	//if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);//[baudrate]);  
	ret |= cfsetospeed(&attrs, speed);//[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1) {//parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB;			//disable parity
	       	attrs.c_iflag &= ~INPCK;
	} else {
		attrs.c_cflag |= (PARENB | PARODD);	//enable parity
	       	attrs.c_iflag |= INPCK;
		//if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB;	// 1 stop bit
	//attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
        attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

//	attrs.c_cc[VMIN] = 0;
//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if(tcsetattr(fd, TCSANOW, &attrs) < 0) 
	{
		close(fd);
	       	ROS_ERROR("tcsetattr err");
	       	return -1;
	}

	if ( change_baud(fd, baudrate) )
	{
		close(fd);
	       	ROS_ERROR("fail to set baudrate %d", baudrate);
	       	return -1;
	}

	return fd;
}

// translate lidar raw data to ROS laserscan message
bool parse_data_3(int len, unsigned char* buf, RawData& dat, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-18)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			idx += 8;
		}
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}
	
		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			ROS_ERROR("bad angle %d", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
		{
			ROS_ERROR("points number %d seem not correct", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N*3 + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		unsigned char* pdat = buf+idx+HDR_SIZE;
		for (int i=0; i<hdr.N; i++)
		{
			dat.confidence[i] = *pdat++;
			sum += dat.confidence[i];

			unsigned short v = *pdat++;
			unsigned short v2 = *pdat++;
			dat.distance[i] = (v2<<8) | v;

			sum += dat.distance[i];
		}

		memcpy(&chk, pdat, 2);

		if (with_chk != 0 && chk != sum) 
		{
			ROS_ERROR("chksum3 error");
			consume = idx + HDR_SIZE + 3*hdr.N + 2;
			return 0;
		}

		memcpy(&dat, &hdr, HDR_SIZE);
		//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
		//printf("get3 %d(%d)\n", hdr.angle, hdr.N);
		
		idx += HDR_SIZE + 3*hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx/2;
	return false;
}

bool parse_data(int len, unsigned char* buf, RawData& dat, int is_mm, int with_conf, int& consume, int with_chk) 
{
	int idx = 0;
	while (idx < len-180)
	{
		if (buf[idx] == 'S' && buf[idx+1] == 'T' && buf[idx+6] == 'E' && buf[idx+7] == 'D')
		{
			idx+=8;
		}
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}
			
		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			ROS_ERROR("bad angle %d", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
		{
			ROS_ERROR("points number %d seem not correct", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N*sizeof(short) + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		unsigned char* pdat = buf+idx+HDR_SIZE;
		for (int i=0; i<hdr.N; i++)
		{
			unsigned short v = *pdat++;
			unsigned short v2 = *pdat++;
			unsigned short val = (v2<<8) | v;

			if (with_conf)
			{
				dat.confidence[i] = val >> 13;
			       	dat.distance[i] = val & 0x1fff;
				if (is_mm == 0) dat.distance[i] *= 10;
			} else {
				dat.confidence[i] = is_mm ? val : val*10;
				dat.confidence[i] = 0;
			}

			sum += val;
		}
		memcpy(&chk, buf+idx+HDR_SIZE+hdr.N*2, 2);

		if (with_chk != 0 && chk != sum) 
		{
			ROS_ERROR("chksum error");
			consume = idx + HDR_SIZE + 2*hdr.N + 2;
			return 0;
		}

		memcpy(&dat, &hdr, HDR_SIZE);
		// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);

		idx += HDR_SIZE + 2*hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx/2;
	return false;
}

int try_serial_port(const char* port, int baud_rate) 
{
	int fd = open_serial_port(port, baud_rate);
	if (fd < 0) {
		return -1;
	}
	
	unsigned char* buf = new unsigned char[4096];

	int nr = 0;
	time_t t = time(NULL);
	while (nr < 4096) 
	{
		int n = read(fd, buf+nr, 4096-nr);
		if (n > 0) nr += n;
		if (time(NULL) > t+1) break;
	}
	close(fd);

	int fnd = -1;
	if (nr == 4096) 
	{
		RawData dat;
		int consume;

		if (parse_data_3(nr, buf, dat, consume, 1) ) fnd = 0;
		if (parse_data(nr, buf, dat, 0, 0, consume, 1) ) fnd = 0;
		if (parse_data(nr, buf, dat, 0, 1, consume, 1) ) fnd = 0;
		if (parse_data(nr, buf, dat, 1, 0, consume, 1) ) fnd = 0;
		if (parse_data(nr, buf, dat, 1, 1, consume, 1) ) fnd = 0;
	}
	delete buf;
	return fnd;
}




#if 0//def NEXT
int parse_data(int len, unsigned char* buf, sensor_msgs::LaserScan& scan_msg, int& consume) 
{
	int idx = 0;
	while (idx < len-128)
	{
		if (buf[idx] != 0xce || buf[idx+1] != 0xfa)
		{
			idx++;
			continue;
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			ROS_ERROR("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 1000 || hdr.N < 30) 
		{
			ROS_ERROR("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (idx + HDR_SIZE + hdr.N*sizeof(short) + 2 > len)
		{
			// data packet not complete
			break;
		}

		// calc checksum
		unsigned short sum = hdr.angle + hdr.N, chk;
		for (int i=0; i<hdr.N; i++)
		{
			unsigned short v;
			memcpy(&v, buf+idx+HDR_SIZE+i*2, 2);
			sum += v;
		}
		memcpy(&chk, buf+idx+HDR_SIZE+hdr.N*2, 2);

		if (chk != sum) 
		{
			ROS_ERROR("chksum error");
			consume = idx + HDR_SIZE + 2*hdr.N + 2;
			return 0;
		}

		scan_msg.angle_min = hdr.angle*M_PI / 1800;
		scan_msg.angle_max = scan_msg.angle_min + M_PI*360/1800;
		scan_msg.angle_increment = M_PI*360/1800 / hdr.N;

		double scan_time = 60/4000.;
		scan_msg.scan_time = scan_time;
	       	scan_msg.time_increment = scan_time / (double)(hdr.N);

		scan_msg.range_min = 0.;
	       	scan_msg.range_max = 100;//max_distance;//8.0;

		scan_msg.intensities.resize(hdr.N);
	       	scan_msg.ranges.resize(hdr.N);

		for (int i=0; i<hdr.N; i++) 
		{
			unsigned short val;
			memcpy(&val, buf + idx + HDR_SIZE + i*2, 2);

			unsigned short dist = val & 0x1FFF;

			scan_msg.ranges[i] = dist > 0 ? dist/100.0 : 
				scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            
			scan_msg.intensities[i] = 1 + (float) (val >> 13);
	       	}
		idx += HDR_SIZE + 2*hdr.N + 2;
		consume = idx;
		return 1;
	}
	if (idx > 1024) consume = idx/2;
	return 0;
}
#endif

// service call back function
bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	char cmd[] = "LSTOPH";
	if ( g_uart_port != -1) {
		write(g_uart_port, cmd, 6);
		return true;
	}

	if (g_udp_socket != -1) 
	{
		send_cmd_udp(g_udp_socket, dev_ip.c_str(), udp_port, 
				0x0043, rand(), 6, cmd);
		ROS_INFO("Stop motor");
		return true;
	}
	return false;
}

// service call back function
bool start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	char cmd[] = "LSTARH";
	if ( g_uart_port != -1) {
		write(g_uart_port, cmd, 6);
		return true;
	}

	if (g_udp_socket != -1) 
	{
		send_cmd_udp(g_udp_socket, dev_ip.c_str(), udp_port, 
				0x0043, rand(), 6, cmd);
		ROS_INFO("Start motor");
		return true;
	}

	return false;
}

int quirk_talk(int fd, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	ROS_INFO("send command : %s", cmd);
	write(fd, cmd, n);
			
	char buf[1024];
	int nr = read(fd, buf, sizeof(buf));

	for (int i=0; i<10 && nr < (int)sizeof(buf); i++)
	{
		int n = read(fd, buf+nr, sizeof(buf)-nr);
		if (n > 0)
		       	nr += n;
		else 
			ros::Duration(0.1).sleep();
	}

	for (int i=0; i<(int)sizeof(buf)-nhdr-nfetch; i++) 
	{
		if (memcmp(buf+i, hdr_str, nhdr) == 0) 
		{
			memcpy(fetch, buf+i+nhdr, nfetch);
			fetch[nfetch] = 0;
			return 0;
		}
	}
	char path[256];
	sprintf(path, "/tmp/%s.dat", hdr_str);
	FILE* fp = fopen(path, "wb");
	if (fp) {
		fwrite(buf, 1, sizeof(buf), fp);
		fclose(fp);
	}

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return -1;
}

// find right baudrate in all possible 
int detect_baudrate(std::string& port)
{
	int possible_rates[] = { 921600, 768000, 500000, 384000, 256000, 230400 };

	int all = sizeof(possible_rates)/sizeof(possible_rates[0]);
	for (int i=0; i<all; i++)
	{
		if ( try_serial_port(port.c_str(), possible_rates[i]) == 0)
		{
			return possible_rates[i]; 
		}
	}
		
       	return -1;
}

int detect_baudrate_LDS25(std::string& port, int& unit_is_mm, int& with_confidence)
{
	int baud_rate;
	if ( try_serial_port(port.c_str(), 38400) == 0)
	{
		baud_rate = 384000; 
		unit_is_mm = 1; 
		with_confidence = 1;
		ROS_INFO("is 384000");
	} else
	{
		baud_rate = 234000; 
		unit_is_mm = 0; 
		with_confidence = 0;
		ROS_INFO("is 234000");
	}
	return baud_rate;
}


int reopen_uart(std::string& port, int baud_rate, int& unit_is_mm, int& with_confidence)
{
	if (baud_rate == -1) 
	{
		baud_rate = detect_baudrate(port);
		if (baud_rate == -1)
		{
			ROS_ERROR("could not find right baudrate for %s", port.c_str());
			return -1;
		}
	}
	if (baud_rate == -77) 
		baud_rate = detect_baudrate_LDS25(port, unit_is_mm, with_confidence);
	
	ROS_INFO("open %s @ %d", port.c_str(), baud_rate);
	return open_serial_port(port.c_str(), baud_rate);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea_laser_publisher");
       	ros::NodeHandle n;
       	ros::NodeHandle priv_nh("~");
	
	std_msgs::UInt16 rpms; 

	// LiDAR comm type, could be "uart" or "udp"
	std::string type;
	priv_nh.param("type", type, std::string("uart")); 

	// dump raw data for debug
	std::string dump_path;
	priv_nh.param("dump", dump_path, std::string("")); 

	FILE* fp_dump = NULL;
	if (!dump_path.empty())
		fp_dump = fopen(dump_path.c_str(), "wb");

	//////////////////////////////////////////////////////////////
	// for serial port comm
	std::string port;
       	int baud_rate;
	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
       	priv_nh.param("baud_rate", baud_rate, 256000);

	// for network comm
	priv_nh.param("dev_ip", dev_ip, std::string("192.168.158.91"));
	priv_nh.param("udp_port", udp_port, 5000);
	priv_nh.param("tcp_port", tcp_port, 5000);

	// raw data format
	int unit_is_mm, with_confidence, with_chk, normal_size;
	priv_nh.param("unit_is_mm", unit_is_mm, 1); // 0 : distance is CM, 1: MM
       	priv_nh.param("with_confidence", with_confidence, 1); // 
       	priv_nh.param("with_checksum", with_chk, 1); // 1 : enable packet checksum
       	priv_nh.param("normal_size", normal_size, -1); // -1 : allow all packet, N : drop packets whose points less than N 

	// data output
	int output_scan, output_cloud, output_360;
	priv_nh.param("output_scan", output_scan, 1); // 1: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", output_cloud, 0); // 1: enable output xyz format, 0 : disable
	priv_nh.param("output_360", output_360, 1); // 1: packet data of 360 degree (multiple RawData), publish once
							// 0: publish every RawData (36 degree)

	// frame information
       	std::string frame_id;
       	int firmware_number; 
       	priv_nh.param("frame_id", frame_id, std::string("LH_laser")); // could be used for rviz
       	priv_nh.param("firmware_version", firmware_number, 2);

	// output data format
	int mirror, from_zero, angle_patch;
       	priv_nh.param("mirror", mirror, 0); // 0: clockwise, 1: counterclockwise
       	priv_nh.param("from_zero", from_zero, 0); // 1: angle range [0 - 360), 0: angle range [-180, 180)
       	priv_nh.param("angle_patch", angle_patch, 1); // make points number of every fans to unique
       	
	// open serial port
	int fd_uart = -1, fd_udp = -1, fd_tcp = -1;

	if (type == "udp") {
		// open UDP port
		fd_udp  = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(50112);
		addr.sin_addr.s_addr = htonl(INADDR_ANY);

		int rt = ::bind(fd_udp, (struct sockaddr *)&addr, sizeof(addr));
		if (rt != 0)
		{
			ROS_ERROR("bind port failed");
			return -1;
		}

		g_udp_socket = fd_udp;

		// acknowlege device 
		rt = send_cmd_udp(fd_udp, dev_ip.c_str(), udp_port, 0x4753, rand(), 0, NULL);

		// start 
		char cmd[12] = "LGCPSH";
		rt = send_cmd_udp(fd_udp, dev_ip.c_str(), udp_port, 0x0043, rand(), 6, cmd);

	} 
	
	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);
   
	// ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1500);
       
	ros::ServiceServer stop_srv = n.advertiseService("stop_motor", stop_motor);
       	ros::ServiceServer start_srv = n.advertiseService("start_motor", start_motor);

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	RawData* dat360 = new RawData[10];

	memset(dat360, 0, sizeof(RawData)*10);

	bool should_publish = false;

	while (ros::ok()) 
	{ 
		ros::spinOnce();

		if (type == "uart" && fd_uart < 0) 
		{
			// check uart device file
		       	if (access(port.c_str(), R_OK)) 
			{
				ROS_ERROR("port %s not ready", port.c_str());
				ros::Duration(10).sleep();
				continue;
		       	}

			fd_uart = reopen_uart(port, baud_rate, unit_is_mm, with_confidence);
			if (fd_uart < 0) {
				ROS_ERROR("Open port %s error", port.c_str());
				continue;
			}
			g_uart_port = fd_uart;
#if 0
			int nr = 0;
			for (int i=0; i<300 && nr<=0; i++) { 
				usleep(10000);
				nr = read(fd_uart, buf, sizeof(buf));
			}
			if (nr <= 0) {
				ROS_ERROR("serial port seem not working");
				return -1;
			}
#endif

			ROS_INFO("connect to %s", port.c_str());
			//read device's UUID 
			char buf[32];
			if (quirk_talk(fd_uart, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, buf) == 0)
			       	ROS_INFO("get product SN : %s", buf);

			// setup output data format
			if (quirk_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
						10, "SET LiDAR ", 9, buf) == 0)
			       	ROS_INFO("set LiDAR unit to %s", buf);

			// enable/disable output intensity
			if (quirk_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
						6, "LiDAR ", 5, buf) == 0)
				ROS_INFO("set LiDAR confidence to %s", buf);
		}

		if (type == "tcp" && fd_tcp < 0) 
		{
			// open TCP port
			int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (sock < 0) { ROS_ERROR("socket TCP failed"); return 0; }

			struct sockaddr_in addr;
			memset(&addr, 0, sizeof(addr));     /* Zero out structure */
			addr.sin_family      = AF_INET;             /* Internet address family */

			addr.sin_addr.s_addr = inet_addr(dev_ip.c_str());
		       	addr.sin_port = htons(tcp_port);

			int ret = connect(sock, (struct sockaddr *) &addr, sizeof(addr)); 
			
			if (ret != 0) 
			{
				ROS_ERROR("connect (%s:%d) failed", dev_ip.c_str(), tcp_port);
			       	close(sock); 
				//sleep(15);
			       	continue;
		       	}
			fd_tcp = sock;
			ROS_INFO("connect (%s:%d) ok", dev_ip.c_str(), tcp_port);
		}

		fd_set fds;
	       	FD_ZERO(&fds); 

		int fd_max = -1;
		if (fd_uart > 0) 
		{
			FD_SET(fd_uart, &fds); 
			if (fd_max < fd_uart) fd_max = fd_uart;
		}

		if (fd_udp > 0) 
		{
			FD_SET(fd_udp, &fds); 
			if (fd_max < fd_udp) fd_max = fd_udp;
		}

		if (fd_tcp > 0) 
		{
			FD_SET(fd_tcp, &fds); 
			if (fd_max < fd_tcp) fd_max = fd_tcp;
		}
		
		
		struct timeval to = { 5, 1 };

	       	int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			ROS_ERROR("read data timeout");
			if (fd_tcp > 0) { close(fd_tcp); fd_tcp = -1; }
			continue;
		}
		
		if (ret < 0) {
			ROS_ERROR("select error");
			if (fd_tcp > 0) { close(fd_tcp); fd_tcp = -1; }
			if (fd_uart > 0) { close(fd_uart); fd_uart = g_uart_port = -1; }
			continue;
		}

		// read UDP data
		if (fd_udp > 0 && FD_ISSET(fd_udp, &fds)) 
		{ 
			RawData dat;

			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int dw = recvfrom(fd_udp, (char*)&dat, sizeof(dat), 0, (struct sockaddr *)&addr, &sz);

			if (dat.code == 0xface && dat.N * 2 + 8 == dw && (dat.angle % 360) == 0)
			{
				dat360[(dat.angle%3600)/360] = dat;
				if (dat.angle == 3240) {
					should_publish = true;
				}
			}
			else if (dat.code == 0xfacf && dat.N * 3 + 10 == dw && (dat.angle % 360) == 0)
			{

			}
			else {
				char line[256];
				int n = 0;
				unsigned char* buf = (unsigned char*)&dat;
				for (int i=0; i<dw || i<16; i++) {
					n += sprintf(line+n, "%02x ", buf[i]);
				}
				ROS_ERROR("get udp %d bytes : %s", dw, line);
			}
		}

		int new_data = -1;
		// read UART data
		if (fd_uart > 0 && FD_ISSET(fd_uart, &fds)) 
		{
			int nr = read(fd_uart, buf+buf_len, BUF_SIZE - buf_len);
			if (nr <= 0) {
				ROS_ERROR("read port %d error %d", buf_len, nr);
				close(fd_uart);
				fd_uart = g_uart_port = -1;
				continue;
			}
			//if (nr == 0) continue;
			new_data = nr;
		}

		// read TCP data
		if (fd_tcp > 0 && FD_ISSET(fd_tcp, &fds)) 
		{
			int nr = recv(fd_tcp,  buf+buf_len, BUF_SIZE - buf_len, 0);
			if (nr <= 0) {
				ROS_ERROR("tcp error");
				close(fd_tcp);
				fd_tcp = -1;
				continue;
			}

			new_data = nr;
		}

		if (new_data > 0)
		{
			if (fp_dump != NULL) {
				fwrite(buf+buf_len, 1, new_data, fp_dump);
				fflush(fp_dump);
			}
			buf_len += new_data;

			int consume = 0; 
			RawData dat;
			bool is_pack;
			if (unit_is_mm && with_confidence)
			{
				is_pack = parse_data_3(buf_len, buf, dat, consume, with_chk);
			}
			else
			{
				is_pack = parse_data(buf_len, buf, dat, 
						unit_is_mm, with_confidence, consume,
						with_chk);
			}

			if (is_pack && dat.N < normal_size) 
			{
				// drop abnormal packet
				ROS_INFO("abnormal %d : %d points", dat.angle, dat.N);
			}
			else if (is_pack)
			{
				dat360[(dat.angle%3600)/360] = dat;

				if (output_360 != 0) 
				{
					// wait for 360 degree 
				       	if (dat.angle == 3240) should_publish = true;
				} 
				else 
				{ 
					// publish immediately
					sensor_msgs::LaserScan msg; 
					msg.header.stamp = ros::Time::now();
					msg.header.frame_id = frame_id;

					msg.angle_min = dat.angle * M_PI/1800; 
					msg.angle_max = (dat.angle+360) * M_PI/1800; 
					msg.angle_increment = M_PI / 5 / dat.N;

					double scan_time = 1/100.;
					msg.scan_time = scan_time;
					msg.time_increment = scan_time / dat.N;

					msg.range_min = 0.; 
					msg.range_max = 100;//max_distance;//8.0; 
					
					msg.intensities.resize(dat.N); 
					msg.ranges.resize(dat.N);

					for (int j=0; j<dat.N; j++) 
					{
						msg.ranges[j] = dat.distance[j]/1000.0 ; 
						msg.intensities[j] = dat.confidence[j];
					} 
					laser_pub.publish(msg); 
				}
			}

			if (consume > 0) 
			{
				if (!is_pack) {
					FILE* fp = fopen("/tmp/bad.dat", "ab");
					if (fp) {
						fwrite(buf, 1, consume, fp);
						fclose(fp);
					}
					ROS_ERROR("drop %d bytes: %02x %02x %02x %02x %02x %02x", 
							consume,
							buf[0], buf[1], buf[2],
						       	buf[3], buf[4], buf[5]);
				}


				for (int i=consume; i<buf_len; i++) 
					buf[i - consume] = buf[i];
				buf_len -= consume; 
			}
		}

		if (should_publish)
		{
			int N = 0, n = 0;
			for (int i=0; i<10; i++) {
				N += dat360[i].N;
				if (dat360[i].N > 0) n++;
			}

			if (n == 10 && output_cloud != 0) 
			{ 
				sensor_msgs::PointCloud cloud; 
				cloud.header.stamp = ros::Time::now();
				cloud.header.frame_id = frame_id; 
				cloud.points.resize(N);
				cloud.channels.resize(1); 
				cloud.channels[0].name = "intensities"; 
				cloud.channels[0].values.resize(N);
  
				int idx = 0;
				for (int j=0; j<10; j++) 
				{
					for (int i=0; i<dat360[j].N; i++) 
					{
						float r = dat360[j].distance[i]/1000.0 ; 
						float a = j*PI/5 + i*PI/5/dat360[j].N;
						cloud.points[idx].x = cos(a) * r;
						cloud.points[idx].y = sin(a) * r;
						cloud.points[idx].z = 0;
					       	cloud.channels[0].values[idx] = 1 + dat360[j].confidence[j];
						idx++;
					}
				} 
				cloud_pub.publish(cloud);
				// printf("cloud published\n");
			}
			if (n == 10 && angle_patch != 0) 
			{
				int mx = 0;
				for (int i=0; i<10; i++) 
				{
					if (mx < dat360[i].N) mx = dat360[i].N;
				}
				       	       
				for (int i=0; i<10; i++) 
				{
					for (int j=dat360[i].N; j<mx; j++) 
					{
						dat360[i].distance[j] = 0;
						//printf("patch [%d] %d\n", i, j);
					}
					dat360[i].N = mx;
				}
				N = mx * 10;
			}

			if (n == 10 && output_scan != 0) 
			{
				sensor_msgs::LaserScan msg; 
			
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = frame_id;

				if (from_zero == 0) {
					msg.angle_min = 0; 
					msg.angle_max = M_PI*2*(N-1)/N; 
				}
				else {
					msg.angle_min = -M_PI;
					msg.angle_max = M_PI - M_PI*2/N;
				}
				msg.angle_increment = M_PI*2 / N;

				double scan_time = 1/10.;
				msg.scan_time = scan_time;
				msg.time_increment = scan_time / N;

				msg.range_min = 0.; 
				msg.range_max = 100;//max_distance;//8.0; 
				
				msg.intensities.resize(N); 
				msg.ranges.resize(N);

				N = 0;
				if (mirror == 0) for (int j=0; j<10; j++) 
				{
					for (int i=0; i<dat360[j].N; i++) 
					{
						msg.ranges[N] = dat360[j].distance[i]/1000.0 ; 
						msg.intensities[N] = dat360[j].confidence[i];
						N++;
					}
				} 
				else for (int jj=0; jj<10; jj++) 
				{
					int j = from_zero ? jj : ((jj+5) % 10);
					
					int idx[10] = { 4, 3, 2, 1, 0, 9, 8, 7, 6, 5 };
					int id = idx[j];
					int cnt = dat360[ id ].N;
					for (int i=0; i<cnt; i++) 
					{
						msg.ranges[N] = dat360[id].distance[cnt-1-i] /1000.0 ; 
						msg.intensities[N] = 1 + dat360[id].confidence[cnt-1-i];
						N++;
					}
				}

				laser_pub.publish(msg); 

#if 0
				static timeval last;
				timeval now;
				gettimeofday(&now, NULL);
				printf("published %ld\n", (now.tv_sec - last.tv_sec)*1000 + now.tv_usec/1000 - last.tv_usec/1000);
				last = now;
#endif
			}

			
			should_publish = false;
		}

	}

	//close(fd);
       	return 0;
}

