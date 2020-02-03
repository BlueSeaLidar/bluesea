/*********************************************************************
 *This is demo for ROS refering to xv_11_laser_driver.
 rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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
//#include <linux/termios.h>

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
	unsigned short data[1000];
};

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
int g_port = -1;

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

// send command to lidar serial port
int send_cmd(unsigned short cmd_id, int len, const void* cmd_body)
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

	return write(g_port, buffer, len + sizeof(CmdHeader) + 4);
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
	if (baudrate == 115200)
		speed = B115200;

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
	return fd;
}

// translate lidar raw data to ROS laserscan message
bool parse_data(int len, unsigned char* buf, RawData& dat, int& consume) 
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

		memcpy(&dat, &hdr, HDR_SIZE);
		memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);

		idx += HDR_SIZE + 2*hdr.N + 2;
		consume = idx;
		return true;
	}
	if (idx > 1024) consume = idx/2;
	return false;
}

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

// service call back function
bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	send_cmd(0x0043, 6, "LSTOPH");
	ROS_INFO("Stop motor");
       	return true;
}

// service call back function
bool start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	send_cmd(0x0043, 6, "LSTARH");
	ROS_INFO("Start motor");
       	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea_laser_publisher");
       	ros::NodeHandle n;
       	ros::NodeHandle priv_nh("~");

	std::string port;
       	int baud_rate;
       	std::string frame_id;
       	int firmware_number; 
	
	std_msgs::UInt16 rpms; 

	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
       	priv_nh.param("baud_rate", baud_rate, 256000);
       	priv_nh.param("frame_id", frame_id, std::string("LH_laser"));
       	priv_nh.param("firmware_version", firmware_number, 2);

#if 1
	// open serial port
	int fd = open_serial_port(port.c_str(), baud_rate);
	if (fd < 0) {
		ROS_ERROR("Open port error ");
		return -1;
	}
	g_port = fd;
#endif

	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1500);
	// ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1500);
       
	ros::ServiceServer stop_srv = n.advertiseService("stop_motor", stop_motor);
       	ros::ServiceServer start_srv = n.advertiseService("start_motor", start_motor);

	unsigned char* buf = new unsigned char[BUF_SIZE];
	int buf_len = 0;

	RawData* dat360 = new RawData[10];

	memset(dat360, 0, sizeof(RawData)*10);

	while (ros::ok()) 
	{ 
		fd_set fds;
	       	FD_ZERO(&fds); 
		FD_SET(fd, &fds); 
		
		struct timeval to = { 0, 2000 };
	       	int ret = select(fd+1, &fds, NULL, NULL, &to); 
		
		if (ret < 0) {

			ROS_ERROR("select error");
			return -1;
		}

		if (FD_ISSET(fd, &fds)) 
		{
			int nr = read(fd, buf+buf_len, BUF_SIZE - buf_len);
			if (nr <= 0) {
				ROS_ERROR("read port %d error %d", buf_len, nr);
				break;
			}

			if (nr == 0) continue;

			buf_len += nr;


			int consume = 0; 
			RawData dat;
			bool is_pack = parse_data(buf_len, buf, dat, consume);
			if (is_pack)
			{
				dat360[(dat.angle%3600)/360] = dat;

				if (dat.angle == 3240)
				{

					int N = 0, n = 0;
					for (int i=0; i<10; i++) {
						N += dat360[i].N;
						if (dat360[i].N > 0) n++;
					}
					if (n == 10) 
					{
						sensor_msgs::LaserScan msg; 
					
						msg.header.stamp = ros::Time::now();
					       	msg.header.frame_id = frame_id;

						msg.angle_min = 0; 
						msg.angle_max = M_PI*2; 
						msg.angle_increment = M_PI*2 / N;

						double scan_time = 1/5.;
					       	msg.scan_time = scan_time;
					       	msg.time_increment = scan_time / N;

						msg.range_min = 0.; 
						msg.range_max = 100;//max_distance;//8.0; 
						
						msg.intensities.resize(N); 
						msg.ranges.resize(N);

						N = 0;
						for (int j=0; j<10; j++) 
						{
							for (int i=0; i<dat360[j].N; i++) 
							{
								msg.ranges[N] = ( dat360[j].data[i] & 0x1FFF )/100.0 ; 
								msg.intensities[N] = 1 + (float) (dat360[j].data[i] >> 13);
								N++;
						       	}
						} 
						laser_pub.publish(msg); 
					}
				}
			}

			if (consume > 0) 
			{
				for (int i=consume; i<buf_len; i++) 
					buf[i - consume] = buf[i];
				buf_len -= consume; 

				if (!is_pack) ROS_ERROR("drop %d bytes", consume);
			}
		}
		ros::spinOnce();
	}

	//close(fd);
       	return 0;
}

