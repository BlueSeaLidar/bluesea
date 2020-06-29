
/*********************************************************************
 * This is demo for ROS refering to xv_11_laser_driver.
 * serial port version :
 rosrun bluesea bluesea_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=1 _output_cloud:=1 _unit_is_mm:=1 _with_confidence:=1
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

int udp_talk(int fd_udp, 
		const char* dev_ip, int dev_port,
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

#if 1
	char s[3096];
	for (int i = 0; i < len2; i++) 
		sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);
	printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n", 
			dev_ip, dev_port, cmd, sn, len, s);
#endif

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

char g_uuid[32] = "";
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
	
		if (idx > 24) for (int i=0; i<idx-22; i++) 
		{
			// get product SN
			if (memcmp(buf+i, "PRODUCT SN: ", 12) == 0)
		       	{
				memcpy(g_uuid, buf+i+12, 9);
				g_uuid[9] = 0;
				printf("found product SN : %s\n", g_uuid);
			}
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			ROS_ERROR("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
		{
			ROS_ERROR("points number %d seem not correct\n", hdr.N);
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
			ROS_ERROR("chksum error");
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
			
		if (idx > 24) for (int i=0; i<idx-22; i++) 
		{
			// get product SN
			if (memcmp(buf+i, "PRODUCT SN: ", 12) == 0)
		       	{
				memcpy(g_uuid, buf+i+12, 9);
				g_uuid[9] = 0;
				printf("found product SN : %s\n", g_uuid);
			}
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE);

		if ((hdr.angle % 360) != 0) 
		{
			ROS_ERROR("bad angle %d\n", hdr.angle);
			idx += 2;
			continue; 
		}

		if (hdr.N > 300 || hdr.N < 30) 
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
		//printf("%d+%d, ", hdr.angle, hdr.N);

		idx += HDR_SIZE + 2*hdr.N + 2;
		consume = idx;
		return true;
	}

	if (idx > 1024) consume = idx/2;
	return false;
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

int quirk_talk(int fd, int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	printf("send command : %s\n", cmd);
	write(fd, cmd, n);
			
	char buf[1024];
	int nr = read(fd, buf, sizeof(buf));

	while (nr < (int)sizeof(buf))
	{
		int n = read(fd, buf+nr, sizeof(buf)-nr);
		if (n > 0) nr += n;
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea_laser_publisher");
       	ros::NodeHandle n;
       	ros::NodeHandle priv_nh("~");

	std::string port, type, dev_ip;
       	int baud_rate, udp_port, tcp_port;
	int mirror, from_zero;
	int unit_is_mm, with_confidence;
	int angle_patch;
	int output_scan, output_cloud;
       	std::string frame_id;
       	int firmware_number; 
	int with_chk;
	
	std_msgs::UInt16 rpms; 

	priv_nh.param("type", type, std::string("uart"));
	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("dev_ip", dev_ip, std::string("192.168.158.91"));
	priv_nh.param("output_scan", output_scan, 1);
	priv_nh.param("output_cloud", output_cloud, 0);
	priv_nh.param("udp_port", udp_port, 5000);
	priv_nh.param("tcp_port", tcp_port, 5000);
       	priv_nh.param("baud_rate", baud_rate, 256000);
       	priv_nh.param("frame_id", frame_id, std::string("LH_laser"));
       	priv_nh.param("firmware_version", firmware_number, 2);
       	priv_nh.param("mirror", mirror, 0);
       	priv_nh.param("from_zero", from_zero, 0);
       	priv_nh.param("angle_patch", angle_patch, 1);
       	priv_nh.param("unit_is_mm", unit_is_mm, 1);
       	priv_nh.param("with_confidence", with_confidence, 1);
       	priv_nh.param("with_checksum", with_chk, 1);

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

		// acknowlege device 
		//rt = udp_talk(fd_udp, dev_ip.c_str(), udp_port, 0x4753, rand(), 0, NULL);

		//char cmd[12] = "LGCPSH";
		//rt = udp_talk(fd_udp, dev_ip.c_str(), udp_port, 0x0043, rand(), 6, cmd);

	} 
	else if (type == "uart") {
		fd_uart = open_serial_port(port.c_str(), baud_rate);
		if (fd_uart < 0) {
			ROS_ERROR("Open port error ");
			return -1;
		}
		g_port = fd_uart;

		//send UUID reading request 
		//char buf[] = "LUUIDH";
		//write(g_port, buf, strlen(buf));
		char buf[32];
		int nr = 0;
		for (int i=0; i<300 && nr<=0; i++) { 
			usleep(10000);
			nr = read(fd_uart, buf, sizeof(buf));
		}
		if (nr <= 0) {
			printf("serial port seem not working\n");
			return -1;
		}

		if (quirk_talk(fd_uart, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid) == 0)
	       	{
		       	printf("get product SN : %s\n", g_uuid);
	       	}

		if (quirk_talk(fd_uart, 6, unit_is_mm == 0 ? "LMDCMH" : "LMDMMH", 
					10, "SET LiDAR ", 9, buf) == 0)
		{
			printf("set LiDAR unit to %s\n", buf);
		}

		if (quirk_talk(fd_uart, 6, with_confidence == 0 ? "LNCONH" : "LOCONH", 
					6, "LiDAR ", 5, buf) == 0)
		{
			printf("set LiDAR confidence to %s\n", buf);
		}
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
		if (type == "tcp" && fd_tcp < 0) 
		{
			// open TCP port
			int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (sock < 0) { ROS_ERROR("socket TCP failed\n"); return 0; }

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
				sleep(15);
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
		
		
		struct timeval to = { 1, 1 };
	       	int ret = select(fd_max+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			ROS_ERROR("read data timeout");
			if (fd_tcp > 0) {
				close(fd_tcp);
				fd_tcp = -1;
			}
			continue;
		}
		
		if (ret < 0) {
			ROS_ERROR("select error");
			return -1;
		}

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
			else {
				printf("get udp %d bytes :", dw);
				unsigned char* buf = (unsigned char*)&dat;
				for (int i=0; i<dw || i<16; i++) {
					printf("%02x ", buf[i]);
				}
				printf("\n");
			}
		}

		int new_data = -1;
		if (fd_uart > 0 && FD_ISSET(fd_uart, &fds)) 
		{
			int nr = read(fd_uart, buf+buf_len, BUF_SIZE - buf_len);
			if (nr <= 0) {
				ROS_ERROR("read port %d error %d", buf_len, nr);
				break;
			}

			if (nr == 0) continue;
			new_data = nr;
		}

		if (fd_tcp > 0 && FD_ISSET(fd_tcp, &fds)) 
		{
			int nr = recv(fd_tcp,  buf+buf_len, BUF_SIZE - buf_len, 0);
			if (nr < 0) {
				ROS_ERROR("tcp error");
				close(fd_tcp);
				fd_tcp = -1;
				continue;
			}

			new_data = nr;
		}

		if (new_data > 0)
		{
			buf_len += new_data;

			int consume = 0; 
			RawData dat;
			bool is_pack;
			if (unit_is_mm && with_confidence)
				is_pack = parse_data_3(buf_len, buf, dat, consume, with_chk);
			else
				is_pack = parse_data(buf_len, buf, dat, 
						unit_is_mm, with_confidence, consume,
						with_chk);
			if (is_pack)
			{
				dat360[(dat.angle%3600)/360] = dat;

				if (dat.angle == 3240)
				{
					should_publish = true;
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


		ros::spinOnce();
	}

	//close(fd);
       	return 0;
}

