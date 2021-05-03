#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <iostream>
#include <queue>

using namespace std;

#pragma pack(1)
typedef struct _TF40TX {
	unsigned char id;
	unsigned char function;
	unsigned short address;
	unsigned short quantity;
	unsigned short crc;
} TF40TX;

#define RX_ID_IDX 0
#define RX_FUNC_IDX 1
#define RX_LEN_IDX 2
#define RX_VAL_IDX 3
#define RX_CRC_IDX 7

#define RX_ID_SIZE 1
#define RX_FUNC_SIZE 1
#define RX_LEN_SIZE 1
#define RX_VAL_SIZE 4
#define RX_CRC_SIZE 2

#define RX_ID_VAL 0x01
#define RX_FUNC_VAL 0x03
#define RX_LEN_VAL 0x04

#pragma pack(1)
typedef struct _TF40RX {
	unsigned char id;
	unsigned char function;
	unsigned char length;
	unsigned short register1;
	unsigned short register2;
	unsigned short crc;
} TF40RX;

std::string node_name;

//Define constants
const static int TX_SIZE = sizeof(TF40TX);
const static int RX_SIZE = sizeof(TF40RX);

unsigned short CRC16(unsigned char data[], unsigned short length);

class TF40ForROS
{
private:
	ros::NodeHandle nh_;

	ros::Publisher sensor_range;

	pthread_mutex_t lock_;

	std::string serial_port_;
	// kajuha 20210302
	int baud_rate_;
	queue<unsigned char> que;

	std::string frame_id_;

	//Define global variables
	int fd;
	unsigned char tx_packet[TX_SIZE];
	unsigned char rx_packet[BUFSIZ];
	unsigned char tmp_packet[RX_SIZE];
	int count = 0;


public:
	TF40ForROS(std::string serial_port, int baud_rate)
		: nh_("~")
	{
		// dependent on user device
		nh_.setParam("serial_port", serial_port);
		nh_.setParam("baud_rate", baud_rate);

  		ros::param::get("~frame_id", frame_id_);

		serial_port_ = serial_port;
		// kajuha 20210302
		baud_rate_ = baud_rate;
		
		// publisher for streaming
		// sensor_range = nh_.advertise<sensor_msgs::Range>("sensor_range/data", 1);
		sensor_range = nh_.advertise<sensor_msgs::Range>(frame_id_, 1000, true);
	}

	~TF40ForROS()
	{}

	bool initialize()
	{
		const char* COMM_PORT = serial_port_.c_str();
		
		if(-1 == (fd = open(COMM_PORT, O_RDWR)))
		{
			cout << node_name << endl;
			cout << "Error opening port \n";
			cout << "Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 38400 raw\n";
			cout << "You may need to have ROOT access";
			return false;
		}

		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		// kajuha 20210302
		#if 1
		switch(baud_rate_)
		{
			case 921600:
				newtio.c_cflag = B921600;
				break;
			case 576000:
				newtio.c_cflag = B576000;
				break;
			case 500000:
				newtio.c_cflag = B500000;
				break;
			case 460800:
				newtio.c_cflag = B460800;
				break;
			case 230400:
				newtio.c_cflag = B230400;
				break;
			case 115200:
				newtio.c_cflag = B115200;
				break;
			case 57600:
				newtio.c_cflag = B57600;
				break;
			case 38400:
				newtio.c_cflag = B38400;
				break;
			case 19200:
				newtio.c_cflag = B19200;
				break;
			case 9600:
				newtio.c_cflag = B9600;
				break;
			case 4800:
				newtio.c_cflag = B4800;
				break;
			default:
				ROS_ERROR("%s : Unsupported baud rate!", node_name.c_str());
				exit(0);
		}
		#else
		newtio.c_cflag = B38400;
		#endif
		newtio.c_cflag |= CS8;
		newtio.c_cflag |= CLOCAL;
		newtio.c_cflag |= CREAD;
		newtio.c_iflag = 0;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0; 
		// kajuha 20210302, receive FSM abnormal, require modified code
		#if 0
		newtio.c_cc[VMIN] = 1; 
		#else
		newtio.c_cc[VMIN] = 0;
		#endif

		tcflush(fd, TCIOFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);

		cout << node_name << " : TF40 communication port is ready\n";

		lock_ = PTHREAD_MUTEX_INITIALIZER;

		return true;
	}

	void closeSensor()
	{
		close(fd);
		cout << node_name << " : Closing TF40 Sensor" << endl;
	}

	bool requestStreamData()
	{
		TF40TX tf40tx;
		tf40tx.id = 0x01;
		tf40tx.function = 0x03;
		tf40tx.address = 0x0100;
		tf40tx.quantity = 0x0200;
		tf40tx.crc = 0xCB95;

		write(fd, (char*)&tf40tx, TX_SIZE);
	}

	bool requestData()
	{
		TF40TX tf40tx;
		tf40tx.id = 0x01;
		tf40tx.function = 0x03;
		tf40tx.address = 0x0F00;
		tf40tx.quantity = 0x0200;
		tf40tx.crc = 0x08F4;

		write(fd, (char*)&tf40tx, TX_SIZE);
	}

	bool stopData()
	{
		TF40TX tf40tx;
		tf40tx.id = 0x01;
		tf40tx.function = 0x03;
		tf40tx.address = 0x0A00;
		tf40tx.quantity = 0x0200;
		tf40tx.crc = 0x09E4;

		write(fd, (char*)&tf40tx, TX_SIZE);
	}

	bool changeBaudrate115200()
	{
		unsigned char baud_rate_packet[] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x01, 0xC2, 0x00, 0xF3, 0x0F};

		write(fd, baud_rate_packet, sizeof(baud_rate_packet));
	}

	bool receiveData()
	{
		// kajuha 20210302
		#if 1
		int rx_size;

		memset(rx_packet, '\0', sizeof(rx_packet));

		rx_size = read(fd, rx_packet, BUFSIZ);

		for (int i=0; i<rx_size; i++) {
			que.push(rx_packet[i]);
		}

		return true;
		#else
		int rx_size;
		short header;
		short check_sum;
		
		// pthread_mutex_lock(&lock_);
	
		memset(rx_packet, '\0', sizeof(rx_packet));

		rx_size = read(fd, rx_packet, RX_ID_SIZE);
		if (rx_packet[0] != RX_ID_VAL) {
			cout << "Address Not Match !!!" << endl;
			return false;
		}

		rx_size = read(fd, rx_packet, RX_FUNC_SIZE);
		if (rx_packet[0] != RX_FUNC_VAL) {
			cout << "Function Not Match !!!" << endl;
			return false;
		}

		rx_size = read(fd, rx_packet, RX_LEN_SIZE);
		if (rx_packet[0] != RX_LEN_VAL) {
			cout << "Byte Count Not Match !!!" << endl;
			return false;
		}

		rx_size = read(fd, tmp_packet, RX_SIZE-RX_ID_SIZE-RX_FUNC_SIZE-RX_LEN_SIZE);
		memcpy(rx_packet+RX_ID_SIZE+RX_FUNC_SIZE+RX_LEN_SIZE, tmp_packet, RX_SIZE-RX_ID_SIZE-RX_FUNC_SIZE-RX_LEN_SIZE);
		rx_packet[0] = RX_ID_VAL;
		rx_packet[1] = RX_FUNC_VAL;
		rx_packet[2] = RX_LEN_VAL;
		unsigned short crc16 = CRC16(rx_packet, RX_SIZE-RX_CRC_SIZE);
		unsigned char crc16hi = *(((char*)&crc16)+0);
		unsigned char crc16lo = *(((char*)&crc16)+1);

		if (!(crc16hi == rx_packet[RX_SIZE-2] && crc16lo == rx_packet[RX_SIZE-1])) {
			cout << "CRC Not Match !!!" << endl;
			return false;
		}

		publishTopic();

		// pthread_mutex_unlock(&lock_);

		return true;
		#endif
	}

	// kajuha 20210302
	enum STATE {ADDRESS, FUNCTION, COUNT, VALUE, CRC, OK};
	bool parseData() {
		static int state = ADDRESS;
		static unsigned char packet[sizeof(TF40RX)] = {'\0', };
		int rx_size;
		short header;
		short check_sum;

		switch (state) {
			case ADDRESS:
				// size check
				if (que.size() >= RX_ID_SIZE) {
					// valid check
					if (que.front() == RX_ID_VAL) {
						packet[RX_ID_IDX] = que.front();

						state = FUNCTION;
					} else {
						printf("%s : ID not Match \n", node_name.c_str());
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case FUNCTION:
				// size check
				if (que.size() >= RX_FUNC_SIZE) {
					// valid check
					if (que.front() == RX_FUNC_VAL) {
						packet[RX_FUNC_IDX] = que.front();

						state = COUNT;
					} else {
						printf("%s : FUNCTION not Match : 0x%02x \n", node_name.c_str(), que.front());
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case COUNT:
				// size check
				if (que.size() >= RX_LEN_SIZE) {
					// valid check
					if (que.front() == RX_LEN_VAL) {
						packet[RX_LEN_IDX] = que.front();

						state = VALUE;
					} else {
						printf("%s : LENGTH not Match \n", node_name.c_str());
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case VALUE:
				// size check
				if (que.size() >= RX_VAL_SIZE) {
					for (int i=0; i<RX_VAL_SIZE; i++) {
						packet[RX_VAL_IDX+i] = que.front();
						que.pop();
					}

					state = CRC;
				}
				break;
			case CRC:
				// size check
				if (que.size() >= RX_CRC_SIZE) {
					for (int i=0; i<RX_CRC_SIZE; i++) {
						packet[RX_CRC_IDX+i] = que.front();
						que.pop();
					}

					unsigned short crc16 = CRC16(packet, RX_SIZE-RX_CRC_SIZE);
					unsigned char crc16hi = *(((char*)&crc16)+0);
					unsigned char crc16lo = *(((char*)&crc16)+1);

					if (!(crc16hi == packet[RX_SIZE-2] && crc16lo == packet[RX_SIZE-1])) {
						cout << node_name << " : CRC Not Match !!!" << endl;
						memset(packet, '\0', RX_SIZE);
					
						state = ADDRESS;

						return false;
					} else {
						state = OK;
					}
				}

				break;
			case OK:
				publishTopic(packet);

#if 0
				printf("%s : ", node_name.c_str());
				for (int i=0; i<RX_SIZE; i++) {
					printf("%02x ", packet[i]);
				}
				printf("\n");
#endif

				memset(packet, '\0', RX_SIZE);
				
				state = ADDRESS;

				return true;

				break;
			default:
				state = ADDRESS;

				break;
		}
		
		return false;
	}

	unsigned short byteExchange(unsigned short data) {
		return ((data & 0x00FF) << 8) | ((data & 0xFF00) >> 8);
	}

	void publishTopic(unsigned char* packet)
	{
		TF40RX tf40rx;

		memcpy(&tf40rx, packet, sizeof(tf40rx));
		unsigned int range = byteExchange(tf40rx.register1) << 16 | byteExchange(tf40rx.register2);
		
		static int count = 0;
		printf("range : %d mm, %d cnt\r\n", range, count++);

		// cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << tf40rx.AccX << endl;
	
		// Publish ROS msgs.
		sensor_msgs::Range sensor_range_msg;


		ros::Time now = ros::Time::now();

		// timestamp
		sensor_range_msg.header.stamp = now;
		
		// frame id
		sensor_range_msg.header.frame_id = frame_id_;
		
		// radiation_type
		sensor_range_msg.radiation_type = sensor_msgs::Range::INFRARED;

		// fov(rad)
		// sensor_range_msg.field_of_view = 0.001; // original
		sensor_range_msg.field_of_view = 0.01;	// visulation

		// min, max range
		sensor_range_msg.min_range = 0.04;
		sensor_range_msg.max_range = 40;

		// range
		sensor_range_msg.range = range / 1000.0;

		// publish the IMU data
		sensor_range.publish(sensor_range_msg);
	}

	int getQueueSize() {
		return que.size();
	}

};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf40");

  std::string serial_port;
  int baud_rate;

  ros::param::get("~serial_port", serial_port);
  ros::param::get("~baud_rate", baud_rate);
  ros::param::get("~node_name", node_name);

  TF40ForROS sensor(serial_port, baud_rate);

  if(sensor.initialize() == false)
  {
    ROS_ERROR("%s\n", node_name.c_str());
    ROS_ERROR("Initialize() returns false, please check your devices.\n");
	ROS_ERROR("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 38400 raw\n");
	ROS_ERROR("You may need to have ROOT access\n");
    return 0;
  }
  else
  {
    ROS_INFO("%s\n", node_name.c_str());
    ROS_INFO("TF40 Initialization OK!\n");
  }

  ros::Rate r(100);

#define STEP_TIME 1.0
  double time_cur = ros::Time::now().toSec();
  double time_pre = time_cur;
  double time_diff;

  bool result;

#if 0
  sensor.changeBaudrate115200();
#endif

  while (ros::ok())
  {
    time_cur = ros::Time::now().toSec();
    time_diff = time_cur - time_pre;
    if ( time_diff > STEP_TIME ) {
    	#if 0
  		sensor.requestData();
  		#else
  		static bool request = true;
  		if (request) {
  			if (result == true) {
  				request = false;
  				printf("%s disable request data\n", node_name.c_str());
  			}

  			sensor.requestStreamData();
  		}
  		#endif
        time_pre = time_cur;
    }

	sensor.receiveData(); // stack smashing detected(because of BUFSIZ, read(BUFSIZ))
	while( sensor.getQueueSize() >= RX_SIZE ) {
		result = sensor.parseData();

		if (result) {
			break;
		}
	}
	// printf("%s : size(%d) \n", node_name.c_str(), sensor.getQueueSize());

	ros::spinOnce();

	r.sleep();
  }

  //ros::spin();

  return 0;
}

/* CRC upper byte value table */
const unsigned char CRCHiBytes[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

/* CRC lower byte value table */
const unsigned char CRCLoBytes[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

unsigned short CRC16(unsigned char data[], unsigned short length) {
	unsigned char HiByte = 0xFF;	// Initialize higher CRC byte
	unsigned char LoByte = 0xFF;	// Initialize lower CRC byte
	unsigned short idx;	// Index in the table

	while (length--) {
		idx = LoByte ^ *data++;
		// Calculate CRC
		LoByte = HiByte ^ CRCHiBytes[idx];
		HiByte = CRCLoBytes[idx];
	}

	return (HiByte << 8 | LoByte);
}