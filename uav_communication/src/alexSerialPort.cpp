#include "alexSerialPort.h"
#include <ros/ros.h>
using namespace std;

static int serial_fd = -1; //串口文件描述符

int SerialClose()
{
  if(serial_fd < 0) return serial_fd;
  close(serial_fd);
  serial_fd = -1;
  return 0;
}

int SerialFlush()
{
  if(serial_fd < 0)
  {
    ROS_ERROR("SerialFlush ERROR\n");
    return -1;
  }
  else
  {
    tcflush(serial_fd, TCIFLUSH);
    return 0;
  }
}

int SerialConfig(int baudrate, char data_bits, char parity_bits, char stop_bits)
{
	int st_baud[]=
	{
		B4800,
		B9600,
		B19200,
		B38400,
		B57600,
		B115200,
		B230400,
		B1000000,
		B1152000,
		B3000000,
	};
        
	int std_rate[]=
	{
		4800,
		9600,
		19200,
		38400,
		57600,
		115200,
		230400,
		1000000,
		1152000,
		3000000,
	};

	struct termios newtio, oldtio;

    //存储原始结构
	if (tcgetattr(serial_fd, &oldtio) != 0)
	{
		ROS_ERROR("tcgetattr ERROR\n");
		return -1;
	}
    
	bzero(&newtio, sizeof(newtio));

	//使能串口接收
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	//设置串口数据位
	if (data_bits == 7)         newtio.c_cflag |= CS7;
    else if (data_bits == 8)    newtio.c_cflag |= CS8;

	//配置奇偶校验位
	switch (parity_bits)
	{
		/* odd */
	case 'O':
	case 'o':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		break;
		/* even */
	case 'E':
	case 'e':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
		/* none */
	case 'N':
	case 'n':
		newtio.c_cflag &= ~PARENB;
		break;
	}
    
    //配置波特率
	size_t n = sizeof(std_rate)/sizeof(std_rate[0]);
	for(size_t i = 0; i < n; i++)
	{
        if(std_rate[i] != baudrate) continue;
		/* set standard baudrate */
		cfsetispeed(&newtio, st_baud[i]);
		cfsetospeed(&newtio, st_baud[i]);
	}
    
	//设置停止位
    if( stop_bits == 1 )        newtio.c_cflag &=  ~CSTOPB;
    else if ( stop_bits == 2 )  newtio.c_cflag |=  CSTOPB;

    //一个串口是阻塞状态的时候便可以设置它为超时状态，这里超时时间设为1秒
    newtio.c_cc[VTIME]  = 10;    //非规范模式读取时的超时时间(单位:百毫秒)
    newtio.c_cc[VMIN] = 0;      //非规范模式读取时的最小字符数
                                //如需需要设置超时则c_cc[VMIN] 必须等于0
                                //这代表能够读取的最小字符是0个，即使用read读取数据超时read返回0

    //raw模式
    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag  &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd, TCIFLUSH);

    /* activite the configuration */
    if (tcsetattr(serial_fd, TCSANOW, &newtio) != 0)
    {
    	printf("tcsetattr ERROR\n");
    	return -1;
    }
    
    return 0;
}

static int SerialSetup(const char *dev_name, int baud_rate)
{
  if(!dev_name) dev_name = "/dev/ttyUSB0";
  ROS_INFO("dev_name:%s\n", dev_name);
  
  //open serial
  serial_fd = open(dev_name, O_RDWR | O_NOCTTY); //阻塞模式
  if(serial_fd < 0)
  {
    ROS_ERROR("SerialOpen ERROR\n");
    return serial_fd;
  }
  if(SerialConfig(baud_rate, 8, 'N', 1) < 0)
  {
    ROS_ERROR("SerialConfig ERROR\n");
    close(serial_fd);
    return -1;
  }
  
  ROS_INFO("SerialStart ok, serial_fd: %d ", serial_fd);
  return 0;
}

static int SerialWrite(const char *buf, size_t len)
{
	return write(serial_fd, buf, len);
}

static int SerialRead(char *buf, size_t len)
{
    return read(serial_fd, buf, len);
}

int Alex_SerialPort_Close()
{
    SerialClose();
}

int Alex_SerialPort_BufferClear()
{
    return SerialFlush();
}

void ALex_SerialPort_ReadBufferSize(size_t *size)
{
    ioctl(serial_fd, FIONREAD, size);
}

int Alex_SerialPort_Send(const char *buf, size_t len)
{	
    return SerialWrite(buf, len);
}

int Alex_SerialPort_Recv(char *buf, size_t len)
{
	return SerialRead(buf, len);
}

int Alex_SerialPort_Setup(const char *device, int baudrate)
{
	return (SerialSetup(device, baudrate) < 0) ? -1: 0;
}
