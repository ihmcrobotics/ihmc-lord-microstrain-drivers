#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <stdint.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sched.h>
#include <sys/mman.h>

using namespace std;

typedef enum
{
	HEADER, HEADER_B, LENGTH, FIELDS, CHECKSUM
} state;

#define REALTIME_PRIORITY 90
#define MAX_SAFE_STACK (32 * 1024)

#define MAXIMUM_PACKET_LENGTH 262
#define HEADER_BYTE_1 0x75
#define HEADER_BYTE_2 0x65

std::string device;
std::string host;
std::string port;

int fd;
int sendSocket;

state readState = HEADER;

void stack_prefault(void)
{

	unsigned char dummy[MAX_SAFE_STACK];

	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}

void usage()
{
	cout << "Usage: " << endl << "-d: Serial device" << endl << "-h: Target host" << endl << "-p: Target port" << endl;
}

int getSendSock(std::string port, std::string host)
{
	struct addrinfo hints, *res, *ai;
	int p;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE;
	hints.ai_protocol = IPPROTO_UDP;
	getaddrinfo(host.c_str(), port.c_str(), &hints, &res);

	for (ai = res; ai != NULL; ai = ai->ai_next)
	{
		if ((p = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol)) < 0)
		{
			perror("Socket");
			continue;
		}

		if (connect(p, ai->ai_addr, ai->ai_addrlen) == -1)
			perror("Connect:");
		return p;
	}

	return 0;
}

void openSocket()
{
	sendSocket = getSendSock(port, host);
	if (sendSocket == 0)
	{
		cerr << "Cannot open UDP port to " << host << ":" << port << endl;
		exit(-1);
	}
}

void openPort()
{
	fd = open(device.c_str(), O_RDWR | O_SYNC | O_NOCTTY, S_IRUSR | S_IWUSR);
	if (fd < 0)
	{
		switch (errno)
		{
			case EACCES:
				cerr << "Cannot open " << device << ", check permissions" << endl;
				break;
			case ENOENT:
				cerr << "Cannot open " << device << ", port does not exists" << endl;
				break;
			default:
				cerr << "An error occurred when opening " << device << endl;
				break;
		}
		exit(-1);
	}

	struct flock lock;
	lock.l_type = F_WRLCK;
	lock.l_whence = SEEK_SET;
	lock.l_start = 0;
	lock.l_len = 0;
	lock.l_pid = getpid();

	if (fcntl(fd, F_SETLK, &lock) != 0)
	{
		cerr << "Cannot lock " << device << endl;
		exit(-1);
	}

	struct termios term;
	if (tcgetattr(fd, &term) < 0)
	{
		cerr << "Unable to get serial port attributes." << endl;
		exit(-1);
	}

	cfmakeraw(&term);
	cfsetispeed(&term, B115200);
	cfsetospeed(&term, B115200);

	if (tcsetattr(fd, TCSAFLUSH, &term) < 0)
	{
		cerr << "Unable to set serial port attributes" << endl;
		exit(-1);
	}

	if (tcflush(fd, TCIOFLUSH) != 0)
	{
		cerr << "tcflush failed" << endl;
		exit(-1);
	}
}

void sendUDP(uint8_t* buffer, size_t length)
{
	if(send(sendSocket, buffer, length, 0) == -1)
	{
		perror("Send");
	}
}

void validateAndSend(uint8_t* buffer, size_t length)
{
	uint8_t checksum_byte1 = 0;
	uint8_t checksum_byte2 = 0;
	for (size_t i = 0; i < length - 2; i++)
	{
		checksum_byte1 += buffer[i];
		checksum_byte2 += checksum_byte1;
	}

	if (checksum_byte1 == buffer[length - 2] && checksum_byte2 == buffer[length - 1])
	{
		sendUDP(buffer, length);
	}
	else
	{
		cout << "Invalid packet, dropping!" << endl;
	}
}

void read()
{

	uint8_t buffer[MAXIMUM_PACKET_LENGTH];
	uint8_t b;
	uint8_t payload;
	do
	{
		switch (readState)
		{
			case HEADER:
				memset(buffer, 0, MAXIMUM_PACKET_LENGTH);
				b = read(fd, buffer, 1);
				if (b > 0 && buffer[0] == HEADER_BYTE_1)
				{
					readState = HEADER_B;
				}
				break;
			case HEADER_B:
				b = read(fd, buffer + 1, 1);
				if (b > 0 && buffer[1] == HEADER_BYTE_2)
				{
					readState = LENGTH;
				}
				else
				{
					readState = HEADER;
				}
				break;
			case LENGTH:
				b = read(fd, buffer + 2, 2);
				if (b >= 2)
				{
					payload = buffer[3];
					readState = FIELDS;
				}
				else
				{
					readState = HEADER;
				}
				break;
			case FIELDS:
				b = read(fd, buffer + 4, payload + 2);
				if (b >= (payload + 2))
				{
					validateAndSend(buffer, payload + 6);
				}
				readState = HEADER;
		}

	} while (b > 0);
}

int main(int argc, char* argv[])
{
	struct sched_param param;
	int c;
	while ((c = getopt(argc, argv, ":d:h:p:")) != EOF)
	{
		switch (c)
		{
			case 'd':
				device = optarg;
				break;
			case 'h':
				host = optarg;
				break;
			case 'p':
				port = optarg;
				break;
			case ':':
				usage();
				exit(-1);
				break;
		}
	}

	if ((device == "") || (host == "") || (port == ""))
	{
		usage();
		exit(-1);
	}

	cout << "Device: " << device << endl;
	cout << "Host: " << host << endl;
	cout << "Port: " << port << endl;

	param.sched_priority = REALTIME_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1)
	{
		perror("sched_setscheduler failed.");
		exit(-1);
	}

	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
	{
		perror("mlockall failed");
		exit(-2);
	}

	stack_prefault();

	openPort();
	openSocket();
	read();

}
