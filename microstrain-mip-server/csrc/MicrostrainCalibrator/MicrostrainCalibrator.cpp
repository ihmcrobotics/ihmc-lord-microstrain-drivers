#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <cstring>
#include <stdint.h>

#define PACKET_LENGTH 262
#define SYNC1 0x75
#define SYNC2 0x65
#define DESC_SET 0x0C
#define PAYLOAD_LENGTH 0x04
#define FIELD_LENGTH 0x04
#define FIELD_DESCRIPTOR 0x39
#define ACK_LENGTH 0x12
#define SAMPLING_TIME 0x7530 // 30000 millis or 30 seconds in hex
#define ACK_FIELD_DESCRIPTION 0xF1
#define BIAS_VECTOR_FIELD_DESCRIPTION 0x9B
#define ACK_PAYLOAD_SIZE 0x04 - 0x02
#define BIAS_VECTOR_PAYLOAD_SIZE 0x0E - 0x02

#define NSEC_PER_SEC 1000000000

#define MAX_SAFE_STACK (32 * 1024)

std::string device;

int fd;

typedef enum
{
    HEADER_BYTE1, HEADER_BYTE2, HEADER_BYTE3, HEADER_BYTE4, FIELD_BYTE1, FIELD_BYTE2, PAYLOAD
} ReadState;

void usage()
{
    std::cout << "Usage: " << std::endl << "-d: Serial device" << std::endl;
}

void openAndLockMicrostrainTTYDevice()
{
    fd = open(device.c_str(), O_RDWR | O_SYNC | O_NOCTTY, S_IRUSR | S_IWUSR);
    if(fd < 0)
    {
        switch(errno)
        {
            case EACCES:
                std::cerr << "Cannot open " << device << ", check permissions" << std::endl;
                break;
            case ENOENT:
                std::cerr << "Cannot open " << device << ", device does not exist" << std::endl;
                break;
            default:
                std::cerr << "An error occured opening " << device << std::endl;
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
        std::cerr << "Cannot lock " << device << std::endl;
        exit(-1);
    }

    struct termios term;
    if (tcgetattr(fd, &term) < 0)
    {
        std::cerr << "Unable to get serial port attributes." << std::endl;
        exit(-1);
    }

    cfmakeraw(&term);
    cfsetispeed(&term, B115200);
    cfsetospeed(&term, B115200);

    if (tcsetattr(fd, TCSAFLUSH, &term) < 0)
    {
        std::cerr << "Unable to set serial port attributes" << std::endl;
        exit(-1);
    }

    if (tcflush(fd, TCIOFLUSH) != 0)
    {
        std::cerr << "tcflush failed" << std::endl;
        exit(-1);
    }
}

void writeCalibrationCommand()
{
    uint8_t mipsPacketBuffer[PACKET_LENGTH];
    uint8_t checksum_byte1 = 0;
    uint8_t checksum_byte2 = 0;

    /* Begin: setup packet according to the MIPS reference */
    memset(mipsPacketBuffer, 0, PACKET_LENGTH);

    //set SYNC bytes
    mipsPacketBuffer[0] = (uint8_t) SYNC1;
    mipsPacketBuffer[1] = (uint8_t) SYNC2;

    //set Descriptor Set byte
    mipsPacketBuffer[2] = (uint8_t) DESC_SET;

    //set Payload Length
    mipsPacketBuffer[3] = (uint8_t) PAYLOAD_LENGTH;

    //set Field Length
    mipsPacketBuffer[4] = (uint8_t) FIELD_LENGTH;

    //set Field Descriptor
    mipsPacketBuffer[5] = (uint8_t) FIELD_DESCRIPTOR;

    //set Payload Data with requested timing
    memset(mipsPacketBuffer + 6, (uint16_t) SAMPLING_TIME, sizeof(uint16_t));

    //calculate Fletcher checksum bytes
    size_t totalBytesMinusChecksumBytes = 4 + PAYLOAD_LENGTH; // 4 header bytes, plus size of payload
    for(size_t i = 0; i < totalBytesMinusChecksumBytes; i++)
    {
        checksum_byte1 += mipsPacketBuffer[i];
        checksum_byte2 += checksum_byte1;
    }

    //set checksum bytes
    mipsPacketBuffer[8] = (uint8_t) checksum_byte1;
    mipsPacketBuffer[9] = (uint8_t) checksum_byte2;

    /* End: MIPS packet setup */

    write(fd, mipsPacketBuffer, totalBytesMinusChecksumBytes + 2);
}

static inline long tsdelta(struct timespec* start, struct timespec* end)
{
    long delta = (end->tv_sec - start->tv_sec) * 1000000000L;
    delta += end->tv_nsec - start->tv_nsec;
    return delta;
}

int readCalibrationAck()
{
    uint8_t mipsPacketBuffer[PACKET_LENGTH];
    ssize_t readBytes;
    size_t fieldDataSize;

    struct timespec startTime, runningTime;
    long waitTime = 0;

    clock_gettime(CLOCK_MONOTONIC, &startTime);
    ReadState currentState = HEADER_BYTE1;
    int stateIndex = 0;
    do
    {
        switch(currentState)
        {

            case HEADER_BYTE1:
                memset(mipsPacketBuffer, 0, PACKET_LENGTH);
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0 && mipsPacketBuffer[stateIndex] == SYNC1)
                {
                    currentState = HEADER_BYTE2;
                    stateIndex++;
                }
                break;
            case HEADER_BYTE2:
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0 && mipsPacketBuffer[stateIndex] == SYNC2)
                {
                    currentState = HEADER_BYTE3;
                    stateIndex++;
                }
                else
                {
                    currentState = HEADER_BYTE1;
                    stateIndex = 0;
                }
                break;
            case HEADER_BYTE3:
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0 && mipsPacketBuffer[stateIndex] == DESC_SET)
                {
                    currentState = HEADER_BYTE4;
                    stateIndex++;
                }
                else
                {
                    currentState = HEADER_BYTE1;
                    stateIndex = 0;
                }
                break;
            case HEADER_BYTE4:
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0 && mipsPacketBuffer[stateIndex] == ACK_LENGTH)
                {
                    currentState = FIELD_BYTE1;
                    stateIndex++;
                }
                else
                {
                    currentState = HEADER_BYTE1;
                    stateIndex = 0;
                }
                break;
            case FIELD_BYTE1:
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0)
                {
                    switch(mipsPacketBuffer[stateIndex])
                    {
                        case 0x04:
                            currentState = FIELD_BYTE2;
                            stateIndex++;
                            fieldDataSize = ACK_PAYLOAD_SIZE;
                            break;
                        case 0x0E:
                            currentState = FIELD_BYTE2;
                            stateIndex++;
                            fieldDataSize = BIAS_VECTOR_PAYLOAD_SIZE;
                            break;
                        default:
                            currentState = HEADER_BYTE1;
                            stateIndex = 0;
                            break;
                    }
                }
                else
                {
                    currentState = HEADER_BYTE1;
                    stateIndex = 0;
                }
                break;
            case FIELD_BYTE2:
                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                if(readBytes > 0)
                {
                    switch(mipsPacketBuffer[stateIndex])
                    {
                        case ACK_FIELD_DESCRIPTION:
                            currentState = PAYLOAD;
                            stateIndex++;
                            break;
                        case BIAS_VECTOR_FIELD_DESCRIPTION:
                            currentState = PAYLOAD;
                            stateIndex++;
                            break;
                        default:
                            currentState = HEADER_BYTE1;
                            stateIndex = 0;
                            break;
                    }
                }
                break;
            case PAYLOAD:
                switch(mipsPacketBuffer[stateIndex - 1])
                {
                    case ACK_FIELD_DESCRIPTION:
                        if(fieldDataSize == ACK_PAYLOAD_SIZE)
                        {
                            readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                            if(readBytes > 0 && mipsPacketBuffer[stateIndex] == FIELD_DESCRIPTOR)
                            {
                                stateIndex++;
                                readBytes = read(fd, mipsPacketBuffer + stateIndex, 1);
                                if(readBytes > 0 && mipsPacketBuffer[stateIndex] == 0x00)
                                {
                                    return 0;
                                }
                                else
                                {
                                    return -1;
                                }
                            }
                            else
                            {
                                currentState = HEADER_BYTE1;
                                stateIndex = 0;
                            }
                        }
                        else
                        {
                            currentState = HEADER_BYTE1;
                            stateIndex = 0;
                        }
                        break;
                    default:
                        currentState = HEADER_BYTE1;
                        stateIndex = 0;
                        break;
                }
                break;
        }
        clock_gettime(CLOCK_MONOTONIC, &runningTime);

        waitTime = tsdelta(&startTime, &runningTime);
    } while (waitTime / NSEC_PER_SEC <= 60);

    return -2;
}

int main(int argc, char* argv[])
{
    int c;
    while ((c = getopt(argc, argv, ":d:")) != EOF)
    {
        switch (c)
        {
            case 'd':
                device = optarg;
                break;
            case ':':
                usage();
                exit(-1);
        }
    }

    if ((device == ""))
    {
        usage();
        exit(-1);
    }

    std::cout << "Running calibration for: " << device << std::flush << std::endl;

    openAndLockMicrostrainTTYDevice();

    writeCalibrationCommand();

    int result = readCalibrationAck();

    if(result == 0)
    {
        std::cout << "Calibration succesful!" << std::endl;
        exit(0);
    }
    else
    {
        std::cerr << "Calibration for device " << device << "failed." << std::endl;
        if(result == -1)
        {
            std::cerr << "Received NACK from IMU. Try calibration again." << std::endl;
        }
        else
        {
            std::cerr << "Timed out waiting for ACK packed from IMU.  Try calibration again." << std::endl;
        }
        exit(-1);
    }
}
