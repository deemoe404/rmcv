//
// Created by yaione on 3/4/22.
//

#include "serialport.h"

namespace rm
{
    unsigned char lookup_CRC(unsigned char* data, unsigned char dataLength, const unsigned char* crcTable)
    {
        uint8_t crc = 0x00;

        while (dataLength--)
        {
            crc = crcTable[crc ^ *data++];
        }
        return crc;
    }

    bool serial_port::initialize(const char* device, unsigned int speed)
    {
        fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd != -1)
        {
            fcntl(fd, F_SETFL, 0);

            struct termios options{};
            if (tcgetattr(fd, &options) == 0)
            {
                cfsetospeed(&options, speed);
                cfsetispeed(&options, speed);

                options.c_cflag |= (CLOCAL | CREAD);
                options.c_cflag &= ~CRTSCTS;
                options.c_cflag &= ~CSIZE;
                options.c_cflag |= CS8;
                options.c_cflag &= ~CSTOPB;
                options.c_iflag = 0;
                options.c_oflag = 0;
                options.c_lflag = 0;

                tcflush(fd, TCIFLUSH);

                if (!tcsetattr(fd, TCSANOW, &options))
                {
                    initialized = true;
                    return true;
                }
            }
        }
        return false;
    }

    bool serial_port::send(const unsigned char* packet, int length) const
    {
        if (initialized) return write(fd, packet, length) > 0;
        return false;
    }

    bool serial_port::receive(unsigned char* packet, int length) const
    {
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(fd, &fdRead);

        timeval timeout{1, 0};

        if (select(fd + 1, &fdRead, nullptr, nullptr, &timeout) <= 0) return false;
        if (!FD_ISSET(fd, &fdRead)) return false;

        return read(fd, packet, length) == length;
    }

    void serial_port::destroyed() const
    {
        if (fd > 0) close(fd);
    }
}
