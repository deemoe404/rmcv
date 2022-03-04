//
// Created by yaione on 3/4/22.
//

#include "serialport.h"

namespace rm {
    bool SerialPort::Initialize(const char *device = "/dev/ttyUSB0") {
        fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd != -1) {
            fcntl(fd, F_SETFL, 0);

            struct termios options{};
            if (tcgetattr(fd, &options) == 0) {
                cfsetospeed(&options, B460800);
                cfsetispeed(&options, B460800);

                options.c_cflag |= (CLOCAL | CREAD);
                options.c_cflag &= ~CRTSCTS;
                options.c_cflag &= ~CSIZE;
                options.c_cflag |= CS8;
                options.c_cflag &= ~CSTOPB;
                options.c_iflag |= IGNPAR;
                options.c_oflag = 0;
                options.c_lflag = 0;

                tcflush(fd, TCIFLUSH);
                return !tcsetattr(fd, TCSANOW, &options);
            }
        }
        std::perror("Failed opening serial port");
        return false;
    }

    uc SerialPort::LookupCRC(uc *data, uc dataLen) {
        uint8_t crc = 0x00;

        while (dataLen--) {
            crc = CRCTable[crc ^ *data++];
        }
        return crc;
    }

    bool SerialPort::Send(Response &message) {
        //check
        buffer[0] = 0x66;

        //pitch
        buffer[1] = message.pitch.bit[0];
        buffer[2] = message.pitch.bit[1];
        buffer[3] = message.pitch.bit[2];
        buffer[4] = message.pitch.bit[3];

        //yaw
        buffer[5] = message.yaw.bit[0];
        buffer[6] = message.yaw.bit[1];
        buffer[7] = message.yaw.bit[2];
        buffer[8] = message.yaw.bit[3];

        buffer[9] = message.rank;

        if (write(fd, buffer, 10) > 0) {
            return true;
        } else {
            std::perror("Failed writing to serial port");
            return false;
        }
    }

    bool SerialPort::Receive(Request &message) {
        return false;
    }

    void SerialPort::Destroyed() const {
        if (fd > 0) {
            close(fd);
        }
    }
}
