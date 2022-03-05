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
        return false;
    }

    uc SerialPort::lookupCRC(uc *data, uc dataLen) {
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

        //rank
        buffer[9] = message.rank;

        return write(fd, buffer, 10) > 0;
    }

    bool SerialPort::Receive(Request &message) {
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(fd, &fdRead);

        struct timeval timeout{};
        timeout.tv_sec = 600;
        timeout.tv_usec = 0;

        if (select(fd + 1, &fdRead, nullptr, nullptr, &timeout) <= 0) {
            return false;
        }

        if (!FD_ISSET(fd, &fdRead)) {
            return false;
        }

        if (read(fd, buffer, 22) > 0) {
            if (buffer[0] != 0x38) {
                return false;
            }

            if (buffer[21] != lookupCRC(buffer, 21)) {
                return false;
            }

            // TODO: confirm the definition of camp & mode.
            message.camp = buffer[1] & 0x01;
            message.mode = (buffer[1] & 0x04) >> 2;
            message.speed = buffer[2];
            message.timestamp = buffer[3] << 8 | buffer[4];

            message.yaw.bit[0] = buffer[5];
            message.yaw.bit[1] = buffer[6];
            message.yaw.bit[2] = buffer[7];
            message.yaw.bit[3] = buffer[8];

            message.pitch.bit[0] = buffer[9];
            message.pitch.bit[1] = buffer[10];
            message.pitch.bit[2] = buffer[11];
            message.pitch.bit[3] = buffer[12];

            message.pitchSpeed.bit[0] = buffer[13];
            message.pitchSpeed.bit[1] = buffer[14];
            message.pitchSpeed.bit[2] = buffer[15];
            message.pitchSpeed.bit[3] = buffer[16];

            message.yawSpeed.bit[0] = buffer[17];
            message.yawSpeed.bit[1] = buffer[18];
            message.yawSpeed.bit[2] = buffer[19];
            message.yawSpeed.bit[3] = buffer[20];

            return true;
        } else {
            return false;
        }
    }

    void SerialPort::Destroyed() const {
        if (fd > 0) {
            close(fd);
        }
    }

}
