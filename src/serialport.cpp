//
// Created by yaione on 3/4/22.
//

#include "rmcv/core/serialport.h"

namespace rm {
    uc LookupCRC(uc *data, uc dataLen) {
        uint8_t crc = 0x00;

        while (dataLen--) {
            crc = CRCTable[crc ^ *data++];
        }
        return crc;
    }

    bool SerialPort::Initialize(const char *device) {
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

    bool SerialPort::Send(Response &message) {
        //check
        sendBuffer[0] = 0x66;

        //pitch
        sendBuffer[1] = message.pitch.bit[0];
        sendBuffer[2] = message.pitch.bit[1];
        sendBuffer[3] = message.pitch.bit[2];
        sendBuffer[4] = message.pitch.bit[3];

        //yaw
        sendBuffer[5] = message.yaw.bit[0];
        sendBuffer[6] = message.yaw.bit[1];
        sendBuffer[7] = message.yaw.bit[2];
        sendBuffer[8] = message.yaw.bit[3];

        //rank
        sendBuffer[9] = message.rank;

        return write(fd, sendBuffer, 10) > 0;
    }

    bool SerialPort::Receive(Request &message) {
        fd_set fdRead;
        FD_ZERO(&fdRead);
        FD_SET(fd, &fdRead);

        struct timeval timeout{};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        if (select(fd + 1, &fdRead, nullptr, nullptr, &timeout) <= 0) {
            return false;
        }

        if (!FD_ISSET(fd, &fdRead)) {
            return false;
        }

        if (read(fd, receiveBuffer, 8) == 8) {
            if (receiveBuffer[0] != 0x38) {
                return false;
            }

            if (receiveBuffer[7] != LookupCRC(receiveBuffer, 7)) {
                return false;
            }

            // TODO: confirm the definition of camp & mode.
            message.ownCamp = receiveBuffer[1] & 0x01;
            message.mode = (receiveBuffer[1] & 0x04) >> 2;
            message.speed = receiveBuffer[2];

            message.pitch.bit[0] = receiveBuffer[3];
            message.pitch.bit[1] = receiveBuffer[4];
            message.pitch.bit[2] = receiveBuffer[5];
            message.pitch.bit[3] = receiveBuffer[6];

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
