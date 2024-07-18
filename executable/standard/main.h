//
// Created by yaione on 7/10/2022.
//

#ifndef RMCV_MAIN_H
#define RMCV_MAIN_H

#include <iostream>
#include <thread>
#include "rmcv.h"

struct Request {
    rm::CampType Enemy;
    rm::AimMode Mode;
    int FireRate;
    float GimbalPitch;
    float GimbalYaw;
};

struct Response {
    float Pitch;
    float Yaw;
    unsigned char Rank; // dm
};

bool GetRequest(unsigned char *buffer, float fairAngle, Request &request) {
    if (buffer[0] != 0x38) {
        return false;
    }

    if (buffer[11] != rm::LookupCRC(buffer, 11)) {
        return false;
    }

    request.Enemy = static_cast<rm::CampType>(buffer[1] & 0x01) == rm::CAMP_BLUE ? rm::CAMP_RED : rm::CAMP_BLUE;
    request.Mode = (int) ((buffer[1] & 0x04) >> 2) == 0 ? rm::AIM_COMBAT : rm::AIM_BUFF;
    request.FireRate = (int) buffer[2];

    std::memcpy(&request.GimbalYaw, buffer + 3, sizeof(float));
    std::memcpy(&request.GimbalPitch, buffer + 7, sizeof(float));
    request.GimbalPitch = (float) ((fairAngle - request.GimbalPitch) / 8191.0f * CV_2PI);

    return true;
}

void FormatResponse(unsigned char *buffer, const shared_ptr<Response> &response) {
    buffer[0] = 0x66;
    std::memcpy(buffer + 1, &response->Pitch, sizeof(float));
    std::memcpy(buffer + 5, &response->Yaw, sizeof(float));
    buffer[9] = response->Rank;
    buffer[10] = rm::LookupCRC(buffer, 10);
};

#endif //RMCV_MAIN_H
