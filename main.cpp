#include <iostream>
#include "core.h"

int main() {
    cv::RotatedRect rr_1(cv::Point2f(100, 100), cv::Size2f(10, 100), 0);
    cv::RotatedRect rr_2(cv::Point2f(200, 100), cv::Size2f(10, 100), 0);
    rm::LightBar lb_1(rr_1);
    rm::LightBar lb_2(rr_2);
    rm::Armour a({lb_1, lb_2});

    return 0;
}
