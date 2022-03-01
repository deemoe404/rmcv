#include <iostream>
#include "imgproc.h"

int main() {
    cv::Mat m = cv::imread("test.jpg");
    rm::CalcGamma(m, m, 0.4);
    cv::imshow("aa", m);
    cv::waitKey(0);



    return 0;
}
