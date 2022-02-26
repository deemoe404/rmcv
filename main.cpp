#include <iostream>
#include "imgproc.h"

int main() {
    cv::Mat m = cv::imread("test.jpg");

    cv::Rect roi(100, 100, 360, 360);
    cv::Mat m2 = m(roi);

    cv::Point2f pts1[4];
    cv::Point2f pts2[3];

    pts1[0] = cv::Point2f( 120,0 );
    pts1[1] = cv::Point2f( 300, 25 );
    pts1[2] = cv::Point2f( 0, 225 );
    pts1[3] = cv::Point2f( 0, 225 );

    pts2[0] = cv::Point2f( 0, 0);
    pts2[1] = cv::Point2f( 225, 0 );
    pts2[2] = cv::Point2f( 0, 225 );

    cv::Mat m3;
    rm::CalcRatio(pts1,pts2,m2,m3);

    cv::imshow("test", m3);
    cv::waitKey(0);

    return 0;
}
