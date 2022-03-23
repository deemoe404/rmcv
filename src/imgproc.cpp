//
// Created by yaione on 2/26/2022.
//

#include "rmcv/imgproc.h"

namespace rm {
    void CalcRatio(cv::Mat &input, cv::Mat &output, cv::Point vertices[4], cv::Rect box, cv::Size outSize) {
        // handel the case of vertices out of screen
        bool overSized = false;
        for (int i = 0; i < 4; i++) {
            if (vertices[i].x < 0 || vertices[i].y < 0 || vertices[i].x > input.cols - 1 ||
                vertices[i].y > input.rows - 1) {
                if (vertices[i].x < 0) vertices[i].x = 0;
                if (vertices[i].y < 0) vertices[i].y = 0;
                if (vertices[i].x > input.cols - 1) vertices[i].x = input.cols - 1;
                if (vertices[i].y > input.rows - 1) vertices[i].y = input.rows - 1;

                overSized = true;
            }
        }
        if (overSized) {
            box = cv::boundingRect(std::vector<cv::Point>({vertices[0], vertices[1], vertices[2], vertices[3]}));
        }

        cv::Point2f srcPts[3] = {{(float) (vertices[1].x - box.x), (float) (vertices[1].y - box.y)},
                                 {(float) (vertices[2].x - box.x), (float) (vertices[2].y - box.y)},
                                 {(float) (vertices[0].x - box.x), (float) (vertices[0].y - box.y)}};
        cv::Point2f dstPts[3] = {{0,                   0},
                                 {(float) (box.width), 0},
                                 {0,                   (float) (box.height)}};
        auto warp = cv::getAffineTransform(srcPts, dstPts);
        cv::warpAffine(input(box), output, warp, output.size());

        // resize to the dst size
        cv::resize(output, output, outSize);
    }

    void CalcGamma(cv::Mat &input, cv::Mat &output, float gamma) {
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar *p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
        }

        // search for LUT table
        cv::LUT(input, lookUpTable, output);
    }

    void ExtractColor(cv::Mat &input, cv::Mat &output, rm::CampType camp) {
        std::vector<cv::Mat> channels;
        cv::split(input, channels);

        // extract color
        auto gray = channels[camp == rm::CAMP_RED ? 0 : 2] - channels[camp == rm::CAMP_RED ? 2 : 0];
        cv::inRange(gray, 80, 255, output);

        // enhance visual
        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
        cv::morphologyEx(output, output, cv::MORPH_CLOSE, kernel);
    }
}
