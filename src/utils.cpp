//
// Created by yaione on 3/3/2022.
//

#include "rmcv/core/utils.h"

namespace rm {
    void VerticesRectify(cv::RotatedRect &input, cv::Point *output, RectType type = RECT_TALL) {
        cv::Point2f temp[4];
        input.points(temp);

        // sort vertices ascending by y
        std::sort(temp, temp + 4, [](cv::Point2f point1, cv::Point2f point2) {
            return point1.y < point2.y;
        });

        bool swap_up = temp[0].x < temp[1].x, swap_down = temp[2].x < temp[3].x;
        output[0] = swap_down ? temp[2] : temp[3]; // left down
        output[1] = swap_up ? temp[0] : temp[1];   // left up
        output[2] = swap_up ? temp[1] : temp[0];   // right up
        output[3] = swap_down ? temp[3] : temp[2]; // right down

        // TODO: Add side support & handle some edge case.
    }

    double NewtonIteration(double (*fd)(double), double x0, double error, int cycle) {
        double a = x0;
        double x = x0 - fd(x0);

        while (fabs(x - a) > error && cycle--) {
            a = x;
            x = x - fd(x);
            if (a == x) {
                return x;
            }
        }
        return x;
    }

    double NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double> &literals, double x0,
                           double error, int cycle) {
        double a = x0;
        double x = x0 - fd(x0, literals);

        while (fabs(x - a) > error && cycle--) {
            a = x;
            x = x - fd(x, literals);
            if (a == x) {
                return x;
            }
        }
        return x;
    }

    double ProjectileMotionFD(double theta, std::vector<double> literals) {
        if (literals.size() == 4) {
            double g = literals[0], d = literals[1], h = literals[2], v0 = literals[3];
            int towards = (int) theta ? (int) theta / abs((int) theta) : 1;

            double fx = h - d * tan(theta) + towards * (pow(d, 2.0) * g / pow(cos(theta) * v0, 2.0) * 2.0);
            double dx = towards * (pow(d, 2.0) * g * sin(theta)) / ((pow(v0, 2.0) * pow(cos(theta), 3.0))) -
                        d * (pow(tan(theta), 2.0) + 1);

            return (1 * fx / dx);
        } else {
            return -1;
        }
    }

    float PointDistance(cv::Point2f pt1, cv::Point2f pt2) {
        return (float) sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    float PointDistance(cv::Point2i pt1, cv::Point2i pt2) {
        return (float) sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    void ExCord(cv::Point pt1, cv::Point pt2, int deltaLen, cv::Point &dst1, cv::Point &dst2) {
        // Special case
        if (pt1.x == pt2.x) {
            dst1.x = pt1.x;
            dst2.x = pt1.x;

            if (pt1.y > pt2.y) {
                dst1.y = pt1.y + deltaLen;
                dst2.y = pt2.y - deltaLen;
            } else {
                dst1.y = pt1.y - deltaLen;
                dst2.y = pt2.y + deltaLen;
            }
        } else if (pt1.y == pt2.y) {
            dst1.y = pt1.y;
            dst2.y = pt1.y;

            if (pt1.x > pt2.x) {
                dst1.x = pt1.x + deltaLen;
                dst2.x = pt2.x - deltaLen;
            } else {
                dst1.x = pt1.x - deltaLen;
                dst2.x = pt2.x + deltaLen;
            }
            // Common case
        } else {
            float k = (float) (pt1.y - pt2.y) / (float) (pt1.x - pt2.x);
            float theta = atan2((float) abs(pt1.y - pt2.y), (float) abs(pt1.x - pt2.x));
            float b = (float) pt1.y - k * (float) pt1.x;
            float zoom = cos(theta) * (float) deltaLen;

            // Left tilt
            if (k > 0) {
                if (pt1.x > pt2.x) {
                    dst1.x = (int) ((float) pt1.x + zoom);
                    dst1.y = (int) (k * (float) dst1.x + b);

                    dst2.x = (int) ((float) pt2.x - zoom);
                    dst2.y = (int) (k * (float) dst2.x + b);
                } else {
                    dst1.x = (int) ((float) pt1.x - zoom);
                    dst1.y = (int) (k * (float) dst1.x + b);

                    dst2.x = (int) ((float) pt2.x + zoom);
                    dst2.y = (int) (k * (float) dst2.x + b);
                }
                // Right tilt
            } else {
                if (pt1.x < pt2.x) {
                    dst1.x = (int) ((float) pt1.x - zoom);
                    dst1.y = (int) (k * (float) dst1.x + b);

                    dst2.x = (int) ((float) pt2.x + zoom);
                    dst2.y = (int) (k * (float) dst2.x + b);
                } else {
                    dst1.x = (int) ((float) pt1.x + zoom);
                    dst1.y = (int) (k * (float) dst1.x + b);

                    dst2.x = (int) ((float) pt2.x - zoom);
                    dst2.y = (int) (k * (float) dst2.x + b);
                }
            }
        }
    }

    std::string PathCombine(const std::string &path1, const std::string &path2) {
        std::filesystem::path fullPath = path1;
        fullPath /= path2;
        return fullPath.string();
    }

    std::string int2str(int number) {
        char str[64] = {0};
        std::snprintf(str, 64, "%d", number);
        std::string res(str);
        return res;
    }

    void PrintMat(cv::Mat &input, int decimal) {
        for (int i = 0; i < input.size().height; i++) {
            std::cout << "[ ";
            for (int j = 0; j < input.size().width; j++) {
                std::cout << std::fixed << std::setw(2) << std::setprecision(decimal) << input.at<float>(i, j);
                if (j != input.size().width - 1)
                    std::cout << ", ";
                else
                    std::cout << " ]" << std::endl;
            }
        }
    }

    void AxisRotateZ(double x, double y, double thetaZ, double &outX, double &outY) {
        double x1 = x;
        double y1 = y;
        double rz = thetaZ * CV_PI / 180;
        outX = cos(rz) * x1 - sin(rz) * y1;
        outY = sin(rz) * x1 + cos(rz) * y1;
    }

    void AxisRotateY(double x, double z, double thetaY, double &outX, double &outZ) {
        double x1 = x;
        double z1 = z;
        double ry = thetaY * CV_PI / 180;
        outX = cos(ry) * x1 + sin(ry) * z1;
        outZ = cos(ry) * z1 - sin(ry) * x1;
    }

    void AxisRotateX(double y, double z, double thetaX, double &outY, double &outZ) {
        double y1 = y;
        double z1 = z;
        double rx = thetaX * CV_PI / 180;
        outY = cos(rx) * y1 - sin(rx) * z1;
        outZ = cos(rx) * z1 + sin(rx) * y1;
    }

    void CalcPerspective(cv::Point2i input[4], cv::Point2i output[4], float outRatio) {
        float leftHeight = rm::PointDistance(input[0], input[1]);
        float rightHeight = rm::PointDistance(input[2], input[3]);
        float maxHeight = fmax(leftHeight, rightHeight);

        cv::Size2i size((int) (maxHeight * outRatio), (int) maxHeight);
        cv::Point2i center = rm::LineCenter(rm::LineCenter(input[0], input[1]), rm::LineCenter(input[2], input[3]));

        output[0].x = center.x - size.width / 2;
        output[0].y = center.y - size.height / 2;
        output[1].x = center.x - size.width / 2;
        output[1].y = center.y + size.height / 2;
        output[2].x = center.x + size.width / 2;
        output[2].y = center.y + size.height / 2;
        output[3].x = center.x + size.width / 2;
        output[3].y = center.y - size.height / 2;
    }

    cv::Point2i LineCenter(cv::Point2i pt1, cv::Point2i pt2) {
        return {pt1.x / 2 + pt2.x / 2, pt1.y / 2 + pt2.y / 2};
    }

    bool RectIntersect(int x01, int x02, int y01, int y02, int x11, int x12, int y11, int y12) {
        int zx = abs(x01 + x02 -x11 - x12);
        int x  = abs(x01 - x02) + abs(x11 - x12);
        int zy = abs(y01 + y02 - y11 - y12);
        int y  = abs(y01 - y02) + abs(y11 - y12);
        if(zx <= x && zy <= y)
            return true;
        else
            return false;
    }
}
