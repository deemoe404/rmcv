//
// Created by yaione on 3/3/2022.
//

#include "include/core/utils.h"

namespace rm {
    rm::FileType GetFileType(const char *filename) {
        struct stat buffer{};
        if (stat(filename, &buffer) == 0) {
            if (S_ISREG(buffer.st_mode))
                return rm::FILETYPE_REGULAR_FILE;

            if (S_ISDIR(buffer.st_mode))
                return rm::FILETYPE_DIRECTORY;

            if (S_ISLNK(buffer.st_mode))
                return rm::FILETYPE_SYMBOLIC_LINK;

            if (S_ISSOCK(buffer.st_mode))
                return rm::FILETYPE_SOCKET;
        }

        return rm::FILETYPE_UNKNOWN;
    }

    bool ListFiles(const char *path, std::vector<std::string> &filenames) { // NOLINT(misc-no-recursion)
        DIR *pDir;
        struct dirent *ptr;
        if (!(pDir = opendir(path))) {
            return false;
        }

        std::filesystem::path fullPath;
        while ((ptr = readdir(pDir)) != nullptr) {
            fullPath = path;
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
                fullPath /= ptr->d_name;
                if (GetFileType(fullPath.string().c_str()) == rm::FILETYPE_DIRECTORY) {
                    rm::ListFiles(fullPath.string().c_str(), filenames);
                } else {
                    filenames.push_back(fullPath.string());
                }
            }
        }

        closedir(pDir);
        return true;
    }

    cv::Rect GetROI(cv::Point2f *imagePoints, int pointsCount, float scaleFactor, const cv::Size &frameSize,
                    const cv::Rect &previous) {
        return rm::GetROI(imagePoints, pointsCount, {scaleFactor, scaleFactor}, frameSize, previous);
    }

    cv::Rect GetROI(cv::Point2f *imagePoints, int pointsCount, const cv::Size2f &scaleFactor, const cv::Size &frameSize,
                    const cv::Rect &previous) {
        cv::Rect boundingRect = cv::boundingRect(std::vector<cv::Point2f>(imagePoints, imagePoints + pointsCount));
        boundingRect.x += previous.x;
        boundingRect.y += previous.y;
        if (scaleFactor.width != 1.0f || scaleFactor.height != 1.0f) {
            cv::Size scale((int) ((double) boundingRect.width * scaleFactor.width / 2.0),
                           (int) ((double) boundingRect.height * scaleFactor.height / 2.0));

            boundingRect.x -= scale.width;
            boundingRect.y -= scale.height;
            boundingRect.width += scale.width * 2;
            boundingRect.height += scale.width * 2;
        }
        if (boundingRect.x < 0) {
            boundingRect.x = 0;
        }
        if (boundingRect.y < 0) {
            boundingRect.y = 0;
        }
        if (boundingRect.x + boundingRect.width >= frameSize.width) {
            boundingRect.width = frameSize.width - boundingRect.x - 1;
        }
        if (boundingRect.y + boundingRect.height >= frameSize.height) {
            boundingRect.height = frameSize.height - boundingRect.y - 1;
        }

        if (boundingRect.width < 0 || boundingRect.height < 0) {
            return {0, 0, 0, 0};
        }

        return boundingRect;
    }

    void VerticesRectify(cv::RotatedRect &input, cv::Point2f *output, RectType type = RECT_TALL) {
        cv::Point2f temp[4];
        input.points(temp);

        // sort vertices ascending by y
        std::sort(temp, temp + 4, [](const cv::Point2f &point1, const cv::Point2f &point2) {
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



    float PointDistance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
        return (float) sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    float PointDistance(const cv::Point2i &pt1, const cv::Point2i &pt2) {
        return (float) sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    }

    void
    ExtendCord(const cv::Point2f &pt1, const cv::Point2f &pt2, float deltaLen, cv::Point2f &dst1, cv::Point2f &dst2) {
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
            float zoomY = sin(theta) * deltaLen;
            float zoomX = cos(theta) * deltaLen;

            // Left tilt
            if (k > 0) {
                if (pt1.x > pt2.x) {
                    dst1.x = pt1.x + zoomX;
                    dst1.y = pt1.y + zoomY;

                    dst2.x = pt2.x - zoomX;
                    dst2.y = pt2.y - zoomY;
                } else {
                    dst1.x = pt1.x - zoomX;
                    dst1.y = pt1.y - zoomY;

                    dst2.x = pt2.x + zoomX;
                    dst2.y = pt2.y + zoomY;
                }
                // Right tilt
            } else {
                if (pt1.x < pt2.x) {
                    dst1.x = pt1.x - zoomX;
                    dst1.y = pt1.y + zoomY;

                    dst2.x = pt2.x + zoomX;
                    dst2.y = pt2.y - zoomY;
                } else {
                    dst1.x = pt1.x + zoomX;
                    dst1.y = pt1.y - zoomY;

                    dst2.x = pt2.x - zoomX;
                    dst2.y = pt2.y + zoomY;
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

    void CalcPerspective(cv::Point2f input[4], cv::Point2f output[4], float outRatio) {
        float leftHeight = rm::PointDistance(input[0], input[1]);
        float rightHeight = rm::PointDistance(input[2], input[3]);
        float maxHeight = fmax(leftHeight, rightHeight);

        cv::Size2f size((maxHeight * outRatio), maxHeight);
        cv::Point2f center = rm::LineCenter(rm::LineCenter(input[0], input[1]), rm::LineCenter(input[2], input[3]));

        output[0].x = center.x - size.width / 2;
        output[0].y = center.y - size.height / 2;
        output[1].x = center.x - size.width / 2;
        output[1].y = center.y + size.height / 2;
        output[2].x = center.x + size.width / 2;
        output[2].y = center.y + size.height / 2;
        output[3].x = center.x + size.width / 2;
        output[3].y = center.y - size.height / 2;
    }

    cv::Point2f LineCenter(const cv::Point2f &pt1, const cv::Point2f &pt2) {
        return {pt1.x / 2 + pt2.x / 2, pt1.y / 2 + pt2.y / 2};
    }

}
