//
// Created by yaione on 3/3/2022.
//

#include "utils.h"

namespace rm {
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

    void VerticesRectify(cv::RotatedRect &input, cv::Point2f *output, RectType type = RECT_TALL) {
        cv::Point2f temp[4];
        input.points(temp);

        // sort vertices ascending by y
        std::sort(temp, temp + 4, [](cv::Point2f point_1, cv::Point2f point_2) {
            return point_1.y < point_2.y;
        });

        bool swap_up = temp[0].x < temp[1].x, swap_down = temp[2].x < temp[3].x;
        output[0] = swap_down ? temp[2] : temp[3]; //left down
        output[1] = swap_up ? temp[0] : temp[1];   //left up
        output[2] = swap_up ? temp[1] : temp[0];   //right up
        output[3] = swap_down ? temp[3] : temp[2]; //right down

        // TODO: Add side support & handle some edge case.
    }
}
