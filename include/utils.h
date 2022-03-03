//
// Created by yaione on 3/3/2022.
//

#ifndef RM_STANDARD2022_UTILS_H
#define RM_STANDARD2022_UTILS_H

#include <cmath>
#include <vector>

namespace rm {
    double NewtonIteration(double (*fd)(double), double x0, double error, int cycle);

    double NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double>& literals, double x0,
                           double error, int cycle);
}

#endif //RM_STANDARD2022_UTILS_H
