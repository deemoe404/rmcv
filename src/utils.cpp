//
// Created by yaione on 3/3/2022.
//

#include "utils.h"

namespace rm {
    double NewtonIteration(double (*fd)(double), double x0 = 0, double error = 0.0001, int cycle = 1024) {
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

    double
    NewtonIteration(double (*fd)(double, std::vector<double>), const std::vector<double> &literals = {}, double x0 = 0,
                    double error = 0.0001, int cycle = 1024) {
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
}
