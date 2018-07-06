#ifndef PARAMETERSEARCH_H_
#define PARAMETERSEARCH_H_

#include <vector>
#include "PID.h"

class ParameterSearch {
    private:
    std::vector<double> p_;
    std::vector<double> dp_;

    double best_err_;
    int pos_;

    int iteration_;

    enum state_e {
        START,
        FORWARD,
        BACKWARD
    };

    enum state_e state_;
    bool initialized_;

    public:
    // void ParameterSeach(int len) {
    //     parameter_.resize(len);
    // }

    ParameterSearch(std::vector<double> parameter, std::vector<double> delta) {
        p_ = parameter;
        dp_ = delta;
        best_err_ = std::numeric_limits<double>::max();
        state_ = START;
        pos_ = 0;
        iteration_ = 0;
    }

    PID next(double error);
};

#endif