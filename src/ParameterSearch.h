#ifndef PARAMETERSEARCH_H_
#define PARAMETERSEARCH_H_

#include <vector>
#include "PID.h"
#include "Logger.h"

class ParameterSearch {
 private:
  std::vector<double> p_;
  std::vector<double> dp_;

  double best_err_;
  int pos_;

  int iteration_;

  enum state_e { START, FORWARD, BACKWARD };

  enum state_e state_;
  bool initialized_;

  Logger logger_;

 public:
  ParameterSearch(std::vector<double> parameter, std::vector<double> delta,
                  double err_init = std::numeric_limits<double>::max()) {
    p_ = parameter;
    dp_ = delta;
    best_err_ = err_init;
    state_ = START;
    pos_ = 0;
    iteration_ = 0;
  }

  // returns next PID with new parameters
  PID next(double error);
};

#endif