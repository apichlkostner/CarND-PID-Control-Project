#include "PID.h"
#include <iostream>
using namespace std;

PID::~PID() {}

static inline double bound_result(double r) {
  if (r > 1.)
    return 1.;
  else if (r < -1.)
    return -1.;
  else
    return r;
}

double PID::calc(double cte) {
  if (fresh_start_) {
    last_time_ = std::chrono::system_clock::now();
    cte_old_ = cte;
    cte_int_ = 0;
    fresh_start_ = false;
  }

  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> dur = current_time - last_time_;
  double delta_t = dur.count();
  double cte_diff = cte - cte_old_;

  constexpr double MINDELTATIME = 0.00001;

  if (delta_t > MINDELTATIME) {
    cte_diff /= delta_t;
  } else { 
    // should not happen - just use cte_diff without dt
  }

  cte_int_ += cte * delta_t;

  double result = -Kp_ * cte - Kd_ * cte_diff - Ki_ * cte_int_;

  // should be bounded to [-1, 1]
  result = bound_result(result);

  cte_old_ = cte;
  last_time_ = current_time;

  return result;
}
