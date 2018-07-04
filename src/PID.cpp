#include "PID.h"
#include <iostream>
using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {}

void PID::UpdateError(double cte) {}

double PID::TotalError() { return 0.; }

static inline double minmax(double r) {
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
    fresh_start_ = false;
  }

  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> dur = current_time - last_time_;
  double delta_t = dur.count();
  double cte_diff = cte - cte_old_;
  if (delta_t < 0.0001)
    cte_diff = 0.;
  else
    cte_diff /= delta_t;

  cte_int_ += cte * delta_t;

  cout << "delta_t = " << delta_t << " integral = " << cte_int_ << endl;

  double result = -Kp_ * cte - Kd_ * cte_diff - Ki_ * cte_int_;
  result = minmax(result);

  cte_old_ = cte;
  last_time_ = current_time;

  return result;
}
