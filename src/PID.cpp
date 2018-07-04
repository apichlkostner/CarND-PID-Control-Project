#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {}

void PID::UpdateError(double cte) {}

double PID::TotalError() {}

static inline double minmax(double r) {
  if (r > 1.) return 1.;
  else if (r < -1.) return -1.;
  else return r;
}

double PID::calc(double cte) {
  if (fresh_start_) {
      cte_old_ = cte;
      fresh_start_ = false;
  }
  double cte_diff = cte - cte_old_;
  cte_int_ += cte;

  double result = -Kp_ * cte - Kd_ * cte_diff - Ki_ * cte_int_;

  cte_old_ = cte;

  result = minmax(result);

  return result;
}
