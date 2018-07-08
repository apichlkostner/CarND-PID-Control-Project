#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>
#include <cassert>

class PID {
 public:
  double cte_old_;
  double cte_int_;

  /*
   * Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  bool fresh_start_ = true;

  std::chrono::time_point<std::chrono::system_clock> last_time_;

  /*
   * Constructor
   */
  PID(double Kp, double Kd, double Ki) : Kp_(Kp), Ki_(Ki), Kd_(Kd){};

  PID(const std::vector<double>& p) {
    assert(p.size() == 3);

    Kp_ = p[0];
    Kd_ = p[1];
    Ki_ = p[2];

    cte_old_ = 0;
    cte_int_ = 0;
  };

  /*
   * Destructor.
   */
  virtual ~PID();

  double calc(double cte);

  void reset() {
    cte_old_ = 0;
    cte_int_ = 0;
  }
};

#endif /* PID_H */
