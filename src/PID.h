#ifndef PID_H
#define PID_H

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double cte_old_;
  double cte_int_;

  /*
   * Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  bool fresh_start_ = true;

  /*
   * Constructor
   */
  PID(double Kp, double Kd, double Ki) : Kp_(Kp), Ki_(Ki), Kd_(Kd){};

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  double calc(double cte);
};

#endif /* PID_H */
