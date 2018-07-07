#include "ParameterSearch.h"
#include <iostream>

PID ParameterSearch::next(double error) {
  if (!initialized_) {
    best_err_ = error;
    initialized_ = true;
  }

  if (pos_ == 0) {
    iteration_++;
  }

  std::cout << iteration_ << ": p = (" << p_[0] << ", "
       << p_[1] << ", " << p_[2] << ") pd = (" << dp_[0] << ", "
       << dp_[1] << ", " << dp_[2] << ") e = " << error
       << " best = " << best_err_ << " s = " << state_ << std::endl;

  logger_.log(std::to_string(iteration_), p_, dp_, error, best_err_);

  switch (state_) {
    case START:
      p_[pos_] += dp_[pos_];
      state_ = FORWARD;
      break;

    case FORWARD:
      if (error < best_err_) {
        best_err_ = error;
        dp_[pos_] *= 1.1;
        pos_ = (pos_ + 1) % p_.size();
        p_[pos_] += dp_[pos_];
        state_ = FORWARD;
      } else {
        p_[pos_] -= 2 * dp_[pos_];
        state_ = BACKWARD;
      }
      break;

    case BACKWARD:
      if (error < best_err_) {
        best_err_ = error;
        dp_[pos_] *= 1.1;
      } else {
        p_[pos_] += dp_[pos_];
        dp_[pos_] *= 0.9;
      }
      state_ = FORWARD;
      pos_ = (pos_ + 1) % p_.size();
      p_[pos_] += dp_[pos_];
      break;

    default:
      std::cout << "Invalid state of ParameterSearch" << std::endl;
      break;
  }
  return PID(p_);
}