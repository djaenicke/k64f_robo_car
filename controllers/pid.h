#ifndef CONTROLLERS_PID_H_
#define CONTROLLERS_PID_H_

#include "mbed.h"

namespace pid {

typedef struct {
  float k_p;
  float k_i;
  float k_d;
  float dt;
  float tol;
} PID_Cals_T;

class PID {
 private:
  float kp_;
  float ki_;
  float kd_;
  float dt_;
  float tol_; /* Tolerance */
  float last_e_;
  float integral_;
 public:
  PID(float kp, float ki, float kd, float dt, float tol = 0.0f);
  float Step(float sp, float fb, float max, float min);
  void  Reset(void);
};

}  // namespace pid

#endif  // CONTROLLERS_PID_H_
