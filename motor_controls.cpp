#include <cmath>

#include "PinNames.h"
#include "cmd.h"
#include "mbed.h"
#include "motor_controls.h"
#include "io_abstraction.h"
#include "tb6612.h"
#include "pid.h"
#include "quad_encoder.h"
#include "battery_monitor.h"
#include "lp_filter.h"
#include "config.h"

#define TOLERANCE              (0.0f) /* rad/s */
#define STOP_THRESHOLD         (0.5f) /* rad/s */
#define VBATT_FILT_ALPHA       (0.4f)

typedef struct {
  tb6612::Motor_Id_T id;
  pid::PID * pid_ptr;
  uint32_t cnt;
  float    ke;         /* DC motor speed constant */
  float    sp_rad_s;   /* Desired angular velocity (rad/s) */
  float    fb_rad_s;   /* Actual angular velocity (rad/s)  */
  float    e_rad_s;    /* Velocity error (rad/s) */
  float    sp_volts;   /* DC motor setpoint voltage  */
  float    fb_volts;   /* DC motor feedback voltage  */
  float    u_volts;    /* DC motor actuation voltage */
  uint8_t  u_percent;  /* DC motor actuation voltage as a % of max voltage */
} Ctrl_Data_T;

/* Motor driver object */
static tb6612::TB6612 motor_driver(MOTOR_A_PWM, MOTOR_B_PWM, MOTOR_A_IN1, \
                                   MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2,
                                   MOTOR_A_POLARITY, MOTOR_B_POLARITY);

/* PID controller objects */
static pid::PID l_pid(CYCLE_TIME_S, TOLERANCE);
static pid::PID r_pid(CYCLE_TIME_S, TOLERANCE);

/* Wheel speed quadrature encoder objects */
static quad_encoder::QuadEncoder r_encoder(R_ENCODER_B, PullNone, R_ENCODER_A, PullNone);
static quad_encoder::QuadEncoder l_encoder(L_ENCODER_A, PullNone, L_ENCODER_B, PullNone);

/* Variables */
static Timer t;
static Ctrl_Data_T r_motor_ctrl_data;
static Ctrl_Data_T l_motor_ctrl_data;
static bool  ctrl_active   = false;
static bool  awaiting_stop = false;
static float meas_vbatt    = 0.0f;
static float max_vbatt     = 0.0f;
static float wheel_speed_filt_alpha = 1.0f;
static const float MIN_V   = 0.0f;
static const float PULSES_2_RPS = (1.0f / PULSES_PER_REV) * 2 * 3.14159 * (1 / CYCLE_TIME_S);

static Mutex ctrl_data_mutex;

static void Run_Controller(Ctrl_Data_T * ctrl_data);

void InitMotorControls(void) {
  memset(&r_motor_ctrl_data, 0, sizeof(r_motor_ctrl_data));
  memset(&l_motor_ctrl_data, 0, sizeof(l_motor_ctrl_data));

  motor_driver.SetPWMPeriod(0.0001);

  r_motor_ctrl_data.ke = R_Ke;
  l_motor_ctrl_data.ke = L_Ke;

  r_motor_ctrl_data.id = R_MOTOR;
  l_motor_ctrl_data.id = L_MOTOR;

  r_motor_ctrl_data.pid_ptr = &r_pid;
  l_motor_ctrl_data.pid_ptr = &l_pid;

  t.start();
}

void RunMotorControls(void) {
  int t_start = 0;

  while (1) {
    t_start = t.read_ms();

    /* Determine the max actuation voltage based on the vbatt measurement */
    meas_vbatt = LpFilter(ReadBatteryVoltage(), meas_vbatt, VBATT_FILT_ALPHA);

    if (meas_vbatt < MAX_MOTOR_VOLTAGE) {
      max_vbatt = meas_vbatt - tb6612::vdrop;
    } else {
      max_vbatt = MAX_MOTOR_VOLTAGE - tb6612::vdrop;
    }
    
    if (ctrl_data_mutex.trylock())
    {
      /* Measure the current wheel speeds via the encoders */
      r_motor_ctrl_data.fb_rad_s = LpFilter(r_encoder.GetPulses() * PULSES_2_RPS, \
                                            r_motor_ctrl_data.fb_rad_s, \
                                            wheel_speed_filt_alpha);

      l_motor_ctrl_data.fb_rad_s = LpFilter(l_encoder.GetPulses() * PULSES_2_RPS, \
                                            l_motor_ctrl_data.fb_rad_s, \
                                            wheel_speed_filt_alpha);
      ctrl_data_mutex.unlock();
      if (ctrl_active) {
        Run_Controller(&r_motor_ctrl_data);
        Run_Controller(&l_motor_ctrl_data);

        /* Are we slowing down to stop? */
        if (awaiting_stop && (r_motor_ctrl_data.e_rad_s < STOP_THRESHOLD) \
            && (l_motor_ctrl_data.e_rad_s < STOP_THRESHOLD)) {
        motor_driver.Stop(R_MOTOR);
        motor_driver.Stop(L_MOTOR);

        ctrl_active = false;
        awaiting_stop = false;
        }
      } else {
          r_motor_ctrl_data.u_percent = 0;
          l_motor_ctrl_data.u_percent = 0;
      }

      motor_driver.SetDC(R_MOTOR, r_motor_ctrl_data.u_percent);
      motor_driver.SetDC(L_MOTOR, l_motor_ctrl_data.u_percent);

      ThisThread::sleep_for((CYCLE_TIME_S * S_2_MS) - (t.read_ms() - t_start));
    }
    else
    {
      ThisThread::sleep_for(1);
    }
  }
}

static void Run_Controller(Ctrl_Data_T * ctrl_data) {
  tb6612::Direction_T dir = tb6612::UNKNOWN_DIR;

  ctrl_data->cnt++;

  /* Compute the voltage set point  */
  ctrl_data->sp_volts = ctrl_data->sp_rad_s * ctrl_data->ke;

  /* Compute the voltage feedback  */
  ctrl_data->fb_volts = ctrl_data->fb_rad_s * ctrl_data->ke;

  /* Compute the velocity error */
  ctrl_data->e_rad_s = ctrl_data->sp_rad_s - ctrl_data->fb_rad_s;

#if 0 == OPEN_LOOP
  /* Run the PID controller */
  ctrl_data->u_volts = ctrl_data->pid_ptr->Step(ctrl_data->sp_volts, \
                                                ctrl_data->fb_volts, \
                                                max_vbatt, MIN_V);
#else
  /* Saturate the set points to be within the actuator voltage range */
  int8_t sign;
  sign = signbit(ctrl_data->sp_volts) ? -1 : 1;
  ctrl_data->u_volts = fabs(ctrl_data->sp_volts) < max_vbatt ? \
                        ctrl_data->sp_volts : sign * max_vbatt;
#endif  // OPEN_LOOP

  /* Convert the actuation voltage to a percent duty cycle */
  ctrl_data->u_percent = (uint8_t)(fabs(ctrl_data->u_volts) * (100 / meas_vbatt));

  /* Determine direction */
  dir = signbit(ctrl_data->u_volts) ? tb6612::REVERSE : tb6612::FORWARD;

  /* Set the motor direction */
  motor_driver.SetDirection(ctrl_data->id, dir);
}

void GetWheelAngVSp(Wheel_Ang_V_T* dest) {
    if (dest) {
    dest->r = r_motor_ctrl_data.sp_rad_s;
    dest->l = l_motor_ctrl_data.sp_rad_s;
  }
}

void GetWheelAngV(Wheel_Ang_V_T* dest) {
  ctrl_data_mutex.lock();
  if (dest) {
    dest->r = r_motor_ctrl_data.fb_rad_s;
    dest->l = l_motor_ctrl_data.fb_rad_s;
  }
  ctrl_data_mutex.unlock();
}

void UpdateMotorControllerInputs(const oscar_pi::cmd& cmd_msg) {
  pid::Gains_T r_gains = { 0.0f };
  pid::Gains_T l_gains = { 0.0f };

  if (0 == cmd_msg.stop) {
    r_motor_ctrl_data.sp_rad_s = cmd_msg.r_wheel_sp;
    l_motor_ctrl_data.sp_rad_s = cmd_msg.l_wheel_sp;
  }
  else {
    r_motor_ctrl_data.sp_rad_s = 0.0f;
    l_motor_ctrl_data.sp_rad_s = 0.0f;
    awaiting_stop = true;
  }

  wheel_speed_filt_alpha = cmd_msg.wheel_speed_filt_alpha;

  r_gains.kp = cmd_msg.r_kp;
  r_gains.ki = cmd_msg.r_ki;
  r_gains.kd = cmd_msg.r_kd;
  r_pid.Reset(&r_gains);

  l_gains.kp = cmd_msg.l_kp;
  l_gains.ki = cmd_msg.l_ki;
  l_gains.kd = cmd_msg.l_kd;
  l_pid.Reset(&l_gains);

  ctrl_active = true;
}
