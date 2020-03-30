#include <cmath>

#include "mbed.h"
#include "motor_controls.h"
#include "io_abstraction.h"
#include "tb6612.h"
#include "pid.h"
#include "encoder.h"
#include "battery_monitor.h"
#include "lp_filter.h"
#include "cfg.h"

#define R_Ke 0.226f
#define L_Ke 0.192f

#define L_Kp 10.0f
#define L_Ki 40.0f
#define L_Kd 0.52f

#define R_Kp 5.5f
#define R_Ki 15.0f
#define R_Kd 0.3f

#define TOLERANCE              (0.0f) /* rad/s */
#define STOP_THRESHOLD         (0.5f) /* rad/s */
#define VBATT_FILT_ALPHA       (0.4f)
#define PULSES_PER_REV         (192)
#define WHEEL_SPEED_FILT_ALPHA (0.4f)

/* Redefine motors ids to locations for code readability */
#define R_MOTOR tb6612::MOTOR_A
#define L_MOTOR tb6612::MOTOR_B

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
                                   MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2);

/* PID controller objects */
static pid::PID l_pid(L_Kp, L_Ki, L_Kd, CYCLE_TIME, TOLERANCE);
static pid::PID r_pid(R_Kp, R_Ki, R_Kd, CYCLE_TIME, TOLERANCE);

/* Wheel speed encoder objects */
static encoder::Encoder r_encoder(R_ENCODER, PullUp, PULSES_PER_REV, 50);
static encoder::Encoder l_encoder(L_ENCODER, PullUp, PULSES_PER_REV, 50);

/* Variables */
static Ctrl_Data_T r_motor_ctrl_data;
static Ctrl_Data_T l_motor_ctrl_data;
static bool  ctrl_active   = false;
static bool  awaiting_stop = false;
static float meas_vbatt    = 0.0f;
static float max_vbatt     = 0.0f;
static const float min_v   = 0.0f;

#if TUNE
static Serial debug_out(USBTX, USBRX, 115200);
static Timer t;
#endif

static void Run_Controller(Ctrl_Data_T * ctrl_data);

void InitMotorControls(void) {
  memset(&r_motor_ctrl_data, 0, sizeof(r_motor_ctrl_data));
  memset(&l_motor_ctrl_data, 0, sizeof(l_motor_ctrl_data));

  motor_driver.SetPWMPeriod(1);

  r_motor_ctrl_data.ke = R_Ke;
  l_motor_ctrl_data.ke = L_Ke;

  r_motor_ctrl_data.id = R_MOTOR;
  l_motor_ctrl_data.id = L_MOTOR;

  r_motor_ctrl_data.pid_ptr = &r_pid;
  l_motor_ctrl_data.pid_ptr = &l_pid;

#if TUNE
  t.start();
#endif
}

void RunMotorControls(void) {
  int8_t sign;
  
  while (1) {
    /* Determine the max actuation voltage based on the vbatt measurement */
    meas_vbatt = LpFilter(ReadBatteryVoltage(), meas_vbatt, VBATT_FILT_ALPHA);
    max_vbatt  = meas_vbatt - tb6612::vdrop;

    /* Measure the current wheel speeds via the encoders */
    sign = tb6612::FORWARD == motor_driver.GetDirection(R_MOTOR) ? 1 : -1;
    r_motor_ctrl_data.fb_rad_s = LpFilter(r_encoder.GetWheelSpeed() * sign, \
                                            r_motor_ctrl_data.fb_rad_s, \
                                            WHEEL_SPEED_FILT_ALPHA);

    sign = tb6612::FORWARD == motor_driver.GetDirection(L_MOTOR) ? 1 : -1;
    l_motor_ctrl_data.fb_rad_s = LpFilter(l_encoder.GetWheelSpeed() * sign, \
                                            l_motor_ctrl_data.fb_rad_s, \
                                            WHEEL_SPEED_FILT_ALPHA);
#if TUNE
        debug_out.printf("%d,%.2f,%.2f,%.2f,%.2f\n\r", \
                        t.read_us(), r_motor_ctrl_data.sp_rad_s, \
                        r_motor_ctrl_data.fb_rad_s, l_motor_ctrl_data.fb_rad_s, \
                        meas_vbatt);
#endif

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

    ThisThread::sleep_for(CYCLE_TIME*1000);
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
                                                max_vbatt, min_v);
#else
  /* Saturate the set points to be within the actuator voltage range */
  int8_t sign;
  sign = signbit(ctrl_data->sp_volts) ? -1 : 1;
  ctrl_data->u_volts  = fabs(ctrl_data->sp_volts) > min_v ? \
                        ctrl_data->sp_volts : sign * min_v;
  ctrl_data->u_volts  = fabs(ctrl_data->sp_volts) < max_vbatt ? \
                        ctrl_data->sp_volts : sign * max_vbatt;
#endif  // OPEN_LOOP

  /* Convert the actuation voltage to a percent duty cycle */
  ctrl_data->u_percent = (uint8_t)(fabs(ctrl_data->u_volts) * (100/max_vbatt));

  /* Determine direction */
  dir = signbit(ctrl_data->u_volts) ? tb6612::REVERSE : tb6612::FORWARD;

  /* Set the motor direction */
  motor_driver.SetDirection(ctrl_data->id, dir);
}

void GetWheelAngV(Wheel_Ang_V_T *dest) {
  if (dest) {
    dest->r = r_motor_ctrl_data.fb_rad_s;
    dest->l = l_motor_ctrl_data.fb_rad_s;
  }
}

void UpdateWheelAngV(float l_sp, float r_sp, bool reset_pid) {
  r_motor_ctrl_data.sp_rad_s = r_sp;
  l_motor_ctrl_data.sp_rad_s = l_sp;
  ctrl_active = true;

  if (reset_pid) {
    r_pid.Reset();
    l_pid.Reset();
  }
}

void StopMotors(void) {
  UpdateWheelAngV(0.0f, 0.0f, true);
  awaiting_stop = true;
}
