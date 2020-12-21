#include "motor_controls.h"

#include <cmath>
#include <cstdint>

#include "battery_monitor.h"
#include "cmd.h"
#include "config.h"
#include "io_abstraction.h"
#include "lp_filter.h"
#include "mbed.h"
#include "pid.h"
#include "quad_encoder.h"
#include "tb6612.h"

#define TOLERANCE (0.0f)       // rad/s
#define STOP_THRESHOLD (0.5f)  // rad/s
#define VBATT_FILT_ALPHA (0.4f)

typedef struct
{
  tb6612::MotorId id;
  pid::PID* pid_ptr;
  float ke;           // DC motor speed constant
  float sp_rad_s;     // Desired angular velocity
  float fb_rad_s;     // Actual angular velocity
  float dt_s;         // Elapsed time between loops
  float sp_volts;     // DC motor setpoint voltage
  float fb_volts;     // DC motor feedback voltage
  float u_volts;      // DC motor actuation voltage
  uint8_t u_percent;  // DC motor actuation voltage as a % of max voltage
} CtrlData;

// Motor driver object
static tb6612::TB6612 motor_driver(MOTOR_A_PWM, MOTOR_B_PWM, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2,
                                   MOTOR_A_POLARITY, MOTOR_B_POLARITY);

// PID controller objects
static pid::PID l_pid(TOLERANCE);
static pid::PID r_pid(TOLERANCE);

// Wheel speed quadrature encoder objects
static quad_encoder::QuadEncoder r_encoder(R_ENCODER_B, PullNone, R_ENCODER_A, PullNone);
static quad_encoder::QuadEncoder l_encoder(L_ENCODER_A, PullNone, L_ENCODER_B, PullNone);

// Objects
static Mutex ctrl_data_mutex;
static Timer t;

// Variables
static us_timestamp_t last_t_us = 0;
static float dt_s = 0;
static float pulses_2_rpm = 0;
static float meas_vbatt = 0.0F;
static float max_vbatt = 0.0F;
static float wheel_speed_filt_alpha = 1.0F;
static CtrlData r_motor_ctrl_data;
static CtrlData l_motor_ctrl_data;
static bool ctrl_active = false;
static bool awaiting_stop = false;

static const uint8_t MAX_PERCENTAGE = 100;
static const float MIN_V = 0.0F;
static const float PULSES_2_REVS = (1.0F / PULSES_PER_REV) * 2 * 3.14159;
static const float TB6612_PWM_PERIOD_S = 0.0001;

// Static function declarations
static void runController(CtrlData* ctrl_data);

void initMotorControls(void)
{
  (void)memset(&r_motor_ctrl_data, 0, sizeof(r_motor_ctrl_data));
  (void)memset(&l_motor_ctrl_data, 0, sizeof(l_motor_ctrl_data));

  motor_driver.setPWMPeriod(TB6612_PWM_PERIOD_S);

  r_motor_ctrl_data.ke = R_Ke;
  l_motor_ctrl_data.ke = L_Ke;

  r_motor_ctrl_data.id = R_MOTOR;
  l_motor_ctrl_data.id = L_MOTOR;

  r_motor_ctrl_data.pid_ptr = &r_pid;
  l_motor_ctrl_data.pid_ptr = &l_pid;

  t.start();
}

void runMotorControls(void)
{
  while (true)
  {
    // Determine the max actuation voltage based on the vbatt measurement
    meas_vbatt = lpFilter(readBatteryVoltage(), meas_vbatt, VBATT_FILT_ALPHA);

    if (meas_vbatt < MAX_MOTOR_VOLTAGE)
    {
      max_vbatt = meas_vbatt - tb6612::vdrop;
    }
    else
    {
      max_vbatt = MAX_MOTOR_VOLTAGE - tb6612::vdrop;
    }

    // Compute the dt_s between executions
    us_timestamp_t current_t_us = t.read_high_resolution_us();

    if (0 != last_t_us)
    {
      r_motor_ctrl_data.dt_s = static_cast<uint32_t>(current_t_us - last_t_us) * US_2_S;
      l_motor_ctrl_data.dt_s = r_motor_ctrl_data.dt_s;
    }
    else
    {
      r_motor_ctrl_data.dt_s = CYCLE_TIME_MS * MS_2_S;
      l_motor_ctrl_data.dt_s = r_motor_ctrl_data.dt_s;
    }

    last_t_us = current_t_us;

    pulses_2_rpm = PULSES_2_REVS / r_motor_ctrl_data.dt_s;

    if (ctrl_data_mutex.trylock())
    {
      // Measure the current wheel speeds via the encoders
      r_motor_ctrl_data.fb_rad_s =
          lpFilter(r_encoder.getPulses() * pulses_2_rpm, r_motor_ctrl_data.fb_rad_s, wheel_speed_filt_alpha);

      l_motor_ctrl_data.fb_rad_s =
          lpFilter(l_encoder.getPulses() * pulses_2_rpm, l_motor_ctrl_data.fb_rad_s, wheel_speed_filt_alpha);

      ctrl_data_mutex.unlock();

      if (ctrl_active)
      {
        runController(&r_motor_ctrl_data);
        runController(&l_motor_ctrl_data);

        // Are we slowing down to stop?
        if (awaiting_stop && (fabs(r_motor_ctrl_data.fb_rad_s) < STOP_THRESHOLD) &&
            (fabs(l_motor_ctrl_data.fb_rad_s) < STOP_THRESHOLD))
        {
          motor_driver.freewheel();
          ctrl_active = false;
          awaiting_stop = false;
        }
        else
        {
          motor_driver.setDutyCycle(R_MOTOR, r_motor_ctrl_data.u_percent);
          motor_driver.setDutyCycle(L_MOTOR, l_motor_ctrl_data.u_percent);
        }
      }
      else
      {
        motor_driver.freewheel();
      }

      ThisThread::sleep_for(CYCLE_TIME_MS);
    }
    else
    {
      ThisThread::sleep_for(1);
    }
  }
}

static void runController(CtrlData* ctrl_data)
{
  // Compute the voltage set point
  ctrl_data->sp_volts = ctrl_data->sp_rad_s * ctrl_data->ke;

  // Compute the voltage feedback
  ctrl_data->fb_volts = ctrl_data->fb_rad_s * ctrl_data->ke;

#if 0 == OPEN_LOOP
  // Run the PID controller
  ctrl_data->u_volts = ctrl_data->pid_ptr->step(ctrl_data->sp_volts, \
                                                ctrl_data->fb_volts, \
                                                ctrl_data->dt_s, \
                                                max_vbatt, MIN_V);
#else
  // Saturate the set points to be within the actuator voltage range
  const int8_t sign = signbit(ctrl_data->sp_volts) ? -1 : 1;
  ctrl_data->u_volts = fabs(ctrl_data->sp_volts) < max_vbatt ? ctrl_data->sp_volts : sign * max_vbatt;
#endif  // OPEN_LOOP

  // Convert the actuation voltage to a percent duty cycle
  ctrl_data->u_percent = static_cast<uint8_t>(fabs(ctrl_data->u_volts) * (MAX_PERCENTAGE / meas_vbatt));

  // Determine direction
  const tb6612::Direction dir = signbit(ctrl_data->u_volts) ? tb6612::REVERSE : tb6612::FORWARD;

  // Set the motor direction
  motor_driver.setDirection(ctrl_data->id, dir);
}

void getWheelAngVSp(WheelAngV* dest)
{
  if (NULL != dest)
  {
    dest->r = r_motor_ctrl_data.sp_rad_s;
    dest->l = l_motor_ctrl_data.sp_rad_s;
  }
  else
  {
    MBED_ASSERT(false);
  }
}

void getWheelAngV(WheelAngV* dest)
{
  ctrl_data_mutex.lock();

  if (NULL != dest)
  {
    dest->r = r_motor_ctrl_data.fb_rad_s;
    dest->l = l_motor_ctrl_data.fb_rad_s;
  }
  else
  {
    MBED_ASSERT(false);
  }

  ctrl_data_mutex.unlock();
}

void updateMotorControllerInputs(const oscar_pi::cmd& cmd_msg)
{
  pid::Gains r_gains = { 0.0F };
  pid::Gains l_gains = { 0.0F };

  if (0 == cmd_msg.stop)
  {
    r_motor_ctrl_data.sp_rad_s = cmd_msg.r_wheel_sp;
    l_motor_ctrl_data.sp_rad_s = cmd_msg.l_wheel_sp;
  }
  else
  {
    r_motor_ctrl_data.sp_rad_s = 0.0F;
    l_motor_ctrl_data.sp_rad_s = 0.0F;
    awaiting_stop = true;
  }

  wheel_speed_filt_alpha = cmd_msg.wheel_speed_filt_alpha;

  r_gains.kp = cmd_msg.r_kp;
  r_gains.ki = cmd_msg.r_ki;
  r_gains.kd = cmd_msg.r_kd;
  r_pid.reset(&r_gains);

  l_gains.kp = cmd_msg.l_kp;
  l_gains.ki = cmd_msg.l_ki;
  l_gains.kd = cmd_msg.l_kd;
  l_pid.reset(&l_gains);

  ctrl_active = true;
}
