#ifndef DRV8871_H
#define DRV8871_H

#include "Arduino.h"

// DRV8871 pin configuration (ESP32-S3)
#define DRV8871_IN1 5   // PWM pin for forward
#define DRV8871_IN2 6   // PWM pin for reverse

// Hall sensor pins
#define HALL_A_PIN 1    // Hall sensor A
#define HALL_B_PIN 2    // Hall sensor B

// Hall sensor configuration
#define PULSES_PER_REV 11  // Pulses per motor revolution (adjust for your motor)
#define GEAR_RATIO 61.2f   // Motor RPM / Gear output RPM (6120/100)
#define MAX_OUTPUT_RPM 100.0f  // Maximum gear output RPM
#define SPEED_FILTER_ALPHA 0.15f  // EMA filter for speed (lower = smoother)

// PID configuration
#define PID_KP 4.0f      // Proportional gain
#define PID_KI 3.0f      // Integral gain
#define PID_KD 0.0f      // Derivative off (noise sensitive)
#define PID_INTEGRAL_MAX 200.0f  // Anti-windup limit (must be high enough to reach max PWM)

// PWM configuration
#define PWM_FREQ 20000      // 20kHz PWM frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)
#define PWM_CHANNEL_1 0     // LEDC channel for IN1
#define PWM_CHANNEL_2 1     // LEDC channel for IN2

// Speed limits (PWM value 0-255)
#define PWM_MAX 255
#define PWM_MIN 0

// Acceleration limit per update cycle (100ms)
// 25 means it takes ~1 second to go from 0 to 255
#define PWM_ACCEL_LIMIT 25

class Drv8871 {

public:
  String name;

  // Motor modes
  enum move_mode { PWM, SPEED };

  // Direction: 0 = CW (normal), 1 = CCW (reversed)
  int direction;
  int mode;

  // Motor state
  float speed;          // Current speed from Hall sensors (0-100%)
  float speed_rps;      // Speed in revolutions per second (from Hall sensors)
  float speed_rpm;      // Speed in RPM (from Hall sensors)
  float output_rpm;     // Gear output speed in RPM
  float output_rps;     // Gear output speed in RPS
  float speed_goal;     // Target speed (PWM value, -255 to 255) - used in PWM mode
  float speed_command;  // Ramped speed (PWM value)
  float target_output_rpm;  // Target gear output RPM for PID control
  float current;        // Placeholder (DRV8871 doesn't provide current feedback)
  float vbus_voltage;   // Placeholder
  float power;          // Placeholder
  float fet_temp;       // Placeholder
  int pwm;              // Current PWM value (-255 to 255)
  int position;         // Placeholder

  bool connected;       // Always true for DRV8871 (no feedback)
  unsigned long last_contact;
  int debug;

  // Hall sensor state
  volatile long pulse_count;        // Total pulse count (can be negative for direction)
  volatile int hall_direction;      // Detected direction from Hall sensors: 1=CW, -1=CCW
  unsigned long last_speed_calc;    // Timestamp for speed calculation
  long last_pulse_count;            // Previous pulse count for speed calculation
  float output_rpm_filtered;        // EMA filtered output RPM for PID

  // PID state
  float pid_integral;     // Integral accumulator
  float pid_prev_error;   // Previous error for derivative
  float pid_output;       // PID output (for debugging/telemetry)

  // Update timing configuration
  unsigned long pid_update_window;  // Update interval in ms

  // Timing for update() execution measurement
  unsigned long update_start_us;
  float update_time_ema_us;
  float update_time_alpha;
  float update_time_percent;

  Drv8871(String _name);
  void begin();

  // Speed control
  void set_speed(float goal_rpm);      // Set speed as percentage (-100 to 100)
  void set_speed_rps(float goal_rps);  // Set speed directly as PWM (-255 to 255)
  void stop();

  // Direction-aware movement (for PWM mode)
  void move_forward(int pwm_value);
  void move_backward(int pwm_value);

  // Motor state
  void setMotorState(bool state);

  // Main update loop
  void update();

  // Hall sensor methods
  void initHallSensors();
  void calculateSpeed();

  // Static ISR functions (needed for attachInterrupt)
  static void IRAM_ATTR hallA_ISR();
  static void IRAM_ATTR hallB_ISR();
  static Drv8871* instance;  // Singleton for ISR access

private:
  bool motorEnabled;

  // Apply PWM to DRV8871
  void applyPwm(int pwm_value);

  // Brake mode (both pins low)
  void brake();
};

#endif
