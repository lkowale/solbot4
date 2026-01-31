#ifndef WSERVO_H
#define WSERVO_H

#include <HardwareSerial.h>
#include "Arduino.h"

// ODrive UART configuration
#define ODRIVE_RX 2
#define ODRIVE_TX 1
#define ODRIVE_UART_NUM 1
#define ODRIVE_BAUD 115200

// Speed limits in rev/s
#define SPEED_LIMIT_MAX 5.0f
#define SPEED_LIMIT_MIN -5.0f

// Acceleration limit in rev/s per update cycle (100ms)
// 0.25 means it takes 4 seconds to go from 0 to 5 rps
#define ACCEL_LIMIT 0.25f

class Wservo {

public:
  String name;

  // Motor modes
  enum move_mode { PWM, SPEED };

  // Direction: 0 = CW (normal), 1 = CCW (reversed)
  int direction;
  int mode;

  HardwareSerial* odriveSerial;

  // Motor state from ODrive
  float speed;          // Current speed in RPM (from ODrive)
  float speed_rps;      // Current speed in rev/s (from ODrive)
  float speed_goal;     // Target speed in rev/s (user setpoint)
  float speed_command;  // Ramped speed sent to ODrive (rev/s)
  float current;        // DC bus current from ODrive (Amps)
  float vbus_voltage;   // DC bus voltage from ODrive (Volts)
  float power;          // Bus power = vbus * ibus (Watts)
  float fet_temp;       // FET temperature from ODrive (Celsius)
  int pwm;              // PWM value (0-255) for compatibility
  int position;         // Position (placeholder, ODrive doesn't use position in velocity mode)

  bool connected;
  unsigned long last_contact;
  int debug;

  // Update timing configuration
  unsigned long pid_update_window;  // Update interval in ms (named for compatibility)

  // Timing for update() execution measurement
  unsigned long update_start_us;
  float update_time_ema_us;
  float update_time_alpha;
  float update_time_percent;

  Wservo(String _name);
  void begin();

  // Speed control
  void set_speed(float goal_rpm);      // Set speed in RPM
  void set_speed_rps(float goal_rps);  // Set speed in rev/s
  void stop();

  // Direction-aware movement (for PWM mode compatibility)
  void move_forward(int pwm_value);
  void move_backward(int pwm_value);

  // Motor state
  void setMotorState(bool state);

  // Main update loop
  void update();

private:
  bool motorEnabled;

  // ODrive communication
  void odriveWrite(const char* cmd);
  void odriveClearBuffer();
  bool odriveReadResponse(float &value, unsigned long timeout_ms = 100);
  void readODriveData();

  // Convert PWM (0-255) to speed (-5 to 5 rps)
  float pwmToSpeed(int pwm_value);
};

#endif
