#include "Wservo.h"

Wservo::Wservo(String _name) {
  name = _name;
  speed = 0;
  speed_rps = 0;
  speed_goal = 0;
  speed_command = 0;
  current = 0;
  vbus_voltage = 0;
  power = 0;
  fet_temp = 0;
  pwm = 0;
  position = 0;
  connected = false;
  last_contact = millis();
  direction = 0;  // 0 = CW (normal)
  mode = SPEED;
  motorEnabled = false;
  debug = 0;

  // Update timing
  pid_update_window = 100;  // 100ms update interval
  update_start_us = 0;
  update_time_ema_us = 0.0f;
  update_time_alpha = 0.12f;
  update_time_percent = 0.0f;
}

void Wservo::begin() {
  // Initialize ODrive UART
  odriveSerial = new HardwareSerial(ODRIVE_UART_NUM);
  odriveSerial->begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
  delay(500);
  Serial.println("ODrive UART initialized");

  setMotorState(false);
}

void Wservo::set_speed(float goal_rpm) {
  // Convert RPM to rev/s and set
  float goal_rps = goal_rpm / 60.0f;
  set_speed_rps(goal_rps);
}

void Wservo::set_speed_rps(float goal_rps) {
  // Clamp speed to limits (store absolute value, direction applied in update())
  if (goal_rps > SPEED_LIMIT_MAX) goal_rps = SPEED_LIMIT_MAX;
  if (goal_rps < SPEED_LIMIT_MIN) goal_rps = SPEED_LIMIT_MIN;

  // If speed is 0, put motor in IDLE for free movement
  // If speed is non-zero, ensure motor is in CLOSED_LOOP_CONTROL
  if (goal_rps == 0 && motorEnabled) {
    setMotorState(false);  // IDLE - motor can move freely
  } else if (goal_rps != 0 && !motorEnabled) {
    setMotorState(true);   // CLOSED_LOOP_CONTROL - motor actively controlled
  }

  speed_goal = goal_rps;

  // Update pwm for compatibility (map -5..5 rps to -255..255)
  pwm = (int)(speed_goal / SPEED_LIMIT_MAX * 255.0f);
}

void Wservo::stop() {
  speed_goal = 0;
  speed_command = 0;  // Immediate stop, no ramping
  pwm = 0;
  odriveWrite("v 0 0\n");
}

float Wservo::pwmToSpeed(int pwm_value) {
  // Map PWM (0-255) to speed (0-5 rps)
  float speed_rps = ((float)pwm_value / 255.0f) * SPEED_LIMIT_MAX;
  return speed_rps;
}

void Wservo::move_forward(int pwm_value) {
  // Clamp PWM
  if (pwm_value > 255) pwm_value = 255;
  if (pwm_value < 0) pwm_value = 0;

  float speed_rps = pwmToSpeed(pwm_value);

  // Apply direction
  if (direction == 1) {
    speed_rps = -speed_rps;
  }

  speed_goal = speed_rps;
  pwm = pwm_value;
}

void Wservo::move_backward(int pwm_value) {
  // Clamp PWM
  if (pwm_value > 255) pwm_value = 255;
  if (pwm_value < 0) pwm_value = 0;

  float speed_rps = -pwmToSpeed(pwm_value);

  // Apply direction
  if (direction == 1) {
    speed_rps = -speed_rps;
  }

  speed_goal = speed_rps;
  pwm = -pwm_value;
}

void Wservo::setMotorState(bool state) {
  if (state) {
    // Enable motor: set to closed loop control
    odriveWrite("w axis0.requested_state 8\n");  // 8 = CLOSED_LOOP_CONTROL
    delay(100);
    odriveWrite("v 0 0\n");  // Set initial velocity to 0
    motorEnabled = true;
    Serial.println("Motor enabled (closed loop)");
  } else {
    // Disable motor: set to idle (motor can be moved freely by hand)
    odriveWrite("w axis0.requested_state 1\n");  // 1 = IDLE
    speed_command = 0;  // Reset ramped speed for clean restart
    motorEnabled = false;
    Serial.println("Motor disabled (idle - free movement)");
  }
}

void Wservo::update() {
  // Start timing
  update_start_us = micros();

  // Ramp speed_command towards speed_goal
  float diff = speed_goal - speed_command;
  if (diff > ACCEL_LIMIT) {
    speed_command += ACCEL_LIMIT;
  } else if (diff < -ACCEL_LIMIT) {
    speed_command -= ACCEL_LIMIT;
  } else {
    speed_command = speed_goal;
  }

  // Apply direction: 0 = CW (positive), 1 = CCW (negative)
  float directed_speed = (direction == 0) ? speed_command : -speed_command;

  // Send ramped speed command to ODrive
  if (motorEnabled) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "v 0 %.3f\n", directed_speed);
    odriveWrite(cmd);
  }

  // Read ODrive data (velocity and current)
  readODriveData();

  // Calculate power from bus voltage and current
  power = vbus_voltage * current;

  if (debug) {
    Serial.print("speed_goal: "); Serial.print(speed_goal);
    Serial.print(" speed_cmd: "); Serial.print(speed_command);
    Serial.print(" speed_rps: "); Serial.print(speed_rps);
    Serial.print(" current: "); Serial.print(current);
    Serial.print(" power: "); Serial.print(power);
    Serial.println();
  }

  // End timing and update EMA
  unsigned long dur_us = micros() - update_start_us;
  if (update_time_ema_us <= 0.0f) {
    update_time_ema_us = (float)dur_us;
  } else {
    update_time_ema_us = update_time_alpha * (float)dur_us + (1.0f - update_time_alpha) * update_time_ema_us;
  }

  // Calculate percentage of update window
  float avg_ms = update_time_ema_us / 1000.0f;
  float update_window = (float)pid_update_window;
  if (update_window > 0)
    update_time_percent = (avg_ms / update_window) * 100.0f;
  else
    update_time_percent = 0.0f;
}

void Wservo::odriveWrite(const char* cmd) {
  if (odriveSerial) {
    odriveSerial->print(cmd);
    delay(2);
  }
}

void Wservo::odriveClearBuffer() {
  if (odriveSerial) {
    while (odriveSerial->available()) {
      odriveSerial->read();
    }
  }
}

bool Wservo::odriveReadResponse(float &value, unsigned long timeout_ms) {
  if (!odriveSerial) return false;

  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (odriveSerial->available()) {
      value = odriveSerial->parseFloat();
      last_contact = millis();
      connected = true;
      return true;
    }
    delay(1);
  }
  return false;
}

void Wservo::readODriveData() {
  static int readCycle = 0;
  readCycle++;

  // Read velocity from ODrive (every cycle)
  odriveClearBuffer();
  odriveWrite("r axis0.encoder.vel_estimate\n");
  float vel = 0;
  if (odriveReadResponse(vel, 10)) {
    speed_rps = vel;
    speed = vel * 60.0f;
  }
  odriveClearBuffer();

  // Read DC bus current from ODrive (every cycle)
  odriveClearBuffer();
  odriveWrite("r ibus\n");
  float ibus = 0;
  if (odriveReadResponse(ibus, 10)) {
    current = ibus;
  }
  odriveClearBuffer();

  // Read DC bus voltage less frequently (every 10 cycles = 1 second)
  if (readCycle % 10 == 0) {
    odriveClearBuffer();
    odriveWrite("r vbus_voltage\n");
    float vbus = 0;
    if (odriveReadResponse(vbus, 10)) {
      vbus_voltage = vbus;
    }
    odriveClearBuffer();

    // Read FET temperature (every 10 cycles = 1 second)
    odriveClearBuffer();
    odriveWrite("r axis0.fet_thermistor.temperature\n");
    float temp = 0;
    if (odriveReadResponse(temp, 10)) {
      fet_temp = temp;
    }
    odriveClearBuffer();
  }

  // Check connection status
  if (millis() - last_contact > 1000) {
    connected = false;
  }
}
