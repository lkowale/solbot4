#include "Drv8871.h"

// Static instance pointer for ISR access
Drv8871* Drv8871::instance = nullptr;

Drv8871::Drv8871(String _name) {
  instance = this;  // Set singleton for ISR
  name = _name;
  speed = 0;
  speed_rps = 0;
  speed_rpm = 0;
  output_rpm = 0;
  output_rps = 0;
  speed_goal = 0;
  speed_command = 0;
  target_output_rpm = 0;
  current = 0;
  vbus_voltage = 0;
  power = 0;
  fet_temp = 0;
  pwm = 0;
  position = 0;
  connected = true;  // Always connected (no feedback from DRV8871)
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

  // Hall sensor state
  pulse_count = 0;
  hall_direction = 1;
  last_speed_calc = 0;
  last_pulse_count = 0;
  output_rpm_filtered = 0;

  // PID state
  pid_integral = 0;
  pid_prev_error = 0;
  pid_output = 0;
}

void Drv8871::begin() {
  // Configure LEDC PWM for ESP32-S3
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to pins
  ledcAttachPin(DRV8871_IN1, PWM_CHANNEL_1);
  ledcAttachPin(DRV8871_IN2, PWM_CHANNEL_2);

  // Start with motor stopped
  brake();

  // Initialize Hall sensors
  initHallSensors();

  Serial.println("DRV8871 initialized");
  Serial.print("  IN1 pin: "); Serial.println(DRV8871_IN1);
  Serial.print("  IN2 pin: "); Serial.println(DRV8871_IN2);
  Serial.print("  Hall A pin: "); Serial.println(HALL_A_PIN);
  Serial.print("  Hall B pin: "); Serial.println(HALL_B_PIN);
  Serial.print("  Pulses/rev: "); Serial.println(PULSES_PER_REV);
  Serial.print("  PWM freq: "); Serial.print(PWM_FREQ); Serial.println(" Hz");
}

void Drv8871::set_speed(float goal_percent) {
  // Convert percentage (-100 to 100) to target output RPM
  target_output_rpm = (goal_percent / 100.0f) * MAX_OUTPUT_RPM;

  // Clamp to limits
  if (target_output_rpm > MAX_OUTPUT_RPM) target_output_rpm = MAX_OUTPUT_RPM;
  if (target_output_rpm < -MAX_OUTPUT_RPM) target_output_rpm = -MAX_OUTPUT_RPM;

  // Enable/disable motor based on target
  if (target_output_rpm == 0 && motorEnabled) {
    setMotorState(false);
    // Reset PID state when stopping
    pid_integral = 0;
    pid_prev_error = 0;
  } else if (target_output_rpm != 0 && !motorEnabled) {
    setMotorState(true);
  }

  speed_goal = target_output_rpm;  // For telemetry
}

void Drv8871::set_speed_rps(float goal_pwm) {
  // Clamp to limits
  if (goal_pwm > PWM_MAX) goal_pwm = PWM_MAX;
  if (goal_pwm < -PWM_MAX) goal_pwm = -PWM_MAX;

  // If speed is 0, disable motor
  // If speed is non-zero, enable motor
  if (goal_pwm == 0 && motorEnabled) {
    setMotorState(false);
  } else if (goal_pwm != 0 && !motorEnabled) {
    setMotorState(true);
  }

  speed_goal = goal_pwm;
  pwm = (int)goal_pwm;
}

void Drv8871::stop() {
  speed_goal = 0;
  speed_command = 0;
  pwm = 0;
  brake();
}

void Drv8871::move_forward(int pwm_value) {
  // Clamp PWM
  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
  if (pwm_value < 0) pwm_value = 0;

  float target = (float)pwm_value;

  // Apply direction
  if (direction == 1) {
    target = -target;
  }

  speed_goal = target;
  pwm = pwm_value;

  if (!motorEnabled) {
    setMotorState(true);
  }
}

void Drv8871::move_backward(int pwm_value) {
  // Clamp PWM
  if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
  if (pwm_value < 0) pwm_value = 0;

  float target = -(float)pwm_value;

  // Apply direction
  if (direction == 1) {
    target = -target;
  }

  speed_goal = target;
  pwm = -pwm_value;

  if (!motorEnabled) {
    setMotorState(true);
  }
}

void Drv8871::setMotorState(bool state) {
  if (state) {
    motorEnabled = true;
    Serial.println("Motor enabled");
  } else {
    brake();
    speed_command = 0;
    motorEnabled = false;
    Serial.println("Motor disabled (brake)");
  }
}

void Drv8871::update() {
  // Start timing
  update_start_us = micros();

  // Calculate actual speed from Hall sensors first (needed for PID)
  calculateSpeed();

  if (mode == SPEED) {
    // PID control mode - regulate PWM to achieve target_output_rpm
    // Use filtered speed for smoother control
    float error = target_output_rpm - output_rpm_filtered;

    // Proportional term
    float p_term = PID_KP * error;

    // Integral term with anti-windup
    pid_integral += error * ((float)pid_update_window / 1000.0f);
    if (pid_integral > PID_INTEGRAL_MAX) pid_integral = PID_INTEGRAL_MAX;
    if (pid_integral < -PID_INTEGRAL_MAX) pid_integral = -PID_INTEGRAL_MAX;
    float i_term = PID_KI * pid_integral;

    // Derivative term
    float d_term = PID_KD * (error - pid_prev_error) / ((float)pid_update_window / 1000.0f);
    pid_prev_error = error;

    // Calculate PID output
    pid_output = p_term + i_term + d_term;

    // Clamp to PWM limits
    if (pid_output > PWM_MAX) pid_output = PWM_MAX;
    if (pid_output < -PWM_MAX) pid_output = -PWM_MAX;

    speed_command = pid_output;
    pwm = (int)speed_command;

  } else {
    // PWM mode - ramp speed_command towards speed_goal
    float diff = speed_goal - speed_command;
    if (diff > PWM_ACCEL_LIMIT) {
      speed_command += PWM_ACCEL_LIMIT;
    } else if (diff < -PWM_ACCEL_LIMIT) {
      speed_command -= PWM_ACCEL_LIMIT;
    } else {
      speed_command = speed_goal;
    }
  }

  // Apply PWM if motor is enabled
  if (motorEnabled) {
    applyPwm((int)speed_command);
  }

  if (debug) {
    Serial.print("target: "); Serial.print(target_output_rpm);
    Serial.print(" out_rpm: "); Serial.print(output_rpm);
    Serial.print(" pwm: "); Serial.print(speed_command);
    Serial.print(" pid: "); Serial.print(pid_output);
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

void Drv8871::applyPwm(int pwm_value) {
  // DRV8871 control:
  // IN1 PWM, IN2 LOW  = Forward at IN1 duty cycle
  // IN1 LOW, IN2 PWM  = Reverse at IN2 duty cycle
  // Both LOW = Coast (motor freewheels)
  // Both HIGH = Brake (motor brakes)

  if (pwm_value > 0) {
    // Forward
    ledcWrite(PWM_CHANNEL_1, pwm_value);
    ledcWrite(PWM_CHANNEL_2, 0);
  } else if (pwm_value < 0) {
    // Reverse
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, -pwm_value);
  } else {
    // Stop (coast)
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
  }
}

void Drv8871::brake() {
  // Both pins low = coast mode
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
}

void Drv8871::initHallSensors() {
  // Configure Hall sensor pins as inputs with pull-up
  pinMode(HALL_A_PIN, INPUT_PULLUP);
  pinMode(HALL_B_PIN, INPUT_PULLUP);

  // Attach interrupts to both Hall sensors
  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallB_ISR, CHANGE);

  last_speed_calc = millis();
  pulse_count = 0;
  last_pulse_count = 0;
}

void IRAM_ATTR Drv8871::hallA_ISR() {
  if (instance == nullptr) return;

  // Read both Hall sensors
  int hallA = digitalRead(HALL_A_PIN);
  int hallB = digitalRead(HALL_B_PIN);

  // Determine direction based on quadrature encoding
  // When Hall A changes, check Hall B to determine direction
  if (hallA == hallB) {
    instance->pulse_count++;
    instance->hall_direction = 1;  // CW
  } else {
    instance->pulse_count--;
    instance->hall_direction = -1;  // CCW
  }
}

void IRAM_ATTR Drv8871::hallB_ISR() {
  if (instance == nullptr) return;

  // Read both Hall sensors
  int hallA = digitalRead(HALL_A_PIN);
  int hallB = digitalRead(HALL_B_PIN);

  // Determine direction based on quadrature encoding
  // When Hall B changes, check Hall A to determine direction
  if (hallA != hallB) {
    instance->pulse_count++;
    instance->hall_direction = 1;  // CW
  } else {
    instance->pulse_count--;
    instance->hall_direction = -1;  // CCW
  }
}

void Drv8871::calculateSpeed() {
  unsigned long now = millis();
  unsigned long dt = now - last_speed_calc;

  // Only calculate if enough time has passed (avoid division by zero)
  if (dt < 10) return;

  // Get pulse count atomically
  noInterrupts();
  long current_count = pulse_count;
  interrupts();

  // Calculate pulses since last measurement
  long delta_pulses = current_count - last_pulse_count;

  // Calculate speed
  // RPM = (pulses / pulses_per_rev) * (60000 / dt_ms)
  // Using 4x counting for quadrature (both edges, both channels)
  float revolutions = (float)abs(delta_pulses) / (float)(PULSES_PER_REV * 4);
  float seconds = (float)dt / 1000.0f;

  speed_rps = revolutions / seconds;
  speed_rpm = speed_rps * 60.0f;

  // Calculate gear output speed (raw)
  float raw_output_rpm = speed_rpm / GEAR_RATIO;
  output_rps = speed_rps / GEAR_RATIO;

  // Apply direction sign to raw value
  if (delta_pulses < 0) {
    speed_rps = -speed_rps;
    speed_rpm = -speed_rpm;
    raw_output_rpm = -raw_output_rpm;
    output_rps = -output_rps;
  }

  // Apply EMA filter to output_rpm for smoother PID control
  output_rpm_filtered = SPEED_FILTER_ALPHA * raw_output_rpm + (1.0f - SPEED_FILTER_ALPHA) * output_rpm_filtered;
  output_rpm = raw_output_rpm;  // Keep raw value for telemetry

  // Validate values (prevent NaN/Inf from propagating)
  if (isnan(output_rpm) || isinf(output_rpm)) output_rpm = 0;
  if (isnan(output_rpm_filtered) || isinf(output_rpm_filtered)) output_rpm_filtered = 0;
  if (isnan(output_rps) || isinf(output_rps)) output_rps = 0;

  // Speed as percentage based on max gear output
  speed = (fabsf(output_rpm_filtered) / MAX_OUTPUT_RPM) * 100.0f;
  if (speed > 100.0f) speed = 100.0f;
  if (delta_pulses < 0) speed = -speed;

  // Store for next calculation
  last_pulse_count = current_count;
  last_speed_calc = now;
}
