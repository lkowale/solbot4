# MQTT & LED Summary 🔧

This is a concise reference for the project MQTT topics (commands and telemetry) and the RGB LED status meanings.

> Device name prefix: `deviceName` is set in code (example: `FR`). All topics are prefixed by this.

---

## MQTT Commands (subscribed topics) ✅

All command messages are JSON objects expected to contain a `data` field.

- `deviceName/pwm`  
  - Payload: `{ "data": <int> }` where value is PWM in range 0..255 (signed in handling).  
  - Behavior: Sets `servo.mode = PWM`. If `data > 0` -> `move_forward(data)`. If `data <= 0` -> `move_backward(-data)`.

- `deviceName/set_speed`  
  - Payload: `{ "data": <int> }` where value is **speed in RPM**.  
  - Behavior: Sets `servo.mode = SPEED` and calls `servo.set_speed(data)`.

- `deviceName/direction`  
  - Payload: `{ "data": <int> }` where `0 = CW`, `1 = CCW`.  
  - Behavior: Sets `servo.direction = data`.

---

## MQTT Telemetry (published topics, ~250 ms) 📡

The device publishes these topics every ~250 ms (topic names use `deviceName/` prefix):

- `deviceName/position` — position (placeholder; ODrive velocity mode may not track position).
- `deviceName/speed` — speed (published value from `servo.speed`).
- `deviceName/pwm_set` — last PWM (compatibility).
- `deviceName/current` — current from ODrive, **EMA filtered** (3 decimal places).
- `deviceName/power` — bus power (W?), 2 decimal places.
- `deviceName/vbus` — bus voltage (V), 2 decimal places.
- `deviceName/fet_temp` — FET temperature (°C), 1 decimal place.
- `deviceName/servo_occupancy` — servo update occupancy (percentage of update window), 2 decimals.
- `deviceName/loop_time_percent` — loop time percent relative to event interval, 2 decimals.
- `deviceName/connected` — reported servo connection state (`1` = connected, `0` = not connected).
- `deviceName/speed_goal` — speed goal (comment indicates rev/s, published with 3 decimal places).

---

## RGB LED Status (colors and behavior) 💡

- Startup/test: **Solid red** for 500 ms (startup flash).  
  - RGB: `led.Color(255, 0, 0)`

- While connecting to Wi‑Fi: **Flashing red** at 250 ms on/off.

- When not connected to MQTT: **Flashing green** at 1 Hz (500 ms on/off).  
  - RGB: `led.Color(0, 255, 0)` (blinks)

- When connected to MQTT: **Solid green** (steady).  
  - RGB: `led.Color(0, 255, 0)`

- Off state: `led.Color(0, 0, 0)`

Note: LED brightness is set to half (`led.setBrightness(128)`).

---

## Notes & Tips 💡

- Commands are parsed using ArduinoJson; if the payload is not valid JSON a deserialize error is logged.
- Topic strings are constructed by: `strcpy(str, deviceName); strcat(str, "/<topic>")`.
- `deviceName` is defined in `src/main.cpp` (example: `FR`). Adjust topics accordingly for multiple devices.

---

If you want, I can: add example MQTT payloads, include a small publish/subscribe test script, or place this summary into the `docs/` folder. ✅
