import odrive
import time

odrv0 = odrive.find_any()
print(f"Połączono! Vbus: {odrv0.vbus_voltage}V")

# Sprawdź konfigurację
print(f"vel_gain: {odrv0.axis0.controller.config.vel_gain}")
print(f"vel_integrator_gain: {odrv0.axis0.controller.config.vel_integrator_gain}")
print(f"vel_ramp_rate: {odrv0.axis0.controller.config.vel_ramp_rate}")
print(f"Motor pre_calibrated: {odrv0.axis0.motor.config.pre_calibrated}")
print(f"Encoder pre_calibrated: {odrv0.axis0.encoder.config.pre_calibrated}")

# Test sterowania
print("\nUruchamiam CLOSED_LOOP...")
odrv0.axis0.requested_state = 8
time.sleep(1)

print("Test prędkości 2 obr/s przez 10 sekund...")
odrv0.axis0.controller.input_vel = 1

for i in range(20):
    vel = odrv0.axis0.encoder.vel_estimate
    iq = odrv0.axis0.motor.current_control.Iq_measured
    vbus = odrv0.vbus_voltage
    ibus = odrv0.ibus
    print(f"[{i*0.5:4.1f}s] Vel: {vel:5.2f} obr/s | Iq: {iq:5.2f}A | "
          f"Vbus: {vbus:5.2f}V | Ibus: {ibus:5.2f}A | "
          f"Moc: {vbus*ibus:6.1f}W")
    time.sleep(0.5)

print("\nStop...")
odrv0.axis0.controller.input_vel = 0
time.sleep(2)
odrv0.axis0.requested_state = 1  # IDLE
print("Zakończono")

