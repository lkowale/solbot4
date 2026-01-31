import odrive
from odrive.utils import dump_errors
import time

print("Connecting to ODrive...")
odrv0 = odrive.find_any()
print(f"Connected. Vbus: {odrv0.vbus_voltage} V")

axis = odrv0.axis0
print("--- CONFIG ---")
print(f"motor.pole_pairs = {axis.motor.config.pole_pairs}")
print(f"encoder.mode = {axis.encoder.config.mode}")
print(f"encoder.cpr = {axis.encoder.config.cpr}")
print(f"encoder.pre_calibrated = {axis.encoder.config.pre_calibrated}")
print("--- ERRORS ---")
dump_errors(odrv0)

print("\nNow streaming encoder counts. Rotate the motor slowly by hand for ~10 seconds.")
print("Watching `shadow_count` and `pos_estimate`. Press Ctrl+C to stop early.")

try:
    t0 = time.time()
    prev = None
    while time.time() - t0 < 12:
        sc = axis.encoder.shadow_count
        pe = axis.encoder.pos_estimate
        print(f"shadow_count: {sc}, pos_estimate: {pe}")
        if prev is not None and abs(sc - prev) > 0:
            pass
        prev = sc
        time.sleep(0.25)
except KeyboardInterrupt:
    print("Stopped by user")

print("\nWhen you rotate the motor one full mechanical revolution, note the change in `shadow_count`.")
print("Set `encoder.config.cpr` to that value (or to `6 * motor.pole_pairs` for hall encoders).")
print("If no counts change, check Hall sensor wiring, supply, and `encoder.config.mode`.")
