#!/usr/bin/env python3
"""
Hall Sensor Diagnostic Tool for ODrive
Monitors Hall sensor state in real-time to detect wiring issues.

Valid Hall states for 3-phase: 1,2,3,4,5,6 (never 0 or 7)
Sequence should be: 5->1->3->2->6->4->5 (or reverse)
"""

import odrive
import time
import sys

print("Searching for ODrive...")
odrv = odrive.find_any()
print(f"Connected! Serial: {odrv.serial_number}")
print(f"Vbus: {odrv.vbus_voltage:.1f}V")

# Check for errors first
print("\n=== Current Errors ===")
from odrive.utils import dump_errors
dump_errors(odrv)

print("\n=== Hall Sensor Configuration ===")
enc = odrv.axis0.encoder
print(f"Encoder mode: {enc.config.mode} (1=HALL)")
print(f"CPR: {enc.config.cpr} (should be 6 * pole_pairs = {6 * odrv.axis0.motor.config.pole_pairs})")
print(f"Pole pairs: {odrv.axis0.motor.config.pole_pairs}")
print(f"Encoder ready: {enc.is_ready}")
print(f"Pre-calibrated: {enc.config.pre_calibrated}")

print("\n=== Hall Sensor Live Monitor ===")
print("Slowly rotate the motor BY HAND and watch the Hall state.")
print("Valid states: 1,2,3,4,5,6 (if you see 0 or 7, wiring is wrong)")
print("Expected sequence: 5->1->3->2->6->4->5 (or reverse)")
print("Press Ctrl+C to stop\n")

# Track state changes
prev_state = None
state_history = []
invalid_states = set()
transition_count = 0

VALID_STATES = {1, 2, 3, 4, 5, 6}
# Valid transitions for forward rotation
VALID_TRANSITIONS_FWD = {(5,1), (1,3), (3,2), (2,6), (6,4), (4,5)}
# Valid transitions for reverse rotation
VALID_TRANSITIONS_REV = {(1,5), (3,1), (2,3), (6,2), (4,6), (5,4)}

try:
    start_time = time.time()
    while True:
        # Read Hall state
        hall_state = odrv.axis0.encoder.hall_state
        shadow_count = odrv.axis0.encoder.shadow_count
        pos = odrv.axis0.encoder.pos_estimate
        vel = odrv.axis0.encoder.vel_estimate

        if hall_state != prev_state:
            transition_count += 1
            elapsed = time.time() - start_time

            # Check validity
            valid_marker = "✓" if hall_state in VALID_STATES else "✗ INVALID!"
            if hall_state not in VALID_STATES:
                invalid_states.add(hall_state)

            # Check transition validity
            trans_marker = ""
            if prev_state is not None:
                trans = (prev_state, hall_state)
                if trans in VALID_TRANSITIONS_FWD:
                    trans_marker = " [FWD]"
                elif trans in VALID_TRANSITIONS_REV:
                    trans_marker = " [REV]"
                else:
                    trans_marker = " [SKIP/ERROR]"

            print(f"[{elapsed:6.2f}s] Hall: {hall_state} {valid_marker}{trans_marker} | "
                  f"shadow: {shadow_count:6d} | pos: {pos:7.3f} | vel: {vel:6.2f}")

            state_history.append(hall_state)
            if len(state_history) > 50:
                state_history.pop(0)

            prev_state = hall_state

        time.sleep(0.01)  # 100Hz polling

except KeyboardInterrupt:
    print("\n\n=== Summary ===")
    print(f"Total transitions: {transition_count}")
    print(f"States observed: {sorted(set(state_history))}")

    if invalid_states:
        print(f"\n⚠️  INVALID STATES DETECTED: {invalid_states}")
        print("This indicates Hall sensor wiring problems:")
        if 0 in invalid_states:
            print("  - State 0: One or more Hall sensors not connected or stuck LOW")
        if 7 in invalid_states:
            print("  - State 7: One or more Hall sensors stuck HIGH")
    else:
        print("\n✓ All Hall states valid (1-6)")

    # Check if we got all 6 states
    unique_states = set(state_history) & VALID_STATES
    if len(unique_states) == 6:
        print("✓ All 6 Hall states observed - wiring looks correct")
    elif len(unique_states) > 0:
        missing = VALID_STATES - unique_states
        print(f"⚠️  Only {len(unique_states)} states observed. Missing: {missing}")
        print("   (Rotate motor more to see all states, or check wiring)")

    # Show sequence
    if len(state_history) >= 6:
        print(f"\nLast sequence: {' -> '.join(map(str, state_history[-12:]))}")

print("\nDone.")
