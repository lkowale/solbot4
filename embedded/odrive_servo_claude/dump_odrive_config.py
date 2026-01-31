#!/usr/bin/env python3
"""
Dump selected ODrive configuration fields to JSON for comparison between boards.
Usage:
  - Unplug one board and run for Board A, then swap and run for Board B; or
  - If you know the serial number, pass with `--serial SERIAL` to target that board.

Example:
  python3 dump_odrive_config.py --out odrvA.json
  python3 dump_odrive_config.py --out odrvB.json
  diff -u odrvA.json odrvB.json
"""

import odrive
import time
import argparse
import json
from odrive.utils import dump_errors

parser = argparse.ArgumentParser(description='Dump ODrive configuration to JSON')
parser.add_argument('--serial', help='ODrive serial number (optional)')
parser.add_argument('--out', help='Output JSON filename', default=None)
parser.add_argument('--timeout', type=float, default=10.0, help='Find timeout seconds')
args = parser.parse_args()

def safe_get(obj, path, default=None):
    try:
        cur = obj
        for p in path.split('.'):
            cur = getattr(cur, p)
        return cur
    except Exception:
        return default

print('Searching for ODrive...')
try:
    if args.serial:
        odrv = odrive.find_any(serial_number=args.serial)
    else:
        odrv = odrive.find_any(timeout=args.timeout)
except TypeError:
    # Some odrive versions don't accept kwargs for find_any
    try:
        odrv = odrive.find_any()
    except Exception as e:
        print('Failed to find ODrive:', e)
        raise SystemExit(1)
except Exception as e:
    print('Failed to find ODrive:', e)
    raise SystemExit(1)

print('Connected. vbus:', odrv.vbus_voltage)

def read_axis(axis):
    motor_fields = [
        'motor_type','pole_pairs','current_lim','current_lim_margin','calibration_current',
        'resistance_calib_max_voltage','requested_current_range','phase_resistance','phase_inductance',
        'pre_calibrated','torque_constant','torque_lim','inverter_temp_limit_lower',
        'inverter_temp_limit_upper','acim_gain_min_flux','acim_autoflux_min_Id'
    ]
    encoder_fields = [
        'mode','cpr','bandwidth','enable_phase_interpolation','pre_calibrated','is_ready',
        'phase_offset','phase_offset_float','direction'
    ]
    controller_fields = [
        'control_mode','vel_limit','vel_gain','vel_integrator_gain','vel_ramp_rate',
        'input_mode','input_filter_bandwidth','vel_integrator_limit',
        'enable_vel_limit','enable_overspeed_error','enable_torque_mode_vel_limit',
        'pos_gain','torque_ramp_rate'
    ]
    axis_config_fields = [
        'enable_watchdog','startup_motor_calibration','startup_encoder_offset_calibration','startup_closed_loop_control'
    ]

    data = {}
    data['state'] = safe_get(axis, 'current_state')
    data['requested_state'] = safe_get(axis, 'requested_state')

    data['motor'] = {f: safe_get(axis, f'motor.config.{f}') for f in motor_fields}
    data['encoder'] = {f: safe_get(axis, f'encoder.config.{f}') for f in encoder_fields}
    # also report a few live values
    data['encoder_live'] = {
        'shadow_count': safe_get(axis, 'encoder.shadow_count'),
        'pos_estimate': safe_get(axis, 'encoder.pos_estimate')
    }
    data['controller'] = {f: safe_get(axis, f'controller.config.{f}') for f in controller_fields}
    data['axis_config'] = {f: safe_get(axis, f'config.{f}') for f in axis_config_fields}

    # errors (numeric codes / enums)
    data['errors'] = {
        'axis': safe_get(axis, 'error'),
        'motor': safe_get(axis, 'motor.error'),
        'encoder': safe_get(axis, 'encoder.error'),
        'controller': safe_get(axis, 'controller.error')
    }
    # current controller gains (auto-calculated from calibration)
    data['current_control'] = {
        'p_gain': safe_get(axis, 'motor.current_control.p_gain'),
        'i_gain': safe_get(axis, 'motor.current_control.i_gain'),
        'bandwidth': safe_get(axis, 'motor.config.current_control_bandwidth')
    }
    # sensorless estimator (may affect behavior even with Hall)
    data['sensorless'] = {
        'pm_flux_linkage': safe_get(axis, 'sensorless_estimator.config.pm_flux_linkage'),
        'observer_gain': safe_get(axis, 'sensorless_estimator.config.observer_gain'),
    }
    # anticogging
    data['anticogging'] = {
        'anticogging_enabled': safe_get(axis, 'controller.config.anticogging.anticogging_enabled'),
        'calib_anticogging': safe_get(axis, 'controller.config.anticogging.calib_anticogging'),
        'pre_calibrated': safe_get(axis, 'controller.config.anticogging.pre_calibrated'),
    }
    return data

conf = {
    'serial_number': safe_get(odrv, 'serial_number'),
    'fw_version': f"{safe_get(odrv, 'fw_version_major')}.{safe_get(odrv, 'fw_version_minor')}.{safe_get(odrv, 'fw_version_revision')}",
    'hw_version': f"{safe_get(odrv, 'hw_version_major')}.{safe_get(odrv, 'hw_version_minor')}",
    'vbus_voltage': safe_get(odrv, 'vbus_voltage'),
    # System config that could differ between boards
    'system_config': {
        'brake_resistance': safe_get(odrv, 'config.brake_resistance'),
        'enable_brake_resistor': safe_get(odrv, 'config.enable_brake_resistor'),
        'dc_bus_undervoltage_trip_level': safe_get(odrv, 'config.dc_bus_undervoltage_trip_level'),
        'dc_bus_overvoltage_trip_level': safe_get(odrv, 'config.dc_bus_overvoltage_trip_level'),
        'dc_max_positive_current': safe_get(odrv, 'config.dc_max_positive_current'),
        'dc_max_negative_current': safe_get(odrv, 'config.dc_max_negative_current'),
        'max_regen_current': safe_get(odrv, 'config.max_regen_current'),
    },
    'axis0': read_axis(odrv.axis0),
    'axis1': read_axis(odrv.axis1)
}

# Try to capture textual dump of errors (best-effort; dump_errors prints to stdout)
print('\nDumping ODrive errors (human-readable):')
dump_errors(odrv)

if not args.out:
    sn = conf['serial_number'] or int(time.time())
    outname = f'odrv_{sn}.json'
else:
    outname = args.out

with open(outname, 'w') as f:
    json.dump(conf, f, indent=2, sort_keys=True)

print(f"Configuration written to {outname}")
print('Now run the script again for the other board and run:')
print(f"  diff -u {outname} other_odrv.json")
