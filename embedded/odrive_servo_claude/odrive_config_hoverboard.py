# === KONFIGURACJA ODRIVE DLA SILNIKA BLDC HOVERBOARD 36V ===
# Data: 2026-01-13
# Silnik: BLDC 36V z hoverboardu z Hall sensors
# Zasilanie: 36V 14A (ograniczony prąd)

import odrive
from odrive.enums import *

# Znajdź ODrive
print("Szukam ODrive...")
odrv0 = odrive.find_any()
print(f"Znaleziono ODrive, Vbus: {odrv0.vbus_voltage}V")

# === WYŁĄCZ AXIS1 (nie używany) ===
print("Wyłączanie axis1...")
odrv0.axis1.config.enable_watchdog = False
odrv0.axis1.config.startup_motor_calibration = False
odrv0.axis1.config.startup_encoder_offset_calibration = False
odrv0.axis1.config.startup_closed_loop_control = False

# Wyczyść błędy
odrv0.axis0.clear_errors()

# === MOTOR CONFIG ===
print("Konfiguracja motora...")
odrv0.axis0.motor.config.motor_type = 0  # HIGH_CURRENT
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.current_lim = 20  
odrv0.axis0.motor.config.current_lim_margin = 2
odrv0.axis0.motor.config.calibration_current = 5
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 60
# odrv0.axis0.motor.config.current_control_bandwidth = 100  
# odrv0.axis0.motor.config.current_control_bandwidth = 50 # Niski dla Hall sensors, better for 30rpm than 100
odrv0.axis0.motor.config.current_control_bandwidth = 20
odrv0.axis0.motor.config.torque_constant = 0.04  # Typical for hoverboard motor

# Wartości zmierzone podczas kalibracji (opcjonalne - odkomentuj jeśli chcesz pominąć kalibrację)
# odrv0.axis0.motor.config.phase_resistance = 0.192
# odrv0.axis0.motor.config.phase_inductance = 0.000316
# odrv0.axis0.motor.config.pre_calibrated = True

# === ENCODER CONFIG (HALL SENSORS) ===
print("Konfiguracja enkodera Hall...")
odrv0.axis0.encoder.config.mode = 1  # ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90  # 6 * 15 pole_pairs
odrv0.axis0.encoder.config.bandwidth = 100  # Niski dla wygładzenia
odrv0.axis0.encoder.config.enable_phase_interpolation = True

# Pomiń kalibrację enkodera (odkomentuj jeśli już skalibrowany)
# odrv0.axis0.encoder.config.pre_calibrated = True

# === CONTROLLER CONFIG ===
print("Konfiguracja kontrolera...")
odrv0.axis0.controller.config.control_mode = 2  # VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.controller.config.vel_gain = 0.50
odrv0.axis0.controller.config.vel_integrator_gain = 2
odrv0.axis0.controller.config.vel_ramp_rate = 30
odrv0.axis0.controller.config.pos_gain = 20.0  # For consistent behavior across boards

# === KALIBRACJA ===
print("\n=== KALIBRACJA ===")
print("UWAGA: Silnik się poruszy!")
input("Naciśnij Enter aby rozpocząć kalibrację motora...")

# Kalibracja motora
print("Kalibracja motora (8A, ~10s)...")
odrv0.axis0.requested_state = 4  # MOTOR_CALIBRATION
import time
time.sleep(10)

# Sprawdź błędy
from odrive.utils import dump_errors
dump_errors(odrv0)

print(f"Phase resistance: {odrv0.axis0.motor.config.phase_resistance}")
print(f"Phase inductance: {odrv0.axis0.motor.config.phase_inductance}")

if odrv0.axis0.motor.config.phase_resistance > 0.1:
    print("✓ Kalibracja motora OK")
    odrv0.axis0.motor.config.pre_calibrated = True
else:
    print("✗ Kalibracja motora FAILED - sprawdź połączenia faz!")
    exit()

# Kalibracja enkodera (opcjonalnie)
print("\n=== KALIBRACJA ENKODERA ===")
choice = input("Uruchomić pełną kalibrację enkodera? (ta pełna procedura obraca silnik) [n/t]: ").strip().lower()
if choice == 't' or choice == 'y':
    input("Naciśnij Enter aby rozpocząć kalibrację enkodera...")
    print("Kalibracja enkodera (silnik zrobi obrót tam i z powrotem)...")
    odrv0.axis0.encoder.config.pre_calibrated = False
    odrv0.axis0.requested_state = 7  # FULL_CALIBRATION_SEQUENCE
    time.sleep(20)

    dump_errors(odrv0)

    if odrv0.axis0.encoder.is_ready:
        print("✓ Kalibracja enkodera OK")
        odrv0.axis0.encoder.config.pre_calibrated = True
    else:
        print("✗ Kalibracja enkodera FAILED - sprawdź połączenia Hall sensors!")
        exit()
else:
    print("Pominięto pełną kalibrację enkodera.")
    print("Używam istniejącego `encoder.config.cpr` i oznaczam enkoder jako skalibrowany.")
    odrv0.axis0.encoder.config.pre_calibrated = True

# === ZAPISZ KONFIGURACJĘ ===
print("\n=== ZAPISYWANIE ===")
odrv0.save_configuration()
print("Konfiguracja zapisana! Reboot...")

try:
    odrv0.reboot()
except Exception as e:
    print(f"ODrive rozłączony podczas reboot (normalne): {e}")

print("\n=== GOTOWE ===")
print("Czekaj ~5 sekund na reboot ODrive...")
