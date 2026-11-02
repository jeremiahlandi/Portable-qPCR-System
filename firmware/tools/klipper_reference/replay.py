#!/usr/bin/env python3
"""Generate (or check) the expected outputs of Klipper's thermal algorithms for the replay tests.

Runs the unmodified Klipper classes in klippy_extras/ against synthesized traces and writes
CSV files the C++ tests (firmware/test/test_thermal) compare against.
"""
import argparse
import csv
import io
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = ROOT / "firmware" / "test" / "data"
TRACES = DATA / "traces"
EXPECTED = DATA / "expected"

sys.path.insert(0, str(HERE))
from klippy_extras import heaters, pid_calibrate, spi_temperature, thermistor, verify_heater  # noqa: E402

# Fixed parameters shared with the C++ tests (keep in sync with test_thermal/test_main.cpp).
PID_PARAMS = {"pid_Kp": 30.0, "pid_Ki": 1.5, "pid_Kd": 100.0}
MAX_POWER = 1.0
SMOOTH_TIME = 1.0
CHECK_PARAMS = {"hysteresis": 5.0, "max_error": 120.0, "heating_gain": 2.0, "check_gain_time": 20.0}
THERMISTOR_POINTS = (25.0, 100000.0, 150.0, 1641.9, 250.0, 226.15)  # EPCOS 100K B57560G104F
THERMISTOR_PULLUP = 4700.0
RTD_NOMINAL = 1000.0
RTD_REFERENCE = 4300.0
AUTOTUNE_TARGET = 95.0
DT = 0.25


# ---------------------------------------------------------------------------- shims
class FakeConfig:
    def __init__(self, values, name="heater block"):
        self.values = values
        self.name = name
        self.printer = FakePrinter()

    def getfloat(self, key, default=None, **kw):
        return float(self.values.get(key, default))

    def getint(self, key, default=None, **kw):
        return int(self.values.get(key, default))

    def getboolean(self, key, default=None, **kw):
        return bool(self.values.get(key, default))

    def get(self, key, default=None, **kw):
        return self.values.get(key, default)

    def get_name(self):
        return self.name

    def get_printer(self):
        return self.printer


class ShutdownRequested(Exception):
    pass


class FakeReactor:
    NEVER = float("inf")
    NOW = 0.0

    def register_timer(self, *a, **k):
        return object()

    def update_timer(self, *a, **k):
        pass


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()

    def register_event_handler(self, *a, **k):
        pass

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def invoke_shutdown(self, msg):
        raise ShutdownRequested(msg)


class FakeHeater:
    """What ControlPID / ControlAutoTune / HeaterCheck need from Klipper's Heater."""

    def __init__(self, max_power=MAX_POWER, smooth_time=SMOOTH_TIME):
        self.max_power = max_power
        self.smooth_time = smooth_time
        self.pwm = 0.0
        self.target = 0.0
        self.temp = 0.0

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def get_pwm_delay(self):
        return DT

    def set_pwm(self, read_time, value):
        self.pwm = value

    def alter_target(self, target):
        self.target = target

    def get_temp(self, eventtime):
        return self.temp, self.target


# ---------------------------------------------------------------------------- plant
class Plant:
    """First-order block with a lagged heater, used to synthesize traces. Mirrored in C++."""

    def __init__(self, gain=0.8, tau=120.0, heater_lag=5.0, ambient=25.0):
        self.gain = gain
        self.tau = tau
        self.heater_lag = heater_lag
        self.ambient = ambient
        self.temp = ambient
        self.power = 0.0

    def step(self, pwm, dt):
        self.power += (pwm - self.power) * (dt / self.heater_lag)
        self.temp += dt * (self.gain * self.power - (self.temp - self.ambient) / self.tau)
        return self.temp


def fmt(x):
    return "%.6f" % x


# ---------------------------------------------------------------------------- traces
def synth_trace(name, schedule, plant_gain=0.8, disturbance=None):
    """Closed loop with the original Klipper PID; returns rows of (time, temp, target)."""
    heater = FakeHeater()
    pid = heaters.ControlPID(heater, FakeConfig(PID_PARAMS))
    plant = Plant(gain=plant_gain)
    rows = []
    t = 0.0
    for target, duration in schedule:
        end = t + duration
        while t < end - 1e-9:
            temp = plant.temp
            if disturbance and abs(t - disturbance[0]) < 1e-9:
                plant.temp -= disturbance[1]
                temp = plant.temp
            rows.append((t, temp, target))
            pid.temperature_update(t, temp, target)
            plant.step(heater.pwm, DT)
            t = round(t + DT, 6)
    return rows


TRACE_SPECS = {
    "ramp_hold_steps": dict(schedule=[(95.0, 120.0), (60.0, 120.0), (72.0, 60.0), (0.0, 30.0)]),
    "disturbance_at_hold": dict(schedule=[(60.0, 200.0)], disturbance=(120.0, 8.0)),
    "runaway_no_heat": dict(schedule=[(95.0, 120.0)], plant_gain=0.0),
    "weak_heater": dict(schedule=[(95.0, 600.0)], plant_gain=0.35),
}


def write_csv(path, header, rows):
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(header)
    for r in rows:
        w.writerow([fmt(v) if isinstance(v, float) else v for v in r])
    return buf.getvalue()


def read_trace(name):
    with open(TRACES / f"{name}.csv", newline="") as f:
        r = csv.DictReader(f)
        return [(float(x["time"]), float(x["temp"]), float(x["target"])) for x in r]


# ---------------------------------------------------------------------------- expectations
def expected_pid(rows):
    heater = FakeHeater()
    pid = heaters.ControlPID(heater, FakeConfig(PID_PARAMS))
    out = []
    for t, temp, target in rows:
        pid.temperature_update(t, temp, target)
        out.append((t, heater.pwm))
    return out


def expected_check(rows):
    heater = FakeHeater()
    cfg = FakeConfig(CHECK_PARAMS, name="verify_heater block")
    check = verify_heater.HeaterCheck(cfg)
    check.heater = heater
    out = []
    faulted = False
    for t, temp, target in rows:
        if abs(t - round(t)) > 1e-9:
            continue  # the check runs once per second
        if not faulted:
            heater.temp, heater.target = temp, target
            try:
                check.check_event(t)
            except ShutdownRequested:
                faulted = True
        out.append((t, 1 if faulted else 0))
    return out


def expected_thermistor():
    th = thermistor.Thermistor(THERMISTOR_PULLUP, 0.0)
    th.setup_coefficients(*THERMISTOR_POINTS)
    adcs = [0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 0.8, 0.9, 0.95, 0.98]
    rows = [(a, th.calc_temp(a)) for a in adcs]
    temps = [0.0, 25.0, 40.0, 60.0, 72.0, 95.0, 105.0, 150.0]
    back = [(T, th.calc_adc(T)) for T in temps]
    return rows, back, (th.c1, th.c2, th.c3)


def expected_rtd():
    conv = spi_temperature.MAX31865.__new__(spi_temperature.MAX31865)
    adc_to_resist = RTD_REFERENCE / float(spi_temperature.MAX31865_ADC_MAX)
    conv.adc_to_resist_div_nominal = adc_to_resist / RTD_NOMINAL
    temps = [-10.0, 0.0, 25.0, 40.0, 60.0, 72.0, 95.0, 105.0, 150.0]
    rows = []
    for T in temps:
        adc = conv.calc_adc(T)
        rows.append((T, adc, conv.calc_temp(adc)))
    return rows


def expected_autotune():
    heater = FakeHeater()
    tune = pid_calibrate.ControlAutoTune(heater, AUTOTUNE_TARGET)
    heater.alter_target(AUTOTUNE_TARGET)
    plant = Plant()
    t = 0.0
    steps = 0
    while tune.check_busy(t, 0.0, 0.0) and steps < 40000:
        temp = plant.temp
        tune.temperature_update(t, temp, heater.target)
        plant.step(heater.pwm, DT)
        t = round(t + DT, 6)
        steps += 1
    kp, ki, kd = tune.calc_final_pid()
    return {"steps": steps, "peaks": len(tune.peaks), "kp": kp, "ki": ki, "kd": kd}


def build():
    files = {}
    for name, spec in TRACE_SPECS.items():
        rows = synth_trace(name, **spec)
        files[TRACES / f"{name}.csv"] = write_csv(TRACES / f"{name}.csv", ["time", "temp", "target"], rows)
        files[EXPECTED / f"{name}.pid.csv"] = write_csv(None, ["time", "pwm"], expected_pid(rows))
        files[EXPECTED / f"{name}.check.csv"] = write_csv(None, ["time", "fault"], expected_check(rows))
    adc_rows, back_rows, coeffs = expected_thermistor()
    files[EXPECTED / "thermistor_adc_to_temp.csv"] = write_csv(None, ["adc", "temp"], adc_rows)
    files[EXPECTED / "thermistor_temp_to_adc.csv"] = write_csv(None, ["temp", "adc"], back_rows)
    files[EXPECTED / "thermistor_coefficients.json"] = json.dumps(
        {"c1": coeffs[0], "c2": coeffs[1], "c3": coeffs[2]}, indent=1) + "\n"
    files[EXPECTED / "max31865.csv"] = write_csv(None, ["temp", "adc", "temp_back"], expected_rtd())
    files[EXPECTED / "autotune.json"] = json.dumps(expected_autotune(), indent=1) + "\n"
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="verify committed files instead of writing")
    args = ap.parse_args()
    files = build()
    if args.check:
        bad = []
        for path, content in files.items():
            if not path.exists() or path.read_text() != content:
                bad.append(str(path.relative_to(ROOT)))
        if bad:
            print("replay: committed data differs from the Klipper originals:", *bad, sep="\n  ")
            return 1
        print(f"replay: {len(files)} files match the Klipper originals")
        return 0
    for path, content in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    print(f"replay: wrote {len(files)} files under {DATA.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
