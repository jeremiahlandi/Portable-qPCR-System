#include "peltier.h"

#include "arduino_hal.h"
#include "pins.h"

namespace qpcr::target {

namespace {

LedcPwmOutput heatPwm(qpcr::pins::kHBridgePwmHeat, kHBridgePwmHz, kHBridgePwmBits);
LedcPwmOutput coolPwm(qpcr::pins::kHBridgePwmCool, kHBridgePwmHz, kHBridgePwmBits);
GpioOutput bridgeEnable(qpcr::pins::kHBridgeEnable);
ArduinoClock uptime;

qpcr::thermal::PeltierControllerConfig controllerConfig() {
    qpcr::thermal::PeltierControllerConfig config;
    // Gains stay at zero until autotune on the real block (#7); maxPower 1.0 defers to the
    // drive's duty cap (kPeltierDefaultMaxDuty) as the effective ceiling.
    config.pid.maxPower = 1.0;
    config.pid.smoothTime = 1.0;
    config.ramp.maxHeatingRate = kBlockMaxHeatingRate;
    config.ramp.maxCoolingRate = kBlockMaxCoolingRate;
    return config;
}

} // namespace

qpcr::thermal::PeltierDrive& peltierDrive() {
    // Constructed on first use, after peltierInit() has set up the pins, so the drive's safe
    // state write reaches real outputs rather than inert wrappers.
    static qpcr::thermal::PeltierDrive drive(heatPwm, coolPwm, bridgeEnable, uptime);
    return drive;
}

qpcr::thermal::PeltierController& peltierController() {
    static qpcr::thermal::PeltierController controller(peltierDrive(), controllerConfig());
    return controller;
}

bool peltierInit() {
    // Enable low first: with R_EN / L_EN low the BTS7960 outputs float whatever RPWM / LPWM do
    // while the LEDC channels come up.
    bridgeEnable.begin(false);
    const bool heatOk = heatPwm.attach();
    const bool coolOk = coolPwm.attach();
    peltierDrive();
    peltierController();
    return heatOk && coolOk;
}

} // namespace qpcr::target
