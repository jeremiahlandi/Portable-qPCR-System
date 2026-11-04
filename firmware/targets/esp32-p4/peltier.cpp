#include "peltier.h"

#include "arduino_hal.h"
#include "pins.h"

namespace qpcr::target {

namespace {

LedcPwmOutput heatPwm(qpcr::pins::kHBridgePwmHeat, kHBridgePwmHz, kHBridgePwmBits);
LedcPwmOutput coolPwm(qpcr::pins::kHBridgePwmCool, kHBridgePwmHz, kHBridgePwmBits);
GpioOutput bridgeEnable(qpcr::pins::kHBridgeEnable);
ArduinoClock uptime;

} // namespace

qpcr::thermal::PeltierDrive& peltierDrive() {
    // Constructed on first use, after peltierInit() has set up the pins, so the drive's safe
    // state write reaches real outputs rather than inert wrappers.
    static qpcr::thermal::PeltierDrive drive(heatPwm, coolPwm, bridgeEnable, uptime);
    return drive;
}

bool peltierInit() {
    // Enable low first: with R_EN / L_EN low the BTS7960 outputs float whatever RPWM / LPWM do
    // while the LEDC channels come up.
    bridgeEnable.begin(false);
    const bool heatOk = heatPwm.attach();
    const bool coolOk = coolPwm.attach();
    peltierDrive();
    return heatOk && coolOk;
}

} // namespace qpcr::target
