#pragma once

#include "gen/api.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace gen::Motion {

struct PIDGains {
    float kP = 0.0f;
    float kI = 0.0f;
    float kD = 0.0f;
};

struct ExitBand {
    float range = 0.0f;
    int timeoutMs = 0;
};

struct ControllerProfile {
    PIDGains gains{};
    float antiWindupRange = 0.0f;
    ExitBand smallError{1.0f, 100};
    ExitBand largeError{3.0f, 500};
    float slew = 0.0f;

    gen::ControllerSettings toGen() const;
};

struct DrivetrainProfile {
    float trackWidthIn = 0.0f;
    float wheelDiameterIn = gen::Omniwheel::NEW_325;
    float wheelRpm = 0.0f;
    float horizontalDrift = 2.0f;

    gen::Drivetrain toGen(pros::MotorGroup* left, pros::MotorGroup* right) const;
};

struct OdomProfile {
    gen::TrackingWheel* vertical1 = nullptr;
    gen::TrackingWheel* vertical2 = nullptr;
    gen::TrackingWheel* horizontal1 = nullptr;
    gen::TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;

    gen::OdomSensors toGen() const;
};

}  // namespace gen::Motion
