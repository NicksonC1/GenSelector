#include "main.h"
#include <algorithm>
#include <cstdint>
#include "gen/electronics.h"
#include "gen/setup.hpp"
#include "gen/selector.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.hpp"

using Button = gen::Controller::Button;
using robot::AutonFunc;

// ========================= Example Project =========================
// This file is a full example for testing the LVGL autonomous selector.
// Top-left: Team number
// Top-right: Autonomous roller
// Bottom-left: Team image from selector config
// Bottom-right: Custom terminal fields (X, Y, Theta)

gen::Controller controller(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, true);
gen::MotorGroup leftDrive({1, -2, 3}, 600.0, 1.0);
gen::MotorGroup rightDrive({-4, 5, -6}, 600.0, 1.0);
gen::CustomIMU imu(9, 1.01123595506);

pros::Rotation verticalEnc(-12);
pros::Rotation horizontalEnc(13);
gen::TrackingWheel verticalWheel(&verticalEnc, gen::Omniwheel::NEW_275, -5.0f);
gen::TrackingWheel horizontalWheel(&horizontalEnc, gen::Omniwheel::NEW_275, -2.0f);

gen::Piston wingPiston('A', false, "wing");
gen::PistonGroup wings({{"wing", &wingPiston}});

constexpr gen::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 11.75f,
    .wheelDiameterIn = gen::Omniwheel::NEW_325,
    .wheelRpm = 600.0f,
    .horizontalDrift = 10.0f,
};

constexpr gen::Motion::ControllerProfile lateralProfile{
    .gains = {7.5f, 0.0f, 6.0f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr gen::Motion::ControllerProfile angularProfile{
    .gains = {2.75f, 0.0f, 17.5f},
    .antiWindupRange = 0.0f,
    .smallError = {1.0f, 100},
    .largeError = {3.0f, 500},
    .slew = 0.0f,
};

constexpr gen::Motion::OdomProfile odomProfile{
    .vertical1 = &verticalWheel,
    .vertical2 = nullptr,
    .horizontal1 = &horizontalWheel,
    .horizontal2 = nullptr,
    .imu = &imu,
};

gen::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
gen::ControllerSettings lateralController = lateralProfile.toGen();
gen::ControllerSettings angularController = angularProfile.toGen();
gen::OdomSensors odomSensor = odomProfile.toGen();
gen::Chassis chassis(drivetrain, lateralController, angularController, odomSensor);

double terminalX() { return 0.0; }
double terminalY() { return 0.0; }
double terminalTheta() { return 0.0; }
double hottestDrivetrainTemp() {
    const double leftHot = std::max(leftDrive[0].get_temperature(), std::max(leftDrive[1].get_temperature(), leftDrive[2].get_temperature()));
    const double rightHot = std::max(rightDrive[0].get_temperature(), std::max(rightDrive[1].get_temperature(), rightDrive[2].get_temperature()));
    return std::max(leftHot, rightHot);
}

pros::adi::DigitalIn autonLimitSwitch('H');

const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "78181A",
    },
    .devices = robot::SelectorDevicesConfig(hottestDrivetrainTemp, &leftDrive[0], &rightDrive[0]),
    .terminal = {
        .fields = {
            {"X", []() -> double { return chassis.getPose().x; }, 2},
            {"Y", []() -> double { return chassis.getPose().y; }, 2},
            {"Theta", []() -> double { return chassis.getPose().theta; }, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};

namespace Auton {

void test() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Auton: test"); }
void left() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Auton: left"); }
void right() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Auton: right"); }
void solo() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Auton: solo"); }
void skills() { pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Auton: skills"); }

}  // namespace Auton

robot::AutonRoutineList autonRoutines = {
    // {"Default Auton", static_cast<AutonFunc>(Auton::test)},
    {"Left", static_cast<AutonFunc>(Auton::left)},
    {"Right", static_cast<AutonFunc>(Auton::right)},
    {"Solo AWP", static_cast<AutonFunc>(Auton::solo)},
    {"Skills", static_cast<AutonFunc>(Auton::skills)},
};

robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);
// ======================= End Example Project =======================

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    autonSelector.start();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    autonSelector.runSelectedOr(Auton::test);
}

void opcontrol() {
    while (true) {
        controller.arcade_two_stick();

        if (controller.pressed(Button::R2)) {
            wings.toggle();
        }
        pros::delay(10);
    }
}
