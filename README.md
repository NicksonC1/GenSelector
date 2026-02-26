# GenSelector

GenSelector is an autonomous routine selector + match HUD for VEX V5 (PROS + LVGL), designed to be used alongside [GenClient](https://github.com/your-org/GenClient).

It gives you:
- A scrollable autonomous selector on the brain screen.
- A clean, competition-style UI with team branding.
- Live telemetry fields (`X`, `Y`, `Theta`).
- Three circular temperature gauges (chassis hot motor + 2 subsystem motors).
- Flexible input support (brain screen, controller button, ADI switch, custom callback).

---

## What GenSelector Does

GenSelector handles two things:

1. **Autonomous Selection**
- Displays your auton routine list.
- Shows 3 visible rows at a time.
- Scrolls automatically when moving past the 3rd visible option.
- Lets you select with touch (BrainScreen mode) or external input (button/switch/custom mode).

2. **Driver-Facing Match UI**
- Top status strip (battery/team/title).
- Left info panel (`X`, `Y`, `Theta`).
- Bottom-left gauge cluster for temperatures.
- Right panel for auton list and highlight.

---

## Requirements

- PROS 4.x project
- LVGL (bundled with PROS kernel)
- C++20 enabled in project toolchain
- GenClient integrated in your project (motors/chassis/odometry)

---

## Install

1. Copy the selector files into your project:
- `include/gen/selector.hpp`
- `src/gen/selector.cpp`

2. Add UI assets to `src/`:
- `background.c` (full 480x240 background)
- `logosmall.c` (small Gen icon near title)

3. Make sure the asset symbols match what selector uses:
- `extern const lv_image_dsc_t background;`
- `extern const lv_image_dsc_t logosmall;`

4. Rebuild project.

---

## Quick Start

### 1) Define autonomous routines

```cpp
using robot::AutonFunc;

robot::AutonRoutineList autonRoutines = {
    {"Left", static_cast<AutonFunc>(Auton::left)},
    {"Right", static_cast<AutonFunc>(Auton::right)},
    {"Solo AWP", static_cast<AutonFunc>(Auton::solo)},
    {"Skills", static_cast<AutonFunc>(Auton::skills)},
};
```

### 2) Define telemetry getters

`terminal.fields` requires function pointers (`double (*)()`), so use free functions or no-capture lambdas.

```cpp
double getX() { return chassis.getPose().x; }
double getY() { return chassis.getPose().y; }
double getTheta() { return chassis.getPose().theta; }

double hottestDrivetrainTemp() {
    const double leftHot = std::max(leftDrive[0].get_temperature(),
                                    std::max(leftDrive[1].get_temperature(), leftDrive[2].get_temperature()));
    const double rightHot = std::max(rightDrive[0].get_temperature(),
                                     std::max(rightDrive[1].get_temperature(), rightDrive[2].get_temperature()));
    return std::max(leftHot, rightHot);
}
```

### 3) Build selector config

```cpp
const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "78181A",
    },
    .devices = robot::SelectorDevicesConfig(
        hottestDrivetrainTemp,  // gauge 1: chassis hottest motor
        &intakeMotor,           // gauge 2
        &indexerMotor           // gauge 3
    ),
    .terminal = {
        .fields = {
            {"X", getX, 2},
            {"Y", getY, 2},
            {"Theta", getTheta, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};
```

### 4) Create selector instance

```cpp
robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);
```

### 5) Hook into PROS lifecycle

```cpp
void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    autonSelector.start();
}

void autonomous() {
    autonSelector.runSelectedOr(Auton::test);  // fallback if out-of-range/null
}
```

---

## Input Modes

- `BrainScreen`
  - Touch row to pick selected auton.
- `ControllerButton`
  - Uses `controller->get_digital_new_press(button)` to advance selection.
- `AdiDigitalIn`
  - Uses limit switch press to advance selection.
- `Custom`
  - Uses your callback `bool (*customNewPress)(void*)`.

---

## Temperature Gauge Behavior

- Gauge labels default to: `Chassis`, `Intake`, `Indexer`
- Gauge max scale defaults to `50` (50 = full arc).
- Values above max are clamped.

---

## UI Asset Notes (LVGL 9)

This project uses LVGL 9 naming.
- Use `lv_image_dsc_t` (not `lv_img_dsc_t`).
- Use LVGL 9 image converter output format.
- If you get undefined symbol errors, verify symbol names in your `.c` files exactly match:
  - `background`
  - `logosmall`

---

## Common Errors

### `unknown type name 'lv_img_dsc_t'`
Cause:
- Asset exported for LVGL 8.

Fix:
- Re-export image for LVGL 9 so it uses `lv_image_dsc_t`.

### `undefined reference to background/logosmall`
Cause:
- Symbol name mismatch between selector extern and generated asset file.

Fix:
- Rename symbol in asset `.c` or update extern declaration.

### `terminal.fields` won’t compile with `chassis.getPose().x`
Cause:
- Field expects function pointer, not direct double value.

Fix:
- Wrap in getter function or no-capture lambda.

---

## Layout Tuning

Most spacing is controlled by constants at the top of `selector.cpp`:
- Screen and top row: `kScreenWidth`, `kScreenHeight`, `kTopRowY`
- Info panel: `kInfoX`, `kInfoY`, `kInfoW`, `kInfoH`
- Gauges: `kGaugeBaseY`, `kGaugeSize`, `kGaugeGap`, `kGaugeStartX`
- Selector panel: `kSelectorX`, `kSelectorY`, `kSelectorW`, `kSelectorH`
- Visible options: `kVisibleRows`

Tip:
- Tune constants first, then fine-tune individual `lv_obj_set_pos(...)` calls.

---

## Recommended Project Structure

- `include/gen/selector.hpp`
- `src/gen/selector.cpp`
- `src/background.c`
- `src/logosmall.c`
- `src/main.cpp`

---

## Credits

Created for use with GenClient-based VEX projects by Team 78181A Genesis.
