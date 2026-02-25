#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "liblvgl/lvgl.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace robot {

using AutonFunc = void (*)();
using AutonRoutineList = std::vector<std::pair<std::string, AutonFunc>>;

enum class SelectorInputType {
    BrainScreen,
    ControllerButton,
    AdiDigitalIn,
    Custom
};

struct SelectorInputConfig {
    SelectorInputType type = SelectorInputType::BrainScreen;
    pros::Controller* controller = nullptr;
    pros::controller_digital_e_t controllerButton = pros::E_CONTROLLER_DIGITAL_X;
    pros::adi::DigitalIn* adiDigitalIn = nullptr;
    void* customDevice = nullptr;
    bool (*customNewPress)(void* device) = nullptr;
};

struct SelectorMenuConfig {
    std::string teamNumber = "0000A";
};

struct SelectorDevicesConfig {
    std::vector<pros::Motor*> motors{};
};

using TerminalValueGetter = double (*)();

struct TerminalField {
    std::string label;
    TerminalValueGetter value = nullptr;
    std::uint8_t precision = 2;
};

struct SelectorTerminalConfig {
    std::vector<TerminalField> fields{};
    std::uint32_t refreshMs = 60;
};

struct SelectorConfig {
    SelectorInputConfig input{};
    SelectorMenuConfig menu{};
    SelectorDevicesConfig devices{};
    SelectorTerminalConfig terminal{};
    std::uint32_t lcdLine = 4;
    std::uint32_t pollDelayMs = 20;
};

class AutonSelector {
  public:
    AutonSelector(SelectorConfig config, AutonRoutineList routines);

    void start();
    void tick();

    std::size_t selectedIndex() const;
    const AutonRoutineList& routines() const;
    void runSelectedOr(AutonFunc fallback) const;

  private:
    struct UiState {
        lv_obj_t* backgroundImage = nullptr;
        lv_obj_t* logoSmallImage = nullptr;

        lv_obj_t* headerPanel = nullptr;
        lv_obj_t* teamIconLabel = nullptr;
        lv_obj_t* teamLabel = nullptr;
        lv_obj_t* titleLabel = nullptr;
        lv_obj_t* subtitleLabel = nullptr;
        lv_obj_t* logoSlotPrimary = nullptr;
        lv_obj_t* logoSlotSecondary = nullptr;

        lv_obj_t* infoPanel = nullptr;
        lv_obj_t* infoHeader = nullptr;
        std::array<lv_obj_t*, 3> infoLines{nullptr, nullptr, nullptr};

        std::array<lv_obj_t*, 3> gaugeArcs{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> gaugeValues{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> gaugeNames{nullptr, nullptr, nullptr};

        lv_obj_t* selectorPanel = nullptr;
        lv_obj_t* selectorHighlight = nullptr;
        lv_obj_t* selectorArrow = nullptr;
        std::array<lv_obj_t*, 3> selectorRows{nullptr, nullptr, nullptr};
        std::array<lv_obj_t*, 3> selectorTexts{nullptr, nullptr, nullptr};
    };

    static void selectorRowEventThunk(lv_event_t* event);

    void onSelectorRowTapped(lv_obj_t* rowObj);

    void buildUi();
    void updateUi();
    void updateInfoLines();
    void updateGauges();
    void updateSelectorList();

    bool newPress();
    double readGaugeSource(std::size_t gaugeIndex) const;
    std::size_t visibleRowFromSelected() const;
    void advanceSelectionDown();

    SelectorConfig config_;
    AutonRoutineList routines_;
    std::size_t selectedIndex_ = 0;
    std::size_t listWindowStart_ = 0;
    std::unique_ptr<pros::Task> task_{};
    UiState ui_{};
    std::uint32_t uiElapsedMs_ = 0;
    bool suppressUiEvent_ = false;
};

}  // namespace robot
