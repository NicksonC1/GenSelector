#include "gen/selector.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

extern const lv_image_dsc_t background;
extern const lv_image_dsc_t logosmall;

namespace robot {
namespace {

constexpr std::int32_t kScreenWidth = 480;
constexpr std::int32_t kScreenHeight = 240;
constexpr std::int32_t kSafeTop = 0;
constexpr std::int32_t kTopRowY = kSafeTop + 8;

constexpr std::int32_t kInfoX = 12;
constexpr std::int32_t kInfoY = kSafeTop + 46;
constexpr std::int32_t kInfoW = 190;
constexpr std::int32_t kInfoH = 105;

constexpr std::int32_t kGaugeBaseY = kInfoY + kInfoH + 6;
constexpr std::int32_t kGaugeSize = 54;
constexpr std::int32_t kGaugeGap = 12;
constexpr std::int32_t kGaugeStartX = 14;
constexpr std::int32_t kGaugeLabelY = kGaugeBaseY + kGaugeSize + 2;

constexpr std::int32_t kSelectorX = 220;
constexpr std::int32_t kSelectorY = 70;
constexpr std::int32_t kSelectorW = 245;
constexpr std::int32_t kSelectorH = 155;
constexpr std::int32_t kSelectorInnerX = 8;
constexpr std::int32_t kSelectorInnerY = 7;
constexpr std::int32_t kSelectorRowH = 52;
constexpr std::size_t kVisibleRows = 3;
constexpr double kGaugeMaxValue = 50.0;

constexpr std::array<const char*, 3> kGaugeNames = {"Chassis", "Intake", "Indexer"};
double clampDouble(double value, double low, double high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void stylePanel(lv_obj_t* panel, std::int32_t width, std::int32_t height, lv_color_t border) {
    lv_obj_set_size(panel, width, height);
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, border, LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x021a3a), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(panel, LV_OPA_10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 0, LV_PART_MAIN);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
}

std::string fallbackLabel(const std::string& label) {
    return label.empty() ? "?" : label;
}

}  // namespace

AutonSelector::AutonSelector(SelectorConfig config, AutonRoutineList routines)
    : config_(std::move(config)),
      routines_(std::move(routines)) {}

void AutonSelector::start() {
    if (task_ != nullptr) {
        return;
    }

    buildUi();
    updateUi();

    task_ = std::make_unique<pros::Task>([this]() {
        const std::uint32_t delayMs = std::max<std::uint32_t>(1, config_.pollDelayMs);
        while (true) {
            tick();
            pros::delay(delayMs);
        }
    }, "Auton Selector");
}

void AutonSelector::tick() {
    if (!routines_.empty() && newPress()) {
        advanceSelectionDown();
    }

    uiElapsedMs_ += std::max<std::uint32_t>(1, config_.pollDelayMs);
    if (uiElapsedMs_ >= std::max<std::uint32_t>(1, config_.terminal.refreshMs)) {
        uiElapsedMs_ = 0;
        updateUi();
    }
}

std::size_t AutonSelector::selectedIndex() const {
    return selectedIndex_;
}

const AutonRoutineList& AutonSelector::routines() const {
    return routines_;
}

void AutonSelector::runSelectedOr(AutonFunc fallback) const {
    if (selectedIndex_ < routines_.size() && routines_[selectedIndex_].second != nullptr) {
        routines_[selectedIndex_].second();
        return;
    }

    if (fallback != nullptr) {
        fallback();
    }
}

void AutonSelector::selectorRowEventThunk(lv_event_t* event) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(event));
    if (self == nullptr) {
        return;
    }
    self->onSelectorRowTapped(static_cast<lv_obj_t*>(lv_event_get_target(event)));
}

void AutonSelector::onSelectorRowTapped(lv_obj_t* rowObj) {
    if (suppressUiEvent_ || config_.input.type != SelectorInputType::BrainScreen || routines_.empty()) {
        return;
    }

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        if (ui_.selectorRows[i] != rowObj) {
            continue;
        }
        const std::size_t candidate = listWindowStart_ + i;
        if (candidate < routines_.size()) {
            selectedIndex_ = candidate;
            updateSelectorList();
        }
        return;
    }
}

void AutonSelector::buildUi() {
    lv_obj_t* screen = lv_screen_active();
    lv_obj_clean(screen);
    lv_obj_set_size(screen, kScreenWidth, kScreenHeight);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    ui_.backgroundImage = lv_image_create(screen);
    lv_image_set_src(ui_.backgroundImage, &background);
    lv_obj_set_pos(ui_.backgroundImage, 0, 0);
    lv_obj_set_size(ui_.backgroundImage, kScreenWidth, kScreenHeight);
    // lv_image_set_inner_align(ui_.backgroundImage, LV_IMAGE_ALIGN_STRETCH);

    // Top row (no boxed panel; drawn directly on background)
    lv_obj_t* batteryLabel = lv_label_create(screen);
    lv_label_set_text(batteryLabel, LV_SYMBOL_BATTERY_3 " 95%");
    lv_obj_set_style_text_color(batteryLabel, lv_color_hex(0x49b6ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(batteryLabel, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(batteryLabel, 14, kTopRowY);

    lv_obj_t* dividerLabel = lv_label_create(screen);
    lv_label_set_text(dividerLabel, "|");
    lv_obj_set_style_text_color(dividerLabel, lv_color_hex(0x4e8dcc), LV_PART_MAIN);
    lv_obj_set_style_text_font(dividerLabel, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(dividerLabel, 90, kTopRowY - 3);

    ui_.teamIconLabel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.teamIconLabel, 106, kTopRowY + 4);
    lv_obj_set_size(ui_.teamIconLabel, 16, 16);
    lv_obj_set_style_radius(ui_.teamIconLabel, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_.teamIconLabel, lv_color_hex(0x7ec6ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_.teamIconLabel, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_.teamIconLabel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(ui_.teamIconLabel, lv_color_hex(0x9ddbff), LV_PART_MAIN);
    lv_obj_set_style_pad_all(ui_.teamIconLabel, 0, LV_PART_MAIN);
    lv_obj_remove_flag(ui_.teamIconLabel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* teamIconHead = lv_obj_create(ui_.teamIconLabel);
    lv_obj_set_size(teamIconHead, 5, 5);
    lv_obj_set_pos(teamIconHead, 4, 1);
    lv_obj_set_style_radius(teamIconHead, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(teamIconHead, lv_color_hex(0xd8f1ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(teamIconHead, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(teamIconHead, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(teamIconHead, 0, LV_PART_MAIN);
    lv_obj_remove_flag(teamIconHead, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* teamIconBody = lv_obj_create(ui_.teamIconLabel);
    lv_obj_set_size(teamIconBody, 10, 5);
    lv_obj_set_pos(teamIconBody, 2, 8);
    lv_obj_set_style_radius(teamIconBody, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(teamIconBody, lv_color_hex(0xd8f1ff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(teamIconBody, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(teamIconBody, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(teamIconBody, 0, LV_PART_MAIN);
    lv_obj_remove_flag(teamIconBody, LV_OBJ_FLAG_SCROLLABLE);

    ui_.teamLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(ui_.teamLabel, lv_color_hex(0x8cc9ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.teamLabel, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(ui_.teamLabel, 130, kTopRowY + 1);

    ui_.logoSmallImage = lv_image_create(screen);
    lv_image_set_src(ui_.logoSmallImage, &logosmall);
    lv_obj_set_pos(ui_.logoSmallImage, 211, kTopRowY - 4);
    lv_obj_set_size(ui_.logoSmallImage, 45, 45);
    lv_image_set_inner_align(ui_.logoSmallImage, LV_IMAGE_ALIGN_STRETCH);

    lv_obj_t* titleDivider = lv_label_create(screen);
    lv_label_set_text(titleDivider, "|");
    lv_obj_set_style_text_color(titleDivider, lv_color_hex(0x6cb8f8), LV_PART_MAIN);
    lv_obj_set_style_text_font(titleDivider, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_pos(titleDivider, 258, kTopRowY - 1);

    ui_.titleLabel = lv_label_create(screen);
    lv_label_set_text(ui_.titleLabel, "Gen-Selector");
    lv_obj_set_style_text_color(ui_.titleLabel, lv_color_hex(0x6ec0ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.titleLabel, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_pos(ui_.titleLabel, 272, kTopRowY);

    ui_.subtitleLabel = lv_label_create(screen);
    lv_label_set_text(ui_.subtitleLabel, "Created by 78181A Genesis");
    lv_obj_set_style_text_color(ui_.subtitleLabel, lv_color_hex(0x8ab9e2), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.subtitleLabel, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_pos(ui_.subtitleLabel, 305, kTopRowY + 35);

    ui_.infoPanel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.infoPanel, kInfoX, kInfoY);
    stylePanel(ui_.infoPanel, kInfoW, kInfoH, lv_color_hex(0x2f79c4));

    ui_.infoHeader = lv_label_create(ui_.infoPanel);
    lv_label_set_text(ui_.infoHeader, "Info");
    lv_obj_set_style_text_color(ui_.infoHeader, lv_color_hex(0x6ec0ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.infoHeader, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_pos(ui_.infoHeader, 10, 4);

    lv_obj_t* infoDivider = lv_obj_create(ui_.infoPanel);
    lv_obj_set_pos(infoDivider, 0, 26);
    lv_obj_set_size(infoDivider, kInfoW - 2, 1);
    lv_obj_set_style_border_width(infoDivider, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(infoDivider, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(infoDivider, lv_color_hex(0x2a6ca8), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(infoDivider, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_pad_all(infoDivider, 0, LV_PART_MAIN);
    lv_obj_remove_flag(infoDivider, LV_OBJ_FLAG_SCROLLABLE);

    for (std::size_t i = 0; i < ui_.infoLines.size(); ++i) {
        ui_.infoLines[i] = lv_label_create(ui_.infoPanel);
        lv_obj_set_style_text_color(ui_.infoLines[i], lv_color_hex(0xb2d8ff), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.infoLines[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_set_pos(ui_.infoLines[i], 10, 31 + static_cast<std::int32_t>(i * 22));
    }

    for (std::size_t i = 0; i < 3; ++i) {
        const std::int32_t x = kGaugeStartX + static_cast<std::int32_t>(i) * (kGaugeSize + kGaugeGap);

        ui_.gaugeArcs[i] = lv_arc_create(screen);
        lv_obj_set_pos(ui_.gaugeArcs[i], x, 163);
        lv_obj_set_size(ui_.gaugeArcs[i], kGaugeSize, kGaugeSize);
        lv_arc_set_rotation(ui_.gaugeArcs[i], 135);
        lv_arc_set_bg_angles(ui_.gaugeArcs[i], 0, 270);
        lv_arc_set_value(ui_.gaugeArcs[i], 0);
        lv_obj_set_style_arc_width(ui_.gaugeArcs[i], 7, LV_PART_MAIN);
        lv_obj_set_style_arc_width(ui_.gaugeArcs[i], 7, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(ui_.gaugeArcs[i], lv_color_hex(0x2f6ca8), LV_PART_MAIN);
        lv_obj_set_style_arc_color(ui_.gaugeArcs[i], lv_color_hex(0x4ab1ff), LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(ui_.gaugeArcs[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_remove_style(ui_.gaugeArcs[i], nullptr, LV_PART_KNOB);
        lv_obj_remove_flag(ui_.gaugeArcs[i], LV_OBJ_FLAG_CLICKABLE);

        ui_.gaugeValues[i] = lv_label_create(screen);
        lv_obj_set_style_text_color(ui_.gaugeValues[i], lv_color_hex(0x4ab1ff), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.gaugeValues[i], &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_width(ui_.gaugeValues[i], kGaugeSize);
        lv_obj_set_style_text_align(ui_.gaugeValues[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(ui_.gaugeValues[i], x, 180);

        ui_.gaugeNames[i] = lv_label_create(screen);
        lv_label_set_text(ui_.gaugeNames[i], kGaugeNames[i]);
        lv_obj_set_style_text_color(ui_.gaugeNames[i], lv_color_hex(0x87bee8), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.gaugeNames[i], &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_set_width(ui_.gaugeNames[i], kGaugeSize + 16);
        lv_obj_set_style_text_align(ui_.gaugeNames[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_pos(ui_.gaugeNames[i], x - 8, 216);
    }

    ui_.selectorPanel = lv_obj_create(screen);
    lv_obj_set_pos(ui_.selectorPanel, kSelectorX, kSelectorY);
    stylePanel(ui_.selectorPanel, kSelectorW, kSelectorH, lv_color_hex(0x1d65a5));

    ui_.selectorHighlight = lv_obj_create(ui_.selectorPanel);
    lv_obj_set_size(ui_.selectorHighlight, kSelectorW - 2 * kSelectorInnerX, kSelectorRowH - 2);
    lv_obj_set_style_radius(ui_.selectorHighlight, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_.selectorHighlight, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(ui_.selectorHighlight, lv_color_hex(0xaadfff), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_.selectorHighlight, lv_color_hex(0xa6ddff), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_.selectorHighlight, LV_OPA_20, LV_PART_MAIN);
    lv_obj_remove_flag(ui_.selectorHighlight, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(ui_.selectorHighlight, LV_OBJ_FLAG_CLICKABLE);

    ui_.selectorArrow = lv_label_create(ui_.selectorHighlight);
    lv_label_set_text(ui_.selectorArrow, "<");
    lv_obj_set_style_text_color(ui_.selectorArrow, lv_color_hex(0x6ec0ff), LV_PART_MAIN);
    lv_obj_set_style_text_font(ui_.selectorArrow, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_obj_set_pos(ui_.selectorArrow, kSelectorW - 70, -10);

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        ui_.selectorRows[i] = lv_obj_create(ui_.selectorPanel);
        lv_obj_set_pos(ui_.selectorRows[i], kSelectorInnerX, kSelectorInnerY + static_cast<std::int32_t>(i) * kSelectorRowH);
        lv_obj_set_size(ui_.selectorRows[i], kSelectorW - 2 * kSelectorInnerX, kSelectorRowH - 2);
        lv_obj_set_style_bg_opa(ui_.selectorRows[i], LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_width(ui_.selectorRows[i], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(ui_.selectorRows[i], 0, LV_PART_MAIN);
        lv_obj_remove_flag(ui_.selectorRows[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(ui_.selectorRows[i], AutonSelector::selectorRowEventThunk, LV_EVENT_CLICKED, this);

        ui_.selectorTexts[i] = lv_label_create(ui_.selectorRows[i]);
        lv_obj_set_style_text_color(ui_.selectorTexts[i], lv_color_hex(0xb4d8ff), LV_PART_MAIN);
        lv_obj_set_style_text_font(ui_.selectorTexts[i], &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_pos(ui_.selectorTexts[i], 10, 8);
    }
}

void AutonSelector::updateUi() {
    if (ui_.teamLabel != nullptr) {
        lv_label_set_text_fmt(ui_.teamLabel, "%s", config_.menu.teamNumber.c_str());
    }
    updateInfoLines();
    updateGauges();
    updateSelectorList();
}

void AutonSelector::updateInfoLines() {
    const std::size_t fieldCount = std::min<std::size_t>(3, config_.terminal.fields.size());

    for (std::size_t i = 0; i < 3; ++i) {
        if (ui_.infoLines[i] == nullptr) {
            continue;
        }

        if (i >= fieldCount) {
            static const std::array<const char*, 3> defaults = {"x: 0", "y: 0", "theta: 0"};
            lv_label_set_text(ui_.infoLines[i], defaults[i]);
            continue;
        }

        const auto& field = config_.terminal.fields[i];
        const std::string label = fallbackLabel(field.label);
        const double value = field.value == nullptr ? 0.0 : field.value();
        lv_label_set_text_fmt(ui_.infoLines[i], "%s: %.*f", label.c_str(), static_cast<int>(field.precision), value);
    }
}

void AutonSelector::updateGauges() {
    for (std::size_t i = 0; i < 3; ++i) {
        const double value = readGaugeSource(i);
        const double clamped = clampDouble(value, 0.0, kGaugeMaxValue);
        const int percent = static_cast<int>(std::lround((clamped / kGaugeMaxValue) * 100.0));
        const int shown = static_cast<int>(std::lround(clamped));

        if (ui_.gaugeArcs[i] != nullptr) {
            lv_arc_set_value(ui_.gaugeArcs[i], percent);
        }
        if (ui_.gaugeValues[i] != nullptr) {
            lv_label_set_text_fmt(ui_.gaugeValues[i], "%d", shown);
        }
    }
}

void AutonSelector::updateSelectorList() {
    if (routines_.empty()) {
        for (auto* label : ui_.selectorTexts) {
            if (label != nullptr) {
                lv_label_set_text(label, "-");
            }
        }
        return;
    }

    const std::size_t maxWindowStart = routines_.size() > kVisibleRows ? routines_.size() - kVisibleRows : 0;
    listWindowStart_ = std::min(listWindowStart_, maxWindowStart);

    const std::size_t selectedRow = visibleRowFromSelected();
    lv_obj_set_pos(ui_.selectorHighlight, kSelectorInnerX, kSelectorInnerY + static_cast<std::int32_t>(selectedRow) * kSelectorRowH);

    for (std::size_t i = 0; i < kVisibleRows; ++i) {
        const std::size_t optionIndex = listWindowStart_ + i;
        if (optionIndex >= routines_.size()) {
            lv_label_set_text(ui_.selectorTexts[i], "");
            continue;
        }

        lv_label_set_text(ui_.selectorTexts[i], routines_[optionIndex].first.c_str());
        if (optionIndex == selectedIndex_) {
            lv_obj_set_style_text_color(ui_.selectorTexts[i], lv_color_hex(0xeaf7ff), LV_PART_MAIN);
        } else {
            lv_obj_set_style_text_color(ui_.selectorTexts[i], lv_color_hex(0xc2ddf7), LV_PART_MAIN);
        }
    }
}

bool AutonSelector::newPress() {
    switch (config_.input.type) {
        case SelectorInputType::BrainScreen:
            return false;

        case SelectorInputType::ControllerButton:
            if (config_.input.controller == nullptr) {
                return false;
            }
            return config_.input.controller->get_digital_new_press(config_.input.controllerButton);

        case SelectorInputType::AdiDigitalIn:
            if (config_.input.adiDigitalIn == nullptr) {
                return false;
            }
            return config_.input.adiDigitalIn->get_new_press();

        case SelectorInputType::Custom:
            if (config_.input.customNewPress == nullptr) {
                return false;
            }
            return config_.input.customNewPress(config_.input.customDevice);
    }

    return false;
}

double AutonSelector::readGaugeSource(std::size_t gaugeIndex) const {
    if (gaugeIndex < config_.devices.motors.size() && config_.devices.motors[gaugeIndex] != nullptr) {
        // 50 is treated as full-scale for these UI bars.
        return config_.devices.motors[gaugeIndex]->get_temperature();
    }

    if (gaugeIndex < config_.terminal.fields.size() && config_.terminal.fields[gaugeIndex].value != nullptr) {
        return config_.terminal.fields[gaugeIndex].value();
    }

    return 0.0;
}

std::size_t AutonSelector::visibleRowFromSelected() const {
    if (selectedIndex_ < listWindowStart_) {
        return 0;
    }
    const std::size_t row = selectedIndex_ - listWindowStart_;
    return std::min<std::size_t>(row, kVisibleRows - 1);
}

void AutonSelector::advanceSelectionDown() {
    if (routines_.empty()) {
        return;
    }

    selectedIndex_ = (selectedIndex_ + 1) % routines_.size();

    // Scroll after moving past the 3 visible rows (i.e., when entering the 4th option).
    if (routines_.size() > kVisibleRows && selectedIndex_ >= listWindowStart_ + kVisibleRows) {
        listWindowStart_ = std::min<std::size_t>(selectedIndex_ - (kVisibleRows - 1), routines_.size() - kVisibleRows);
    }

    if (selectedIndex_ < listWindowStart_) {
        listWindowStart_ = selectedIndex_;
    }

    updateSelectorList();
}

}  // namespace robot
