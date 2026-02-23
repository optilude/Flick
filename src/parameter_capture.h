// Parameter Capture - Soft Takeover for Edit Modes
// Copyright (C) 2026 Boyd Timothy <btimothy@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "daisy.h"
#include "daisy_hardware.h"
#include <cmath>

namespace flick {

/**
 * @brief Implements soft takeover for knob-based parameters in edit modes.
 *
 * When entering an edit mode, the current parameter value is frozen and the knob
 * position is recorded. The parameter remains frozen until the knob moves beyond
 * a threshold (default 5% of full range), preventing sudden jumps when knobs
 * control different parameters in different modes.
 *
 * Usage:
 * - Normal mode: Use p_knob.Process() directly
 * - Edit mode: Substitute with knob_capture.Process()
 * - On entering edit mode: Call Capture(current_parameter_value)
 * - On exiting edit mode: Call Reset()
 */
class KnobCapture {
public:
  /**
   * @brief Constructs a KnobCapture bound to a specific knob.
   *
   * @param knob Reference to the Parameter object representing the knob
   * @param threshold Movement threshold (0.0-1.0) required to activate, default 0.05
   */
  explicit KnobCapture(daisy::Parameter& knob, float threshold = 0.05f)
      : knob_(knob), frozen_value_(0.0f), baseline_control_(0.0f),
        is_active_(true), threshold_(threshold) {}

  /**
   * @brief Freezes the current parameter value and records knob position.
   *
   * Call this when entering an edit mode. The parameter value is frozen and
   * the current knob position is recorded as the baseline. Subsequent calls
   * to Process() will return the frozen value until the knob moves beyond
   * the threshold.
   *
   * @param current_param_value The current parameter value to freeze
   */
  void Capture(float current_param_value) {
    frozen_value_ = current_param_value;
    baseline_control_ = knob_.Process();
    is_active_ = false;
  }

  /**
   * @brief Returns the parameter value based on capture state.
   *
   * Pass the current calculated parameter value (e.g., p_knob.Process() * multiplier).
   * Returns either the frozen value or passes through the current value based on whether
   * the raw knob has moved beyond the threshold.
   *
   * The capture logic uses raw knob positions (0-1) for threshold detection but works
   * with parameter values for freezing and returning. This keeps scaling logic in the
   * calling code while the capture class only handles freeze/unfreeze behavior.
   *
   * @param current_param_value The current calculated parameter value
   * @return Parameter value (frozen or current)
   */
  float Process(float current_param_value) {
    float current_control = knob_.Process();

    if (is_active_) {
      // Pass-through mode (normal operation or already activated)
      return current_param_value;
    }

    // Capture mode - check for movement using raw control values
    if (std::fabs(current_control - baseline_control_) >= threshold_) {
      // Threshold exceeded, activate and pass through current value
      is_active_ = true;
      return current_param_value;
    }

    // Still frozen, return the frozen parameter value
    return frozen_value_;
  }

  /**
   * @brief Resets to pass-through mode.
   *
   * Call this when exiting an edit mode to restore normal operation.
   */
  void Reset() {
    is_active_ = true;
  }

  /**
   * @brief Checks if the knob has been activated.
   * @return true if in pass-through mode or activated by movement
   */
  bool IsActive() const { return is_active_; }

private:
  daisy::Parameter& knob_;    ///< Reference to the knob's Parameter object
  float frozen_value_;        ///< Frozen parameter value from Capture()
  float baseline_control_;    ///< Raw knob position when captured
  bool is_active_;            ///< true = pass-through, false = frozen
  float threshold_;           ///< Movement threshold for activation
};

/**
 * @brief Implements soft takeover for switch-based parameters in edit modes.
 *
 * Similar to KnobCapture but for discrete toggle switches. When captured, the
 * parameter value remains frozen until the switch moves to a different position.
 *
 * Usage:
 * - Declare with switch index and value lookup array
 * - Call Capture(current_param_value) when entering edit mode
 * - Call Process() to get parameter value (handles lookup internally)
 * - Call Reset() when exiting edit mode
 */
class SwitchCapture {
public:
  /**
   * @brief Constructs a SwitchCapture bound to a specific toggle switch.
   *
   * @param hw Reference to the Funbox hardware object
   * @param switch_idx The toggle switch identifier (TOGGLESWITCH_1/2/3)
   */
  SwitchCapture(Funbox& hw, Funbox::Toggleswitch switch_idx)
      : hw_(hw), switch_idx_(switch_idx),
        frozen_value_(0.0f), baseline_position_(0), is_active_(true) {}

  /**
   * @brief Freezes the current parameter value and records switch position.
   *
   * @param current_param_value The current parameter value to freeze
   */
  void Capture(float current_param_value) {
    frozen_value_ = current_param_value;
    baseline_position_ = hw_.GetToggleswitchPosition(switch_idx_);
    is_active_ = false;
  }

  /**
   * @brief Returns the parameter value based on capture state.
   *
   * Pass the current calculated parameter value (e.g., value_map[switch_position]).
   * Returns either the frozen value or passes through the current value based on whether
   * the switch has moved to a different position.
   *
   * The capture logic uses raw switch positions for change detection but works
   * with parameter values for freezing and returning. This keeps lookup logic in the
   * calling code while the capture class only handles freeze/unfreeze behavior.
   *
   * @param current_param_value The current calculated parameter value
   * @return Parameter value (frozen or current)
   */
  float Process(float current_param_value) {
    if (is_active_) {
      // Pass-through mode
      return current_param_value;
    }

    // Capture mode - check for movement using raw switch position
    int current_position = hw_.GetToggleswitchPosition(switch_idx_);
    if (current_position != baseline_position_) {
      // Switch moved, activate and pass through current value
      is_active_ = true;
      return current_param_value;
    }

    // Still frozen
    return frozen_value_;
  }

  /**
   * @brief Resets to pass-through mode.
   */
  void Reset() {
    is_active_ = true;
  }

  /**
   * @brief Checks if the switch has been activated.
   * @return true if in pass-through mode or activated by movement
   */
  bool IsActive() const { return is_active_; }

private:
  Funbox& hw_;                    ///< Reference to hardware object
  Funbox::Toggleswitch switch_idx_; ///< Which toggle switch
  float frozen_value_;            ///< Frozen parameter value
  int baseline_position_;         ///< Switch position when captured
  bool is_active_;                ///< true = pass-through, false = frozen
};

} // namespace flick
