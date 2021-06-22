#pragma once

#include <ros/types.h>

namespace smb_lowlevel_controller {

typedef int16_t SmbModeType;
enum SmbMode : SmbModeType {
  FREEZE = 0,          // Freeze
  THAW,                // UnFreeze
  MODE_VELOCITY,       // Track wheel velocity
  MODE_TORQUE,         // Track wheel torque
  MODE_DC_CMD,         // Command motor duty cycle open loop
  MODE_TWIST           // Track the base twist
};

} // namespace smb_lowlevel_controller

