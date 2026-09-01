#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xw_chassis {

enum class ChargeState : int
{
  STANDBY = 0,
  CHARGING = 1,
  DISCHARGING = 2,
};

struct BatterySample
{
  double voltage{0.0};
  double current{0.0};
  double soc_percent{0.0};
  double soh_percent{0.0};
  ChargeState state{ChargeState::STANDBY};
  double mos_temperature{0.0};
  double env_temperature{0.0};
  uint32_t warning_bits{0};
  uint32_t protection_bits{0};
  int comm_status{0};
};

/** Parse validated 30-byte FB 01 19 ... BCC FD frame. byte_order: "big" or "little". */
std::optional<BatterySample> parse_battery_frame(
  const std::vector<uint8_t> & frame, const std::string & byte_order);

}  // namespace xw_chassis
