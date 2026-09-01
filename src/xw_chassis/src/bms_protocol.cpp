#include "xw_chassis/bms_protocol.hpp"

#include "xw_chassis/serial_protocol.hpp"

#include <cstring>

namespace xw_chassis {
namespace {

int64_t decode_signed(
  const uint8_t * frame, size_t start, size_t size, bool big_endian)
{
  uint64_t v = 0;
  if (big_endian) {
    for (size_t i = 0; i < size; ++i) {
      v = (v << 8) | frame[start + i];
    }
  } else {
    for (size_t i = 0; i < size; ++i) {
      v |= static_cast<uint64_t>(frame[start + i]) << (8 * i);
    }
  }
  const size_t bits = size * 8;
  const uint64_t sign_bit = 1ULL << (bits - 1);
  if (v & sign_bit) {
    const uint64_t mask = (1ULL << bits) - 1;
    return static_cast<int64_t>(v | ~mask);
  }
  return static_cast<int64_t>(v);
}

uint64_t decode_unsigned(
  const uint8_t * frame, size_t start, size_t size, bool big_endian)
{
  uint64_t v = 0;
  if (big_endian) {
    for (size_t i = 0; i < size; ++i) {
      v = (v << 8) | frame[start + i];
    }
  } else {
    for (size_t i = 0; i < size; ++i) {
      v |= static_cast<uint64_t>(frame[start + i]) << (8 * i);
    }
  }
  return v;
}

}  // namespace

std::optional<BatterySample> parse_battery_frame(
  const std::vector<uint8_t> & frame, const std::string & byte_order)
{
  if (byte_order != "little" && byte_order != "big") {
    return std::nullopt;
  }
  if (frame.size() != kBmsSize) {
    return std::nullopt;
  }
  if (
    frame[0] != kBmsHeader || frame[1] != kBmsType || frame[2] != kBmsPayloadLen ||
    frame[29] != kBmsTail)
  {
    return std::nullopt;
  }
  if (xor_bcc(frame.data(), 28) != frame[28]) {
    return std::nullopt;
  }

  const bool be = (byte_order == "big");
  BatterySample s;
  s.voltage = static_cast<double>(decode_unsigned(frame.data(), 3, 4, be)) * 0.001;
  s.current = static_cast<double>(decode_signed(frame.data(), 7, 4, be)) * 0.001;
  s.soc_percent = static_cast<double>(decode_unsigned(frame.data(), 11, 2, be)) * 0.1;
  s.soh_percent = static_cast<double>(frame[13]);
  if (frame[14] > 2) {
    return std::nullopt;
  }
  s.state = static_cast<ChargeState>(frame[14]);
  s.mos_temperature = static_cast<double>(decode_signed(frame.data(), 15, 2, be)) * 0.1;
  s.env_temperature = static_cast<double>(decode_signed(frame.data(), 17, 2, be)) * 0.1;
  s.warning_bits = static_cast<uint32_t>(
    (decode_unsigned(frame.data(), 19, 2, be) << 16) |
    decode_unsigned(frame.data(), 21, 2, be));
  s.protection_bits = static_cast<uint32_t>(
    (decode_unsigned(frame.data(), 23, 2, be) << 16) |
    decode_unsigned(frame.data(), 25, 2, be));
  s.comm_status = static_cast<int>(frame[27]);

  if (s.voltage < 5.0 || s.voltage > 100.0) {
    return std::nullopt;
  }
  if (s.current < -500.0 || s.current > 500.0) {
    return std::nullopt;
  }
  if (s.soc_percent < 0.0 || s.soc_percent > 100.0) {
    return std::nullopt;
  }
  if (s.soh_percent < 0.0 || s.soh_percent > 100.0) {
    return std::nullopt;
  }
  if (s.mos_temperature < -60.0 || s.mos_temperature > 150.0) {
    return std::nullopt;
  }
  if (s.env_temperature < -60.0 || s.env_temperature > 150.0) {
    return std::nullopt;
  }
  return s;
}

}  // namespace xw_chassis
