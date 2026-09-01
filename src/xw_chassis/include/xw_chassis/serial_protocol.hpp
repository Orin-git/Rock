#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xw_chassis {

constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
constexpr size_t kSendSize = 11;
constexpr size_t kRecvSize = 24;

constexpr uint8_t kAutoChargeHeader = 0x7C;
constexpr uint8_t kAutoChargeTail = 0x7F;
constexpr size_t kAutoChargeSize = 8;

constexpr uint8_t kBmsHeader = 0xFB;
constexpr uint8_t kBmsTail = 0xFD;
constexpr size_t kBmsSize = 30;
constexpr uint8_t kBmsType = 0x01;
constexpr uint8_t kBmsPayloadLen = 0x19;

uint8_t xor_bcc(const uint8_t * data, size_t len);

/** Build 11-byte speed command (m/s, rad/s → mm/s scaled int16 BE). */
std::vector<uint8_t> pack_speed(double vx, double vy, double wz, int mode = 0);

struct MotionFrame
{
  int flag_stop{0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct ChargeFrame
{
  double current{0.0};
  int red{0};
  bool charging{false};
  int charge_set_state{0};
};

std::optional<MotionFrame> parse_motion_frame(const uint8_t * buf, size_t len);
std::optional<ChargeFrame> parse_charge_frame(const uint8_t * buf, size_t len);
/** Validate BMS envelope; returns copy of 30-byte frame on success. */
std::optional<std::vector<uint8_t>> parse_bms_raw_frame(const uint8_t * buf, size_t len);

struct ParsedFrames
{
  std::vector<MotionFrame> motion;
  std::vector<ChargeFrame> charge;
  std::vector<std::vector<uint8_t>> bms;
};

/** Byte-stream reassembler for motion / charge / BMS frames. */
class FrameParser
{
public:
  void reset();
  ParsedFrames feed(const uint8_t * data, size_t len);

private:
  static int next_header(const std::vector<uint8_t> & buf);
  std::vector<uint8_t> buf_;
};

}  // namespace xw_chassis
