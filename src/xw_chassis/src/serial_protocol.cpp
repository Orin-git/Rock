#include "xw_chassis/serial_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace xw_chassis {

uint8_t xor_bcc(const uint8_t * data, size_t len)
{
  uint8_t check = 0;
  for (size_t i = 0; i < len; ++i) {
    check ^= data[i];
  }
  return check;
}

static int16_t clamp_i16(int v)
{
  if (v > 32767) {
    return 32767;
  }
  if (v < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(v);
}

std::vector<uint8_t> pack_speed(double vx, double vy, double wz, int mode)
{
  std::vector<uint8_t> tx(kSendSize, 0);
  tx[0] = kFrameHeader;
  tx[1] = static_cast<uint8_t>(mode & 0xFF);
  tx[2] = 0;
  const double vals[3] = {vx, vy, wz};
  for (int i = 0; i < 3; ++i) {
    const int16_t scaled = clamp_i16(static_cast<int>(std::lround(vals[i] * 1000.0)));
    const size_t base = static_cast<size_t>(3 + i * 2);
    tx[base] = static_cast<uint8_t>((scaled >> 8) & 0xFF);
    tx[base + 1] = static_cast<uint8_t>(scaled & 0xFF);
  }
  tx[9] = xor_bcc(tx.data(), 9);
  tx[10] = kFrameTail;
  return tx;
}

static double odom_trans(uint8_t hi, uint8_t lo)
{
  const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  return static_cast<double>(raw) / 1000.0;
}

std::optional<MotionFrame> parse_motion_frame(const uint8_t * buf, size_t len)
{
  if (buf == nullptr || len != kRecvSize) {
    return std::nullopt;
  }
  if (buf[0] != kFrameHeader || buf[23] != kFrameTail) {
    return std::nullopt;
  }
  if (xor_bcc(buf, 22) != buf[22]) {
    return std::nullopt;
  }
  MotionFrame f;
  f.flag_stop = static_cast<int>(buf[1]);
  f.vx = odom_trans(buf[2], buf[3]);
  f.vy = odom_trans(buf[4], buf[5]);
  // MCU Z sign opposite ROS CCW+
  f.wz = -odom_trans(buf[6], buf[7]);
  return f;
}

std::optional<ChargeFrame> parse_charge_frame(const uint8_t * buf, size_t len)
{
  if (buf == nullptr || len != kAutoChargeSize) {
    return std::nullopt;
  }
  if (buf[0] != kAutoChargeHeader || buf[7] != kAutoChargeTail) {
    return std::nullopt;
  }
  if (xor_bcc(buf, 6) != buf[6]) {
    return std::nullopt;
  }
  ChargeFrame f;
  f.current = ((static_cast<uint16_t>(buf[1]) << 8) | buf[2]) / 1000.0;
  f.red = static_cast<int>(buf[3]);
  f.charging = buf[4] != 0;
  f.charge_set_state = static_cast<int>(buf[5]);
  return f;
}

std::optional<std::vector<uint8_t>> parse_bms_raw_frame(const uint8_t * buf, size_t len)
{
  if (buf == nullptr || len != kBmsSize) {
    return std::nullopt;
  }
  if (
    buf[0] != kBmsHeader || buf[1] != kBmsType || buf[2] != kBmsPayloadLen ||
    buf[29] != kBmsTail)
  {
    return std::nullopt;
  }
  if (xor_bcc(buf, 28) != buf[28]) {
    return std::nullopt;
  }
  return std::vector<uint8_t>(buf, buf + kBmsSize);
}

void FrameParser::reset()
{
  buf_.clear();
}

int FrameParser::next_header(const std::vector<uint8_t> & buf)
{
  int best = -1;
  for (uint8_t h : {kFrameHeader, kAutoChargeHeader, kBmsHeader}) {
    auto it = std::find(buf.begin(), buf.end(), h);
    if (it != buf.end()) {
      const int pos = static_cast<int>(std::distance(buf.begin(), it));
      if (best < 0 || pos < best) {
        best = pos;
      }
    }
  }
  return best;
}

ParsedFrames FrameParser::feed(const uint8_t * data, size_t len)
{
  ParsedFrames out;
  if (data != nullptr && len > 0) {
    buf_.insert(buf_.end(), data, data + len);
  }
  while (true) {
    if (buf_.empty()) {
      break;
    }
    const int start = next_header(buf_);
    if (start < 0) {
      buf_.clear();
      break;
    }
    if (start > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + start);
    }
    const uint8_t kind = buf_[0];
    if (kind == kAutoChargeHeader) {
      if (buf_.size() < kAutoChargeSize) {
        break;
      }
      auto frame = parse_charge_frame(buf_.data(), kAutoChargeSize);
      if (frame.has_value()) {
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(kAutoChargeSize));
        out.charge.push_back(*frame);
        continue;
      }
      buf_.erase(buf_.begin());
      continue;
    }
    if (kind == kBmsHeader) {
      if (buf_.size() < kBmsSize) {
        break;
      }
      auto frame_b = parse_bms_raw_frame(buf_.data(), kBmsSize);
      if (frame_b.has_value()) {
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(kBmsSize));
        out.bms.push_back(*frame_b);
        continue;
      }
      buf_.erase(buf_.begin());
      continue;
    }
    if (buf_.size() < kRecvSize) {
      break;
    }
    auto frame_m = parse_motion_frame(buf_.data(), kRecvSize);
    if (frame_m.has_value()) {
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(kRecvSize));
      out.motion.push_back(*frame_m);
      continue;
    }
    buf_.erase(buf_.begin());
  }
  return out;
}

}  // namespace xw_chassis
