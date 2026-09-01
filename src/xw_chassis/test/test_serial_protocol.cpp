#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "xw_chassis/bms_protocol.hpp"
#include "xw_chassis/serial_protocol.hpp"

using xw_chassis::xor_bcc;

static std::vector<uint8_t> make_charge(
  double current_a = 0.42, int red = 2, int charging = 1, int state = 1)
{
  const int raw = static_cast<int>(std::lround(current_a * 1000.0));
  std::vector<uint8_t> buf(xw_chassis::kAutoChargeSize, 0);
  buf[0] = 0x7C;
  buf[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  buf[2] = static_cast<uint8_t>(raw & 0xFF);
  buf[3] = static_cast<uint8_t>(red & 0xFF);
  buf[4] = static_cast<uint8_t>(charging & 0xFF);
  buf[5] = static_cast<uint8_t>(state & 0xFF);
  buf[6] = xor_bcc(buf.data(), 6);
  buf[7] = 0x7F;
  return buf;
}

static std::vector<uint8_t> make_bms(
  const std::string & byte_order = "big",
  int voltage_mv = 25000,
  int current_ma = -1200,
  int soc_permille = 500)
{
  std::vector<uint8_t> frame = {0xFB, 0x01, 0x19};
  auto append_int = [&](int64_t v, size_t size, bool signed_v) {
    std::vector<uint8_t> bytes(size, 0);
    uint64_t u = static_cast<uint64_t>(v);
    if (signed_v && v < 0) {
      const uint64_t mask = (size >= 8) ? ~0ULL : ((1ULL << (size * 8)) - 1);
      u = static_cast<uint64_t>(v) & mask;
    }
    if (byte_order == "big") {
      for (size_t i = 0; i < size; ++i) {
        bytes[size - 1 - i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
      }
    } else {
      for (size_t i = 0; i < size; ++i) {
        bytes[i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
      }
    }
    frame.insert(frame.end(), bytes.begin(), bytes.end());
  };
  append_int(voltage_mv, 4, false);
  append_int(current_ma, 4, true);
  append_int(soc_permille, 2, false);
  frame.push_back(98);  // SOH
  frame.push_back(2);   // discharging
  append_int(315, 2, true);
  append_int(260, 2, true);
  append_int(0, 2, false);
  append_int(0, 2, false);
  append_int(0, 2, false);
  append_int(0, 2, false);
  frame.push_back(1);  // comm ok
  frame.push_back(xor_bcc(frame.data(), frame.size()));
  frame.push_back(0xFD);
  EXPECT_EQ(frame.size(), xw_chassis::kBmsSize);
  return frame;
}

TEST(SerialProtocol, PackSpeedMode)
{
  auto speed = xw_chassis::pack_speed(0.1, 0.0, 0.0, 1);
  ASSERT_EQ(speed.size(), 11u);
  EXPECT_EQ(speed[0], 0x7B);
  EXPECT_EQ(speed[1], 1);
  EXPECT_EQ(speed[10], 0x7D);
  EXPECT_EQ(speed[9], xor_bcc(speed.data(), 9));
}

TEST(SerialProtocol, ParseCharge)
{
  auto raw = make_charge();
  auto f = xw_chassis::parse_charge_frame(raw.data(), raw.size());
  ASSERT_TRUE(f.has_value());
  EXPECT_NEAR(f->current, 0.42, 1e-3);
  EXPECT_EQ(f->red, 2);
  EXPECT_TRUE(f->charging);
  EXPECT_EQ(f->charge_set_state, 1);
}

TEST(SerialProtocol, ParseBmsRawAndBattery)
{
  auto raw = make_bms();
  auto env = xw_chassis::parse_bms_raw_frame(raw.data(), raw.size());
  ASSERT_TRUE(env.has_value());
  auto sample = xw_chassis::parse_battery_frame(*env, "big");
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->voltage, 25.0, 1e-3);
  EXPECT_NEAR(sample->current, -1.2, 1e-3);
  EXPECT_NEAR(sample->soc_percent, 50.0, 1e-3);
  EXPECT_EQ(sample->state, xw_chassis::ChargeState::DISCHARGING);
}

TEST(SerialProtocol, ParserInterleaved)
{
  xw_chassis::FrameParser parser;
  auto speed = xw_chassis::pack_speed(0.1, 0.0, 0.0, 1);
  EXPECT_EQ(speed[1], 1);

  std::vector<uint8_t> rx(24, 0);
  rx[0] = 0x7B;
  rx[1] = 1;
  rx[23] = 0x7D;
  rx[22] = xor_bcc(rx.data(), 22);

  auto bms = make_bms();
  auto charge = make_charge();
  std::vector<uint8_t> blob;
  blob.insert(blob.end(), rx.begin(), rx.end());
  blob.insert(blob.end(), charge.begin(), charge.end());
  blob.insert(blob.end(), bms.begin(), bms.end());
  blob.insert(blob.end(), rx.begin(), rx.end());

  auto parsed = parser.feed(blob.data(), blob.size());
  EXPECT_EQ(parsed.motion.size(), 2u);
  EXPECT_EQ(parsed.charge.size(), 1u);
  EXPECT_EQ(parsed.bms.size(), 1u);
  EXPECT_TRUE(parsed.charge[0].charging);
  EXPECT_EQ(parsed.bms[0], bms);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
