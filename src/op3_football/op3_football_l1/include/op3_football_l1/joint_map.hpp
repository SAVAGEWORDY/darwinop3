#ifndef OP3_FOOTBALL_L1_JOINT_MAP_HPP_
#define OP3_FOOTBALL_L1_JOINT_MAP_HPP_

#include <cmath>
#include <map>
#include <string>
#include <utility>

namespace op3_football_l1
{

// XM430-W350 conversion constants (same as ROBOTIS device file)
constexpr int32_t kValueOf0Radian = 2048;
constexpr int32_t kValueOfMinRadian = 0;
constexpr int32_t kValueOfMaxRadian = 4095;
constexpr double kMinRadian = -3.14159265;
constexpr double kMaxRadian = 3.14159265;

inline const std::map<int, std::string> & idToName()
{
  static const std::map<int, std::string> table = {
    {1, "r_sho_pitch"}, {2, "l_sho_pitch"},
    {3, "r_sho_roll"}, {4, "l_sho_roll"},
    {5, "r_el"}, {6, "l_el"},
    {7, "r_hip_yaw"}, {8, "l_hip_yaw"},
    {9, "r_hip_roll"}, {10, "l_hip_roll"},
    {11, "r_hip_pitch"}, {12, "l_hip_pitch"},
    {13, "r_knee"}, {14, "l_knee"},
    {15, "r_ank_pitch"}, {16, "l_ank_pitch"},
    {17, "r_ank_roll"}, {18, "l_ank_roll"},
    {19, "head_pan"}, {20, "head_tilt"},
  };
  return table;
}

inline const std::map<std::string, int> & nameToId()
{
  static std::map<std::string, int> table;
  if (table.empty()) {
    for (const auto & kv : idToName()) {
      table[kv.second] = kv.first;
    }
  }
  return table;
}

inline bool jointName(int id, std::string & name)
{
  const auto & table = idToName();
  auto it = table.find(id);
  if (it == table.end()) {
    return false;
  }
  name = it->second;
  return true;
}

inline double tickToRadian(int32_t value)
{
  if (value > kValueOf0Radian) {
    return static_cast<double>(value - kValueOf0Radian) * kMaxRadian /
           static_cast<double>(kValueOfMaxRadian - kValueOf0Radian);
  }
  if (value < kValueOf0Radian) {
    return static_cast<double>(value - kValueOf0Radian) * kMinRadian /
           static_cast<double>(kValueOfMinRadian - kValueOf0Radian);
  }
  return 0.0;
}

inline int32_t radianToTick(double radian)
{
  if (radian > 0.0) {
    return static_cast<int32_t>(
      radian * (kValueOfMaxRadian - kValueOf0Radian) / kMaxRadian + kValueOf0Radian);
  }
  if (radian < 0.0) {
    return static_cast<int32_t>(
      radian * (kValueOfMinRadian - kValueOf0Radian) / kMinRadian + kValueOf0Radian);
  }
  return kValueOf0Radian;
}

inline int32_t clampTick(int32_t value)
{
  if (value < 0) {
    return 0;
  }
  if (value > 4095) {
    return 4095;
  }
  return value;
}

}  // namespace op3_football_l1

#endif  // OP3_FOOTBALL_L1_JOINT_MAP_HPP_
