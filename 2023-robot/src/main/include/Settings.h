#include <fmt/core.h>
#include <frc/DataLogManager.h>
#include <frc/Preferences.h>

class BooleanPreferenceRegistration {
 public:
  BooleanPreferenceRegistration(std::string_view name, bool defaultValue) {
    frc::Preferences::InitBoolean(name, defaultValue);
    frc::DataLogManager::Log(
      fmt::format("Registered '{}' boolean with default '{}'", name, defaultValue));
  }
};

#define REGISTER_BOOL(fn, name, defaultValue)                                            \
  static constexpr std::string_view kPreference##fn = name;                              \
  bool Get##fn() { return frc::Preferences::GetBoolean(kPreference##fn, defaultValue); } \
  void Set##fn(bool v) { frc::Preferences::SetBoolean(kPreference##fn, v); }             \
  static BooleanPreferenceRegistration preference##fn { kPreference##fn, defaultValue };

class IntPreferenceRegistration {
 public:
  IntPreferenceRegistration(std::string_view name, int defaultValue) {
    frc::Preferences::InitInt(name, defaultValue);
    frc::DataLogManager::Log(
      fmt::format("Registered '{}' int with default '{}'", name, defaultValue));
  }
};

#define REGISTER_INT(fn, name, defaultValue)                                        \
  static constexpr std::string_view kPreference##fn = name;                         \
  int Get##fn() { return frc::Preferences::GetInt(kPreference##fn, defaultValue); } \
  void Set##fn(int v) { frc::Preferences::SetInt(kPreference##fn, v); }             \
  static IntPreferenceRegistration preference##fn { kPreference##fn, defaultValue };

class DoublePreferenceRegistration {
 public:
  DoublePreferenceRegistration(std::string_view name, double defaultValue) {
    frc::Preferences::InitDouble(name, defaultValue);
    frc::DataLogManager::Log(
      fmt::format("Registered '{}' double with default '{}'", name, defaultValue));
  }
};

#define REGISTER_DOUBLE(fn, name, defaultValue)                                           \
  static constexpr std::string_view kPreference##fn = name;                               \
  double Get##fn() { return frc::Preferences::GetDouble(kPreference##fn, defaultValue); } \
  void Set##fn(double v) { frc::Preferences::SetDouble(kPreference##fn, v); }             \
  static DoublePreferenceRegistration preference##fn { kPreference##fn, defaultValue };

class StringPreferenceRegistration {
 public:
  StringPreferenceRegistration(std::string_view name, std::string_view defaultValue) {
    frc::Preferences::InitString(name, defaultValue);
    frc::DataLogManager::Log(
      fmt::format("Registered '{}' string with default '{}'", name, defaultValue));
  }
};

#define REGISTER_STRING(fn, name, defaultValue)                                         \
  static constexpr std::string_view kPreference##fn = name;                             \
  static constexpr std::string_view kPreference##fnDefault = defaultValue;              \
  std::string Get##fn() {                                                               \
    return frc::Preferences::GetString(kPreference##fn, kPreference##fnDefault);        \
  }                                                                                     \
  void Set##fn(std::string_view v) { frc::Preferences::SetString(kPreference##fn, v); } \
  static StringPreferenceRegistration preference##fn { kPreference##fn, kPreference##fnDefault };
