#ifndef MICROMOUSE_INTERFACES_LOCALIZATION_PORT_H
#define MICROMOUSE_INTERFACES_LOCALIZATION_PORT_H

#include <array>
#include <cstdint>

namespace micromouse::interfaces
{

inline constexpr std::size_t localization_sensor_count = 5;

enum class ModuleStatus : std::uint8_t
{
    Ok,
    NotReady,
    InvalidInput,
};

struct LocalizationObservation
{
    std::array<float, localization_sensor_count> range_meters{};
    bool has_yaw = false;
    float yaw_radians = 0.0f;
    std::uint32_t sequence = 0;
};

struct LocalizationEstimate
{
    int row = -1;
    int col = -1;
    float probability = 0.0f;
};

class LocalizationPort
{
public:
    virtual ~LocalizationPort() = default;

    virtual ModuleStatus submit_observation(const LocalizationObservation &observation) = 0;
    virtual bool try_get_estimate(LocalizationEstimate &estimate) = 0;
};

}  // namespace micromouse::interfaces

#endif  // MICROMOUSE_INTERFACES_LOCALIZATION_PORT_H
