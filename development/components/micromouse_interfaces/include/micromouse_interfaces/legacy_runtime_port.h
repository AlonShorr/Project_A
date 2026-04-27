#ifndef MICROMOUSE_INTERFACES_LEGACY_RUNTIME_PORT_H
#define MICROMOUSE_INTERFACES_LEGACY_RUNTIME_PORT_H

#include "micromouse_interfaces/localization_port.h"

namespace micromouse::interfaces
{

struct LegacyTelemetrySnapshot
{
    LocalizationObservation observation{};
    bool boot_button_pressed = false;
};

class LegacyRuntimePort
{
public:
    virtual ~LegacyRuntimePort() = default;

    virtual bool poll_telemetry(LegacyTelemetrySnapshot &snapshot) = 0;
    virtual void publish_estimate(const LocalizationEstimate &estimate) = 0;
};

}  // namespace micromouse::interfaces

#endif  // MICROMOUSE_INTERFACES_LEGACY_RUNTIME_PORT_H
