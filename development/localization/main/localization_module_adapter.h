#ifndef LOCALIZATION_MODULE_ADAPTER_H
#define LOCALIZATION_MODULE_ADAPTER_H

#include <micromouse_interfaces/localization_port.h>

#include "localization.h"

namespace micromouse
{

class LocalizationModuleAdapter : public interfaces::LocalizationPort
{
public:
    explicit LocalizationModuleAdapter(Localization &localization) noexcept : m_localization(localization) {}

    interfaces::ModuleStatus submit_observation(const interfaces::LocalizationObservation &observation) override
    {
        if (observation.has_yaw)
        {
            m_localization.update(observation.range_meters, observation.yaw_radians);
        }
        else
        {
            m_localization.update(observation.range_meters);
        }
        return interfaces::ModuleStatus::Ok;
    }

    bool try_get_estimate(interfaces::LocalizationEstimate &estimate) override
    {
        m_localization.get_best_position(estimate.row, estimate.col, estimate.probability);
        return estimate.row >= 0 && estimate.col >= 0;
    }

private:
    Localization &m_localization;
};

}  // namespace micromouse

#endif  // LOCALIZATION_MODULE_ADAPTER_H
