#pragma once

#ifndef COMPILE_WITHOUT_OPENVR
# include <openvr.h>
#endif

#include "MocapSourceEngine.h"

namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
/// \brief Motion capture source for the vive engine.
///        Searches for tracked devices and follow the first one it achieve to get the measurement.
class MocapViveEngine : public MocapSourceEngine
{
public:
    /// \param devicesToTrack Device model name accepted by the engine
    MocapViveEngine(std::vector<std::string> const& devicesToTrack = { "{htc}vr_tracker_vive_1_0" });
    virtual ~MocapViveEngine();

    virtual bool hasMoreMeasurements();
    virtual MeasurementStatus getMeasurement(Eigen::Framef& mocapFrame);

private:

#ifndef COMPILE_WITHOUT_OPENVR
    static float copysign(float number, float sign);
    static void hmdToFrame(vr::HmdMatrix34_t const& matrix, Eigen::Framef& mocapFrame);

    MeasurementStatus getMeasurement(vr::TrackedDevicePose_t const& devicePos, Eigen::Framef& mocapFrame);

    const std::vector<std::string> m_devicesToTrack;

    /// \brief VR system adapter
    vr::IVRSystem* m_vrSystem;

    /// \brief The tracked device index.
    vr::TrackedDeviceIndex_t m_trackedDeviceIdx;
#endif
};

} // namespace Engine.

} // namespace InfiniTAM.

