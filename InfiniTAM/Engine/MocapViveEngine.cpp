#include "MocapViveEngine.h"

#ifndef COMPILE_WITHOUT_OPENVR

#include <array>
#include <cmath>
#include <stdexcept>

#include <openvr.h>

namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
MocapViveEngine::MocapViveEngine(std::vector<std::string> const& devicesToTrack):
    m_devicesToTrack(devicesToTrack),
    m_vrSystem (nullptr),
    m_trackedDeviceIdx (vr::k_unTrackedDeviceIndexInvalid)
{
    vr::EVRInitError eError = vr::VRInitError_None;

    m_vrSystem = vr::VR_Init(&eError, vr::VRApplication_Background);

    if (eError != vr::VRInitError_None)
    {
        m_vrSystem = nullptr;
        throw std::runtime_error(std::string("Impossible to load VR runtime: ")
                                 + vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    }
}

////////////////////////////////////////////////////////////////////////////////
MocapViveEngine::~MocapViveEngine()
{
    if (m_vrSystem)
        vr::VR_Shutdown();
}

////////////////////////////////////////////////////////////////////////////////
bool MocapViveEngine::hasMoreMeasurements()
{
    // TODO: Check the MocapSourceEngine, if we can return false at one moment.
    // Probably remake the interface, i.e. change the name of this function
    return true;
}

////////////////////////////////////////////////////////////////////////////////
MocapViveEngine::MeasurementStatus MocapViveEngine::getMeasurement(Eigen::Framef& mocapFrame)
{
    std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> trackedDevicePoses;

    // Note: TrackingUniverseStanding may be the same as TrackingUniverseRawAndUncalibrated in our case.
    m_vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,
                                                0,
                                                trackedDevicePoses.data(), trackedDevicePoses.size());

    if (m_trackedDeviceIdx == vr::k_unTrackedDeviceIndexInvalid
            || !m_vrSystem->IsTrackedDeviceConnected(m_trackedDeviceIdx))
    {
        // We do not yet have found a device, or it is lost.

        for (vr::TrackedDeviceIndex_t deviceIdx = 0; deviceIdx < trackedDevicePoses.size(); ++deviceIdx)
        {
            // Check if connected
            if (!m_vrSystem->IsTrackedDeviceConnected(deviceIdx))
                continue;

            // Try to retrieve the model name
            std::array<char,1024> buf;
            vr::TrackedPropertyError propertyError;
            m_vrSystem->GetStringTrackedDeviceProperty(deviceIdx,
                                                       vr::Prop_RenderModelName_String,
                                                       buf.data(), buf.size(),
                                                       &propertyError);

            if (propertyError != vr::TrackedProp_Success)
                continue;
            std::string deviceName (buf.data());

            // Check if we want to track this device
            if (std::find(m_devicesToTrack.begin(), m_devicesToTrack.end(), deviceName) == m_devicesToTrack.end())
                continue;

            // This device is valid, we track it.
            m_trackedDeviceIdx = deviceIdx;
            std::clog << "Start tracking device " << m_trackedDeviceIdx << ' ' << deviceName << std::endl;

            return getMeasurement(trackedDevicePoses[deviceIdx], mocapFrame);
        }

        std::clog << "No device found" << std::endl;
        return MEASUREMENT_ERROR;
    }

    return getMeasurement(trackedDevicePoses[m_trackedDeviceIdx], mocapFrame);
}

////////////////////////////////////////////////////////////////////////////////
MocapViveEngine::MeasurementStatus MocapViveEngine::getMeasurement(vr::TrackedDevicePose_t const& devicePos,
                                                                   Eigen::Framef& mocapFrame)
{
    if (!devicePos.bPoseIsValid)
    {
        std::clog << "Mocap: poseInvalid" << std::endl;
        return MEASUREMENT_ERROR;
    }

    switch (devicePos.eTrackingResult)
    {
    case vr::ETrackingResult::TrackingResult_Running_OK:
        break;

    case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
        std::clog << "MEASUREMENT_CALIBRATING" << std::endl;
        return MEASUREMENT_CALIBRATING;

    case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
    case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
        std::clog << "MEASUREMENT_OUT_OF_RANGE" << std::endl;
        return MEASUREMENT_OUT_OF_RANGE;

    default:
    case vr::ETrackingResult::TrackingResult_Uninitialized:
    case vr::ETrackingResult::TrackingResult_Fallback_RotationOnly:
        std::clog << "TrackingResult_Uninitialized or rotation only" << std::endl;
        return MEASUREMENT_ERROR;
    }

    hmdToFrame(devicePos.mDeviceToAbsoluteTracking, mocapFrame);

    return MEASUREMENT_OK;
}

////////////////////////////////////////////////////////////////////////////////
// From https://github.com/ValveSoftware/openvr/blob/master/samples/unity_keyboard_sample/Assets/SteamVR/Scripts/SteamVR_Utils.cs
void MocapViveEngine::hmdToFrame(vr::HmdMatrix34_t const& matrix, Eigen::Framef& mocapFrame)
{
    mocapFrame.setWXYZ(         std::sqrt(std::max(0.0f, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2.0f,
                       copysign(std::sqrt(std::max(0.0f, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2.0f, matrix.m[2][1] - matrix.m[1][2]),
                       copysign(std::sqrt(std::max(0.0f, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2.0f, matrix.m[0][2] - matrix.m[2][0]),
                       copysign(std::sqrt(std::max(0.0f, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2.0f, matrix.m[1][0] - matrix.m[0][1]),
                       matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]);
}

////////////////////////////////////////////////////////////////////////////////
float MocapViveEngine::copysign(float number, float sign)
{
    float numberMagnitude = std::abs(number);
    return sign > 0.0f ? numberMagnitude : -1.0f * numberMagnitude;
}

} // namespace Engine.

} // namespace InfiniTAM.

#else

namespace InfiniTAM
{

namespace Engine
{

MocapViveEngine::MocapViveEngine()
{
  throw std::runtime_error("Compiled without OpenVR support");
}

MocapViveEngine::~MocapViveEngine()
{
}

bool MocapViveEngine::hasMoreMeasurements()
{
    return false;
}

MocapViveEngine::MeasurementStatus MocapViveEngine::getMeasurement(Eigen::Framef& mocapFrame)
{
    return MEASUREMENT_ERROR;
}


} // namespace Engine.

} // namespace InfiniTAM.

#endif
