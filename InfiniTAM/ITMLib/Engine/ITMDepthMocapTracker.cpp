// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthMocapTracker.h"
#include "../../ORUtils/Cholesky.h"
#include "../Objects/ITMViewMocap.h"

#include <unsupported/Eigen/MatrixFunctions>
//#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

#include <math.h>
#include <iomanip>

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthMocapTracker::ITMDepthMocapTracker( Vector2i imgSize,
                                            TrackerIterationType *trackingRegime,
                                            int noHierarchyLevels,
                                            int noICPRunTillLevel,
                                            float distThresh,
                                            float terminationThreshold,
                                            const ITMLowLevelEngine *lowLevelEngine,
                                            MemoryDeviceType memoryType) :
    ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, noICPRunTillLevel, distThresh, terminationThreshold, lowLevelEngine, memoryType)
{
    int i = 0;
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthMocapTracker::PreTrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    // Compute mocap-based estimation of f_cam_cam0.
    ITMViewMocap* mocapView = (ITMViewMocap*)view;
    Eigen::Framef f_cam_tracker;
    PoseToFrame( f_cam_tracker, view->calib->m_h_cam_tracker.calib );
    static Eigen::Framef f_cam0_mocapBase = mocapView->m_f_tracker_mocapBase * f_cam_tracker;
    Eigen::Framef f_cam_cam0 = f_cam0_mocapBase.getInverse() * mocapView->m_f_tracker_mocapBase * f_cam_tracker;

    Eigen::Vector3f r_cam_cam0 = f_cam_cam0.m_quat.toRotationVector();
    static uint32_t i = 0;
    std::cout << std::setw(6) << i << " : "
              << std::setw(10) << r_cam_cam0.x()       << " " << std::setw(10) << r_cam_cam0.y()       << " " << std::setw(10) << r_cam_cam0.z()       << " --- "
              << std::setw(10) << f_cam_cam0.m_pos.x() << " " << std::setw(10) << f_cam_cam0.m_pos.y() << " " << std::setw(10) << f_cam_cam0.m_pos.z() << " "
              << std::endl;
    ++i;

//    static uint32_t i = 0;
//    std::cout << std::setw(6) << i << " : "
//              << std::setw(10) << f_cam_cam0.m_quat.w() << std::setw(10) << f_cam_cam0.m_quat.x() << std::setw(10) << f_cam_cam0.m_quat.y() << std::setw(10) << f_cam_cam0.m_quat.z()
//              << std::setw(10) << f_cam_cam0.m_pos.x()  << std::setw(10) << f_cam_cam0.m_pos.y()  << std::setw(10) << f_cam_cam0.m_pos.z()
//              << std::endl;
//    ++i;

    // Convert ICL-NUIM frame convention to InfiniTAM convention.
    // NOTE: I have no idea where this conversion stems from.
    Eigen::Vector3f mocap_rotVec = f_cam_cam0.m_quat.toRotationVector();
    Eigen::Vector3f mocap_pos    = f_cam_cam0.m_pos;

    Eigen::Vector3f mocap_rotVec_corrected(-mocap_rotVec.x(), mocap_rotVec.y(), -mocap_rotVec.z());
    Eigen::Vector3f mocap_pos_corrected(mocap_pos.x(), -mocap_pos.y(), mocap_pos.z());

    Eigen::Framef mocap_frame_corrected;
    mocap_frame_corrected.m_quat.fromRotationVector(mocap_rotVec_corrected);
    mocap_frame_corrected.m_pos = mocap_pos_corrected;

    // Initialize the tracker.
    Eigen::Framef mocap_inv_frame_corrected = mocap_frame_corrected.getInverse();
    ITMPose mocapInv_pose;
    FrameToPose(mocapInv_pose, mocap_inv_frame_corrected);

//    approxInvPose = trackingState->pose_d->GetInvM();
//    trackingState->pose_d->SetInvM(approxInvPose);
    trackingState->pose_d->SetFrom(&mocapInv_pose);

    trackingState->pose_d->Coerce(); // Orthonormalization.
    approxInvPose = trackingState->pose_d->GetInvM();
    approxPose = trackingState->pose_d->GetM();
}

} // namespace Engine.

} // namespace ITMLib
