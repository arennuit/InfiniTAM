// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthMocapTracker_CPU.h"

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthMocapTracker_CPU::ITMDepthMocapTracker_CPU( Vector2i imgSize,
                                                    TrackerIterationType *trackingRegime,
                                                    int noHierarchyLevels,
                                                    int noICPRunTillLevel,
                                                    float distThresh,
                                                    float terminationThreshold,
                                                    const ITMLowLevelEngine *lowLevelEngine) :
    ITMDepthTracker_CPU( imgSize,
                         trackingRegime,
                         noHierarchyLevels,
                         noICPRunTillLevel,
                         distThresh,
                         terminationThreshold,
                         lowLevelEngine )
{

}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthMocapTracker_CPU::PreTrackCamera( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose )
{
    PreTrackCamera_mocap( trackingState, view, approxInvPose );
}

} // namespace Engine.

} // namespace ITMLib.
