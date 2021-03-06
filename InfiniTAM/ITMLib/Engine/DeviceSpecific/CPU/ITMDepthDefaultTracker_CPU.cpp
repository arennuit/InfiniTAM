// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthDefaultTracker_CPU.h"

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthDefaultTracker_CPU::ITMDepthDefaultTracker_CPU( Vector2i imgSize,
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
void ITMDepthDefaultTracker_CPU::PreTrackCamera( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose )
{
    PreTrackCamera_default( trackingState, view, approxInvPose );
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthDefaultTracker_CPU::TrackCamera( ITMTrackingState *trackingState, const ITMView *view )
{
    TrackCamera_default( trackingState, view );
}

} // namespace Engine.

} // namespace ITMLib.
