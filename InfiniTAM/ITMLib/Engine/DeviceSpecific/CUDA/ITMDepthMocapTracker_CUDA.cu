// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthMocapTracker_CUDA.h"

using namespace ITMLib::Engine;

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthMocapTracker_CUDA::ITMDepthMocapTracker_CUDA( Vector2i imgSize,
                                                      TrackerIterationType *trackingRegime,
                                                      int noHierarchyLevels,
                                                      int noICPRunTillLevel,
                                                      float distThresh,
                                                      float terminationThreshold,
                                                      const ITMLowLevelEngine *lowLevelEngine ) :
    ITMDepthTracker_CUDA( imgSize,
                          trackingRegime,
                          noHierarchyLevels,
                          noICPRunTillLevel,
                          distThresh,
                          terminationThreshold,
                          lowLevelEngine )
{

}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthMocapTracker_CUDA::PreTrackCamera( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose )
{
    PreTrackCamera_mocap( trackingState, view, approxInvPose );
}


} // namespace Engine.

} // namespace ITMLib.
