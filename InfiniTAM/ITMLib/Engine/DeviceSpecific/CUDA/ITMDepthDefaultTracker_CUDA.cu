// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthDefaultTracker_CUDA.h"

using namespace ITMLib::Engine;

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthDefaultTracker_CUDA::ITMDepthDefaultTracker_CUDA( Vector2i imgSize,
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
void ITMDepthDefaultTracker_CUDA::PreTrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    PreTrackCamera_default( trackingState, view );
}


} // namespace Engine.

} // namespace ITMLib.
