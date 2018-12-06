// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMDepthTracker_CUDA.h"

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
class ITMDepthDefaultTracker_CUDA : public ITMDepthTracker_CUDA
{
public:
    ITMDepthDefaultTracker_CUDA( Vector2i imgSize,
                               TrackerIterationType *trackingRegime,
                               int noHierarchyLevels,
                               int noICPRunTillLevel,
                               float distThresh,
                               float terminationThreshold,
                               const ITMLowLevelEngine *lowLevelEngine );
    ~ITMDepthDefaultTracker_CUDA(void) {}

    void PreTrackCamera(ITMTrackingState *trackingState, const ITMView *view) override;
};

} // namespace Engine.

} // namespace ITMLib.
