// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMDepthTracker_CPU.h"

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
class ITMDepthMocapTracker_CPU : public ITMDepthTracker_CPU
{
public:
    ITMDepthMocapTracker_CPU( Vector2i imgSize,
                              TrackerIterationType *trackingRegime,
                              int noHierarchyLevels,
                              int noICPRunTillLevel,
                              float distThresh,
                              float terminationThreshold,
                              const ITMLowLevelEngine *lowLevelEngine );
    ~ITMDepthMocapTracker_CPU( ) {}

    void PreTrackCamera(ITMTrackingState *trackingState, const ITMView *view) override;
};

} // namespace Engine.

} // namespace ITMLib.