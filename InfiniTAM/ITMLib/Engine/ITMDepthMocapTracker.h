// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Engine/ITMDepthTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

using namespace ITMLib::Objects;

namespace ITMLib
{

namespace Engine
{

/** Base class for engine performing ICP/mocap based depth tracking.
    A typical example would be the original KinectFusion
    tracking algorithm with a pre-conditionning based on a 6D beacon.
*/
class ITMDepthMocapTracker : public ITMDepthTracker
{
public:
    void PreTrackCamera( ITMTrackingState *trackingState, const ITMView *view);

    ITMDepthMocapTracker( Vector2i imgSize,
                          TrackerIterationType *trackingRegime,
                          int noHierarchyLevels,
                          int noICPRunTillLevel,
                          float distThresh,
                          float terminationThreshold,
                          const ITMLowLevelEngine *lowLevelEngine,
                          MemoryDeviceType memoryType );
    virtual ~ITMDepthMocapTracker(void) {}
};

} // namespace Engine.

} // namespace ITMLib.
