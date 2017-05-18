#pragma once

#include "../ITMLib/ITMLib.h"

#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
class MocapSourceEngine
{
public:
    MocapSourceEngine() {}
    ~MocapSourceEngine() {}

    virtual bool hasMoreMeasurements() = 0;
    void getMeasurement(Eigen::Frame& mocapFrame) = 0;
};

} // namespace Engine.

} // namespace InfiniTAM.

