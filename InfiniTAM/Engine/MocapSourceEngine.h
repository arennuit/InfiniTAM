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
    virtual ~MocapSourceEngine();

    virtual bool hasMoreMeasurements() = 0;
    virtual void getMeasurement(Eigen::Framef& mocapFrame) = 0;
};

} // namespace Engine.

} // namespace InfiniTAM.

