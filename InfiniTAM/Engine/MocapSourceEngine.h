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
    enum MeasurementStatus
    {
        MEASUREMENT_OK,
        MEASUREMENT_CALIBRATING,
        MEASUREMENT_OUT_OF_RANGE,
        /// \brief Fallback error code, when not out of range or calibrating
        MEASUREMENT_ERROR
    };

    virtual ~MocapSourceEngine();

    virtual bool hasMoreMeasurements() = 0;
    virtual MeasurementStatus getMeasurement(Eigen::Framef& mocapFrame) = 0;
};

} // namespace Engine.

} // namespace InfiniTAM.

