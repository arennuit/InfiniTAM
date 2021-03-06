#pragma once

#include "MocapSourceEngine.h"

#include <fstream>
#include <string>

namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
class MocapFileEngine : public MocapSourceEngine
{
public:
    MocapFileEngine(std::string const& mocapFilename);
    ~MocapFileEngine();

    bool hasMoreMeasurements();
    MeasurementStatus getMeasurement(Eigen::Framef& mocapFrame)
    {
        mocapFrame = m_cachedFrame;
        return MEASUREMENT_OK;
    }

private:
    std::string m_mocapFilename;

    std::ifstream m_fileStream;

    Eigen::Framef m_cachedFrame;

    void loadMocapIntoCache();
};

} // namespace Engine.

} // namespace InfiniTAM

