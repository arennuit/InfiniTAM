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
    void getMeasurement(Eigen::Frame& mocapFrame) {mocapFrame = m_cachedFrame;}

private:
    std::string m_mocapFilename;

    std::ifstream m_fileStream;

    Eigen::Frame m_cachedFrame;

    void loadMocapIntoCache();
};

} // namespace Engine.

} // namespace InfiniTAM

