#pragma once

#include "MocapSourceEngine.h"

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
    ~MocapFileEngine() { }

    bool hasMoreMeasurements();
//    void getMeasurement(Eigen::Frame& mocapFrame) {return m_}

private:
    std::string m_mocapFilename;

//    bool m_isMeasurementNew;
//    Eigen::Frame m_cachedFrame;

    void loadMocapIntoCache();
//    int m_cachedFrameNo;
//    int m_currentFrameNo;
};

} // namespace Engine.

} // namespace InfiniTAM

