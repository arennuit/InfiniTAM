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
    void getMeasurement(Eigen::Frame& mocapFrame) {mocapFrame = m_cachedFrame; m_cachedFrame.m_pos.x() = 5.0f;}

private:
    std::string m_mocapFilename;

//    bool m_isMeasurementNew;
    Eigen::Frame m_cachedFrame;

    void loadMocapIntoCache();
//    int m_cachedFrameNo;
//    int m_currentFrameNo;
};

} // namespace Engine.

} // namespace InfiniTAM

