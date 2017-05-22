#include "MocapFileEngine.h"


namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
MocapFileEngine::MocapFileEngine(std::string const& mocapFilename) :
    MocapSourceEngine(),
    m_mocapFilename(mocapFilename),
    m_fileStream(mocapFilename.c_str())
{

}

////////////////////////////////////////////////////////////////////////////////
MocapFileEngine::~MocapFileEngine()
{
    m_fileStream.close();
}

////////////////////////////////////////////////////////////////////////////////
bool MocapFileEngine::hasMoreMeasurements()
{
    uint idx;
    float x, y, z,
          qx, qy, qz, qw;
    m_fileStream >> idx >> x >> y >> z >> qx >> qy >> qz >> qw;

    m_cachedFrame.setWXYZ(qw, qx, qy, qz, x, y, z);

    return true;
}

} // namespace Engine.

} // namespace InfiniTAM.
