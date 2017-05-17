#include "MocapFileEngine.h"

#include <fstream>

namespace InfiniTAM
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
MocapFileEngine::MocapFileEngine(std::string const& mocapFilename) :
    MocapSourceEngine(),
    m_mocapFilename(mocapFilename)
{
//    m_currentFrameNo = 0;
//    m_cachedFrameNo = -1;

    std::ifstream src(mocapFilename.c_str());
}

////////////////////////////////////////////////////////////////////////////////
void MocapFileEngine::loadMocapIntoCache()
{
//    // Check the cached frame was read.
//    if (m_currentFrameNo == m_cachedFrameNo)
//        return;

//    m_cachedFrameNo = m_currentFrameNo;


}

////////////////////////////////////////////////////////////////////////////////
bool MocapFileEngine::hasMoreMeasurements()
{
    loadMocapIntoCache();

//    return ((cached_rgb!=NULL) && (cached_depth!=NULL));
    return true;
}

} // namespace Engine.

} // namespace InfiniTAM.
