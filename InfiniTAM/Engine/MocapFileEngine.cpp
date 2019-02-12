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

    Eigen::Framef f_tracker_base( qw, qx, qy, qz, x, y, z );
    Eigen::Vector3f r_tracker_base = f_tracker_base.m_quat.toRotationVector();
    Eigen::Vector3f p_tracker_base = f_tracker_base.m_pos;

    // Default format.
    Eigen::Vector3f r_tracker_base_corrected( r_tracker_base.x(), r_tracker_base.y(), r_tracker_base.z() );
    Eigen::Vector3f p_tracker_base_corrected( p_tracker_base.x(), p_tracker_base.y(), p_tracker_base.z() );

//    // ICL-NUIM format.
//    Eigen::Vector3f r_tracker_base_corrected( -r_tracker_base.x(),  r_tracker_base.y(), -r_tracker_base.z() );
//    Eigen::Vector3f p_tracker_base_corrected(  p_tracker_base.x(), -p_tracker_base.y(),  p_tracker_base.z() );

    m_cachedFrame.m_quat.fromRotationVector(r_tracker_base_corrected);
    m_cachedFrame.m_pos = p_tracker_base_corrected;

    return true;
}

} // namespace Engine.

} // namespace InfiniTAM.
