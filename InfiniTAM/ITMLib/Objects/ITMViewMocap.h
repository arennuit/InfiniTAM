// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMView.h"

#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

namespace ITMLib
{

namespace Objects
{

// Adds the mocap frame to the view.
class ITMViewMocap : public ITMView
{
public:
    Eigen::Framef m_mocapFrame;

    ITMViewMocap(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
     : ITMView(calibration, imgSize_rgb, imgSize_d, useGPU)
    { }

    ~ITMViewMocap() {}

    // Suppress the default copy constructor and assignment operator
    ITMViewMocap(const ITMViewMocap&) = delete;
    ITMViewMocap& operator=(const ITMViewMocap&) = delete;
};

} // namespace Objects.

} // namespace ITMLib.
