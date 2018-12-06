// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthMocapTracker.h"

namespace ITMLib
{
	namespace Engine
	{
        class ITMDepthMocapTracker_CPU : public ITMDepthMocapTracker
		{
		protected:
			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
            ITMDepthMocapTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
            ~ITMDepthMocapTracker_CPU(void);
		};
	}
}
