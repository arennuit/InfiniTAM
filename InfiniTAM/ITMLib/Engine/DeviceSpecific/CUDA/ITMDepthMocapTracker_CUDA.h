// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthMocapTracker.h"

namespace ITMLib
{
	namespace Engine
	{
        class ITMDepthMocapTracker_CUDA : public ITMDepthMocapTracker
		{
		public:
			struct AccuCell;

		private:
			AccuCell *accu_host;
			AccuCell *accu_device;

		protected:
			int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
            ITMDepthMocapTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine);
            ~ITMDepthMocapTracker_CUDA(void);
		};
	}
}
