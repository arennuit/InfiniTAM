// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

// DEBUG.
#include <fstream>
#include <string>

using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Base class for engine performing ICP based depth tracking.
		    A typical example would be the original KinectFusion
		    tracking algorithm.
		*/
		class ITMDepthTracker : public ITMTracker
		{
        protected:
			const ITMLowLevelEngine *lowLevelEngine;
			ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

			ITMTrackingState *trackingState; const ITMView *view;

			int *noIterationsPerLevel;
			int noICPLevel;

			float terminationThreshold;

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
			bool HasConverged(float *step) const;

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);

			float *distThresh;

			int levelId;
			TrackerIterationType iterationType;

            Matrix4f scenePose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

            ITMPose       FrameToPose( Eigen::Framef const& frame );
            Eigen::Framef PoseToFrame( ITMPose const& pose );
            Matrix4f      FrameToMat( Eigen::Framef const& frame );
            Eigen::Framef MatToFrame( Matrix4f const& mat );
            void PrintPose( ITMPose& pose );

            void computeSVD_3R( float const * hessian, Eigen::Matrix<float, 3, 3>& U, Eigen::Matrix<float, 3, 1>& S, Eigen::Matrix<float, 3, 3>& V );
            void computeSVD_3T( float const * hessian, Eigen::Matrix<float, 3, 3>& U, Eigen::Matrix<float, 3, 1>& S, Eigen::Matrix<float, 3, 3>& V );
            void computeSVD_6(  float const * hessian, Eigen::Matrix<float, 6, 6>& U, Eigen::Matrix<float, 6, 1>& S, Eigen::Matrix<float, 6, 6>& V );

            virtual void PreTrackCamera( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose ) = 0;
            void PreTrackCamera_default( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose );
            void PreTrackCamera_mocap(   ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose );

        public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);
		};
	}
}
