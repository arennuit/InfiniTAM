// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker.h"
#include "../../ORUtils/Cholesky.h"

#include <math.h>

using namespace ITMLib::Engine;

////////////////////////////////////////////////////////////////////////////////
ITMDepthTracker::ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
	float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType)
{
    // Tracking regime (rotations, translations, or both) per level.
	viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);

    // Maximum number of iterations per level.
    this->noIterationsPerLevel = new int[noHierarchyLevels];

    this->noIterationsPerLevel[0] = 2; //TODO -> make parameter
	for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	{
		noIterationsPerLevel[levelId] = noIterationsPerLevel[levelId - 1] + 2;
	}

    this->distThresh = new float[noHierarchyLevels];
	float distThreshStep = distThresh / noHierarchyLevels;
	this->distThresh[noHierarchyLevels - 1] = distThresh;
	for (int levelId = noHierarchyLevels - 2; levelId >= 0; levelId--)
		this->distThresh[levelId] = this->distThresh[levelId + 1] - distThreshStep;

	this->lowLevelEngine = lowLevelEngine;

	this->noICPLevel = noICPRunTillLevel;

    // Threshold used in hasConverged().
	this->terminationThreshold = terminationThreshold;
}

////////////////////////////////////////////////////////////////////////////////
ITMDepthTracker::~ITMDepthTracker(void) 
{ 
	delete this->viewHierarchy;
	delete this->sceneHierarchy;

	delete[] this->noIterationsPerLevel;
	delete[] this->distThresh;
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view)
{
	this->trackingState = trackingState;
	this->view = view;

	sceneHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;
	viewHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	// the image hierarchy allows pointers to external data at level 0
	viewHierarchy->levels[0]->depth = view->depth;
	sceneHierarchy->levels[0]->pointsMap = trackingState->pointCloud->locations;
	sceneHierarchy->levels[0]->normalsMap = trackingState->pointCloud->colours;

	scenePose = trackingState->pose_pointCloud->GetM();
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::PrepareForEvaluation()
{
    // Subsample view and scene to prepare the depth maps at levels > 0.
    // NOTE: depth maps at level 0 are provided by the imager (view) and by the
    //       raytracer (scene).
	for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
        // View preparation.
        ITMTemplatedHierarchyLevel<ITMFloatImage> *currentLevelView  = viewHierarchy->levels[i];
        ITMTemplatedHierarchyLevel<ITMFloatImage> *previousLevelView = viewHierarchy->levels[i - 1];

		lowLevelEngine->FilterSubsampleWithHoles(currentLevelView->depth, previousLevelView->depth);

        currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;

        // Scene preparation.
        ITMSceneHierarchyLevel *currentLevelScene  = sceneHierarchy->levels[i];
        ITMSceneHierarchyLevel *previousLevelScene = sceneHierarchy->levels[i - 1];

        //lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->pointsMap, previousLevelScene->pointsMap);
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->normalsMap, previousLevelScene->normalsMap);

        currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
	this->iterationType = viewHierarchy->levels[levelId]->iterationType;
	this->sceneHierarchyLevel = sceneHierarchy->levels[0];
	this->viewHierarchyLevel = viewHierarchy->levels[levelId];
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (shortIteration)
	{
		float smallHessian[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);
	}
	else
	{
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);
	}
}

////////////////////////////////////////////////////////////////////////////////
bool ITMDepthTracker::HasConverged(float *step) const
{
	float stepLength = 0.0f;
	for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

	if (sqrt(stepLength) / 6 < terminationThreshold) return true; //converged

	return false;
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = 0.0f; step[4] = 0.0f; step[5] = 0.0f;
		break;
	case TRACKER_ITERATION_TRANSLATION:
		step[0] = 0.0f; step[1] = 0.0f; step[2] = 0.0f;
		step[3] = (float)(delta[0]); step[4] = (float)(delta[1]); step[5] = (float)(delta[2]);
		break;
	default:
	case TRACKER_ITERATION_BOTH:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = (float)(delta[3]); step[4] = (float)(delta[4]); step[5] = (float)(delta[5]);
		break;
	}

	Matrix4f Tinc;

    // WARNING: matrix use a colum-major convention (rather than the usual row-major).
    // Hence the coefficient look translated here (though they are not).
	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	para_new = Tinc * para_old;
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    // Prepare the view and scene.
    this->SetEvaluationData(trackingState, view);
    this->PrepareForEvaluation();

    Matrix4f approxInvPose = trackingState->pose_d->GetInvM();

    // Loop on levels.
	for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
    {
		this->SetEvaluationParams(levelId);
        if (iterationType == TRACKER_ITERATION_NONE)
            continue;

        // Loop on iterations per level.
        float f_old = 1e20f;
		for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
        {
            // Evaluate error function and gradients.
            float f_new;
            float hessian[6 * 6];
            float nabla[6];

            int validPointsNum = this->ComputeGandH(f_new, nabla, hessian, approxInvPose);

            // Check if error decreased.
            if ((validPointsNum <= 0) || (f_new > f_old))
            {
//                // Revert and stop iterating at the current level.
//                trackingState->pose_d->SetFrom(&lastKnownGoodPose);
//                approxInvPose = trackingState->pose_d->GetInvM();

//                lambda *= 10.0f;

                // DEBUG.
                std::cout << "!!!!!!!!!!!!!!!!!!!! ERROR INCREASED. f_old: " << f_old << " f_new: " << f_new << " !!!!!!!!!!!!!!!!!!!!" << std::endl;

                break;
            }

//            lastKnownGoodPose.SetFrom(trackingState->pose_d);

            f_old = f_new;

//            for (int i = 0; i < 6*6; ++i) hessian_good[i] = hessian_new[i] / noValidPoints_new;
//            for (int i = 0; i < 6; ++i) nabla_good[i] = nabla_new[i] / noValidPoints_new;

//            lambda /= 10.0f;

//            // DEBUG: no need to damp the hessian for inversion, as we now do an SVD.
//            for (int i = 0; i < 6*6; ++i) A[i] = hessian_good[i];
//            for (int i = 0; i < 6; ++i) A[i+i*6] *= 1.0f + lambda;

            // Compute the new camera frame.
            // NOTE: the result is orthonormalized (to make sure it belongs to SE3).
            float step[6];

            ComputeDelta(step, nabla, hessian, iterationType != TRACKER_ITERATION_BOTH);
			ApplyDelta(approxInvPose, step, approxInvPose);

			trackingState->pose_d->SetInvM(approxInvPose);
            trackingState->pose_d->Coerce(); // Orthonormalization.
            approxInvPose = trackingState->pose_d->GetInvM();

            // If step is small, assume it's going to decrease the error and finish.
            if (HasConverged(step))
                break;
		}
	}
}

