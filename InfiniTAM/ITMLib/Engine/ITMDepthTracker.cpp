// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker.h"
#include "../../ORUtils/Cholesky.h"
#include "../Objects/ITMViewMocap.h"

#include <unsupported/Eigen/MatrixFunctions>
//#include "Core/EigenExtensions/EigenGeometryAndPlugins.h"

#include <math.h>
#include <iomanip>

namespace ITMLib
{

namespace Engine
{

////////////////////////////////////////////////////////////////////////////////
ITMDepthTracker::ITMDepthTracker( Vector2i imgSize,
                                  TrackerIterationType *trackingRegime,
                                  int noHierarchyLevels,
                                  int noICPRunTillLevel,
                                  float distThresh,
                                  float terminationThreshold,
                                  const ITMLowLevelEngine *lowLevelEngine,
                                  MemoryDeviceType memoryType ) :
    ITMTracker ()
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

    // ?.
    this->distThresh = new float[noHierarchyLevels];
	float distThreshStep = distThresh / noHierarchyLevels;
	this->distThresh[noHierarchyLevels - 1] = distThresh;
	for (int levelId = noHierarchyLevels - 2; levelId >= 0; levelId--)
		this->distThresh[levelId] = this->distThresh[levelId + 1] - distThreshStep;

    // ?.
	this->lowLevelEngine = lowLevelEngine;

    // .
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
        // Convert the 6x6 hessian into a 3x3 hessian.
		float smallHessian[3 * 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                smallHessian[r + c * 3] = hessian[r + c * 6];

        // Solve.
		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);

//        // Compute the SVD.
//        Eigen::Matrix<float, 3, 3> AtA;
//        AtA << smallHessian[ 0], smallHessian[ 1], smallHessian[ 2],
//               smallHessian[ 3], smallHessian[ 4], smallHessian[ 5],
//               smallHessian[ 6], smallHessian[ 7], smallHessian[ 8];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd( AtA, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "---------" << std::endl;
//        std::cout << "Iteration type : " << iterationType << std::endl;
//        Eigen::Matrix<float, 3, 1> sv = svd.singularValues();
//        float divider = sv(0);
//        for ( uint i = 0; i < sv.rows(); ++i)
//            sv(i) /= divider;
//        std::cout << "Its singular values are:" << std::endl << sv << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd.matrixV() << std::endl;
	}
	else
	{
        // Solve.
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);

//        // Compute the 6x6 SVD.
//        Eigen::Matrix<float, 6, 6> AtA;
//        AtA << hessian[ 0], hessian[ 1], hessian[ 2], hessian[ 3], hessian[ 4], hessian[ 5],
//               hessian[ 6], hessian[ 7], hessian[ 8], hessian[ 9], hessian[10], hessian[11],
//               hessian[12], hessian[13], hessian[14], hessian[15], hessian[16], hessian[17],
//               hessian[18], hessian[19], hessian[20], hessian[21], hessian[22], hessian[23],
//               hessian[24], hessian[25], hessian[26], hessian[27], hessian[28], hessian[29],
//               hessian[30], hessian[31], hessian[32], hessian[33], hessian[34], hessian[35];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 6, 6> > svd( AtA, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "---------" << std::endl;
//        std::cout << "Iteration type : " << iterationType << std::endl;
//        Eigen::Matrix<float, 6, 1> sv = svd.singularValues();
//        float divider = sv(0);
//        for ( uint i = 0; i < sv.rows(); ++i)
//            sv(i) /= divider;
//        std::cout << "Its singular values are:" << std::endl << sv << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd.matrixV() << std::endl;
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
    Tinc.m00 =  1.0f;		Tinc.m10 = -step[2];	Tinc.m20 =  step[1];	Tinc.m30 = step[3];
    Tinc.m01 =  step[2];	Tinc.m11 =  1.0f;		Tinc.m21 = -step[0];	Tinc.m31 = step[4];
    Tinc.m02 = -step[1];	Tinc.m12 =  step[0];	Tinc.m22 =  1.0f;		Tinc.m32 = step[5];
    Tinc.m03 =  0.0f;		Tinc.m13 =  0.0f;		Tinc.m23 =  0.0f;		Tinc.m33 = 1.0f;

    para_new = Tinc * para_old;
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::PreTrackCamera_default( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose )
{
    approxInvPose = trackingState->pose_d->GetInvM();
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::PreTrackCamera_mocap( ITMTrackingState *trackingState, const ITMView *view, Matrix4f& approxInvPose )
{
//    //////////////////////////////////
//    // Offset approach

//    // Init approxInvPose.
//    approxInvPose = trackingState->pose_d->GetInvM();

//    // Compute offset f_cam_cam_km1_fromVive from the vive.
//    ITMViewMocap* mocapView = (ITMViewMocap*)view;
//    Eigen::Framef f_cam_vive;
//    PoseToFrame( f_cam_vive, view->calib->m_h_cam_beacon.calib );
//    static Eigen::Framef f_cam0_mocapBase = mocapView->m_f_tracker_mocapBase * f_cam_vive;
//    Eigen::Framef f_cam_cam0_fromVive = f_cam0_mocapBase.getInverse() * mocapView->m_f_tracker_mocapBase * f_cam_vive;
//    static Eigen::Framef f_cam_km1_cam0_fromVive = f_cam_cam0_fromVive;
//    Eigen::Framef f_cam_cam_km1_fromVive = f_cam_km1_cam0_fromVive.getInverse() * f_cam_cam0_fromVive;

//    // Apply offset f_cam_cam_km1_fromVive.
//    ITMPose pose_cam_km1_cam0( approxInvPose );
//    Eigen::Framef f_cam_km1_cam0;
//    PoseToFrame( f_cam_km1_cam0, pose_cam_km1_cam0 );
//    Eigen::Framef f_cam_cam0 = f_cam_km1_cam0 * f_cam_cam_km1_fromVive;
//    ITMPose pose_cam_cam0;
//    FrameToPose( pose_cam_cam0, f_cam_cam0 );
//    approxInvPose = pose_cam_cam0.GetM();

//    f_cam_km1_cam0_fromVive = f_cam_cam0_fromVive;

    //////////////////////////////////
    // Global approach

    // Init approxInvPose.
    approxInvPose = trackingState->pose_d->GetInvM();

    // Pre-positioning: compute mocap-based estimation of f_cam_cam0.
    ITMViewMocap* mocapView = (ITMViewMocap*)view;
    Eigen::Framef f_cam_tracker;
    PoseToFrame( f_cam_tracker, view->calib->m_h_cam_beacon.calib );
    static Eigen::Framef f_cam0_mocapBase = mocapView->m_f_tracker_mocapBase * f_cam_tracker;
    Eigen::Framef f_cam_cam0 = f_cam0_mocapBase.getInverse() * mocapView->m_f_tracker_mocapBase * f_cam_tracker;

    ITMPose pose_cam_cam0;
    FrameToPose( pose_cam_cam0, f_cam_cam0 );
    approxInvPose = pose_cam_cam0.GetM();
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
    // Prepare the view and scene.
    this->SetEvaluationData(trackingState, view);
    this->PrepareForEvaluation();

    // PreTrack
    Matrix4f approxInvPose;
    PreTrackCamera( trackingState, view, approxInvPose );

    trackingState->pose_d->SetInvM(approxInvPose);
    trackingState->pose_d->Coerce(); // Orthonormalization.
    approxInvPose = trackingState->pose_d->GetInvM();

    // Loop on levels.
    float f_new;
    float hessian_raw[6 * 6];
    float hessian_norm[6 * 6];
    float hessian_damp[6 * 6];
    float nabla_raw[6];
    float nabla_norm[6];
    for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
    {
        this->SetEvaluationParams(levelId);
        if (iterationType == TRACKER_ITERATION_NONE)
            continue;

        std::cout << "---------" << std::endl;

        // Loop on iterations per level.
//        ITMPose lastKnownGoodPose( *(trackingState->pose_d) );
        float f_old = 1e20f;
        float lambda = 1.0;
        for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
        {
            // Evaluate gradients and error function.
            int validPointsNum = this->ComputeGandH(f_new, nabla_raw, hessian_raw, approxInvPose);

            // Check if error increased.
            // NOTE: if so revert and do not update hessian_norm.
            if ( (validPointsNum <= 0) || (f_new > f_old) )
            {
                // Revert.
//                trackingState->pose_d->SetFrom(&lastKnownGoodPose);
//                approxInvPose = trackingState->pose_d->GetInvM();

                // Increase damping.
                // NOTE : We are likely close to a singularity of the hessian.
                lambda *= 10.0f;

                // Cout ill-conditioning: error increased.
                std::cout << "\033[1;31m";
                std::cout << "\n   " << std::setw(2) << levelId << std::setw(7) << iterNo << " !!!!!!!!!!!!!!!!!!!! ERROR INCREASED. validPointsNum : " << validPointsNum << " f_old: " << f_old << " f_new: " << f_new << " !!!!!!!!!!!!!!!!!!!!";
                std::cout << "\033[0m\n";

                // HACK - Antoine Rennuit.
                if (trackingState->m_frameIdx == 0)
                    break;
            }
            else
            {
                // Get ready for next step.
//                lastKnownGoodPose.SetFrom( trackingState->pose_d );
                f_old = f_new;

                // Normalize the hessian and gradient.
                // NOTE: normalization according to the number of points evaluted.
                for (int i = 0; i < 6*6; ++i)
                    hessian_norm[i] = hessian_raw[i] / validPointsNum;
                for (int i = 0; i < 6; ++i)
                    nabla_norm[i] = nabla_raw[i] / validPointsNum;

                // Lower damping.
                // NOTE : we are likely away from any singularity of the hessian.
                lambda /= 10.0f;
            }

            // Damp the hessian.
            // NOTE: this should be no longer needed when the SVD is used.
            for (int i = 0; i < 6*6; ++i)
                hessian_damp[i] = hessian_norm[i];
            for (int i = 0; i < 6; ++i)
                hessian_damp[i+i*6] *= 1.0f + lambda;

            // Compute the new camera frame.
            // NOTE: the result is orthonormalized (to make sure it belongs to SE3).
            float step[6];

            ComputeDelta(step, nabla_norm, hessian_damp, iterationType != TRACKER_ITERATION_BOTH);
            ApplyDelta(approxInvPose, step, approxInvPose);

            trackingState->pose_d->SetInvM(approxInvPose);
            trackingState->pose_d->Coerce(); // Orthonormalization.
            approxInvPose = trackingState->pose_d->GetInvM();

            // If step is small, assume it's going to decrease the error and finish.
            if (HasConverged(step))
                break;
        }
    }

//    // Decompose space into useful and useless components using an SVD.
//    if ( iterationType != TRACKER_ITERATION_BOTH )
//    {
//        // Convert the 6x6 hessian into a 3x3 hessian.
//        float smallHessian[3 * 3];
//        for (int r = 0; r < 3; r++)
//            for (int c = 0; c < 3; c++)
//                smallHessian[r + c * 3] = hessian[r + c * 6];

//        // Compute the SVD.
//        Eigen::Matrix<float, 3, 3> AtA;
//        AtA << smallHessian[ 0], smallHessian[ 1], smallHessian[ 2],
//               smallHessian[ 3], smallHessian[ 4], smallHessian[ 5],
//               smallHessian[ 6], smallHessian[ 7], smallHessian[ 8];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd( AtA, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd.matrixV() << std::endl;
//    }
//    else
//    {
//        // Compute the SVD on rotations.
//        Eigen::Matrix<float, 3, 3> AtA_rot;
//        AtA_rot << hessian[ 0], hessian[ 1], hessian[ 2],
//                   hessian[ 6], hessian[ 7], hessian[ 8],
//                   hessian[12], hessian[13], hessian[14];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd_rot( AtA_rot, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "ROTATIONS" << std::endl;
//        Eigen::Matrix<float, 3, 1> sv_rot = svd_rot.singularValues();
//        float divider_rot = sv_rot(0);
//        for ( uint i = 0; i < sv_rot.rows(); ++i)
//            sv_rot(i) /= divider_rot;
//        std::cout << "Its singular values are:" << std::endl << sv_rot << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd_rot.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd_rot.matrixV() << std::endl;
//        std::cout << std::endl;

//        // Compute the SVD on translations.
//        Eigen::Matrix<float, 3, 3> AtA_trans;
//        AtA_trans << hessian[21], hessian[22], hessian[23],
//                     hessian[27], hessian[28], hessian[29],
//                     hessian[33], hessian[34], hessian[35];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 3, 3> > svd_trans( AtA_trans, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "TRANSLATIONS" << std::endl;
//        Eigen::Matrix<float, 3, 1> sv_trans = svd_trans.singularValues();
//        float divider_trans = sv_trans(0);
//        for ( uint i = 0; i < sv_trans.rows(); ++i)
//            sv_trans(i) /= divider_trans;
//        std::cout << "Its singular values are:" << std::endl << sv_trans << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd_trans.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd_trans.matrixV() << std::endl;
//        std::cout << std::endl;

//        // Compute the 6x6 SVD.
//        Eigen::Matrix<float, 6, 6> AtA;
//        AtA << hessian[ 0], hessian[ 1], hessian[ 2], hessian[ 3], hessian[ 4], hessian[ 5],
//               hessian[ 6], hessian[ 7], hessian[ 8], hessian[ 9], hessian[10], hessian[11],
//               hessian[12], hessian[13], hessian[14], hessian[15], hessian[16], hessian[17],
//               hessian[18], hessian[19], hessian[20], hessian[21], hessian[22], hessian[23],
//               hessian[24], hessian[25], hessian[26], hessian[27], hessian[28], hessian[29],
//               hessian[30], hessian[31], hessian[32], hessian[33], hessian[34], hessian[35];
//        Eigen::JacobiSVD<Eigen::Matrix<float, 6, 6> > svd_both( AtA, Eigen::ComputeFullU | Eigen::ComputeFullV  );
//        std::cout << "BOTH" << std::endl;
//        Eigen::Matrix<float, 6, 1> sv_both = svd_both.singularValues();
//        float divider_both = sv_both(0);
//        for ( uint i = 0; i < sv_both.rows(); ++i)
//            sv_both(i) /= divider_both;
//        std::cout << "Its singular values are:" << std::endl << sv_both << std::endl;
//        std::cout << "Its Matrix U is:" << std::endl << svd_both.matrixU() << std::endl;
//        std::cout << "Its Matrix V is:" << std::endl << svd_both.matrixV() << std::endl;
//    }
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::FrameToPose(ITMPose& pose, Eigen::Framef const& frame)
{
    Matrix4f pose_mat;
    FrameToMat(pose_mat, frame);

    pose.SetM(pose_mat);
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::PoseToFrame(Eigen::Framef& frame, ITMPose const& pose)
{
    Matrix4f pose_mat = pose.GetM();
    Eigen::Matrix4f frame_mat;
    frame_mat << pose_mat.m[0], pose_mat.m[4], pose_mat.m[ 8], pose_mat.m[12],
                 pose_mat.m[1], pose_mat.m[5], pose_mat.m[ 9], pose_mat.m[13],
                 pose_mat.m[2], pose_mat.m[6], pose_mat.m[10], pose_mat.m[14],
                 pose_mat.m[3], pose_mat.m[7], pose_mat.m[11], pose_mat.m[15];

    frame = Eigen::Framef(frame_mat);
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::FrameToMat(Matrix4f& mat, Eigen::Framef const& frame)
{
    Eigen::Matrix4f frame_mat = frame.toMatrix4();
    mat.m[0] = frame_mat(0, 0); mat.m[4] = frame_mat(0, 1); mat.m[ 8] = frame_mat(0, 2); mat.m[12] = frame_mat(0, 3);
    mat.m[1] = frame_mat(1, 0), mat.m[5] = frame_mat(1, 1), mat.m[ 9] = frame_mat(1, 2), mat.m[13] = frame_mat(1, 3),
    mat.m[2] = frame_mat(2, 0), mat.m[6] = frame_mat(2, 1), mat.m[10] = frame_mat(2, 2), mat.m[14] = frame_mat(2, 3),
    mat.m[3] = frame_mat(3, 0), mat.m[7] = frame_mat(3, 1), mat.m[11] = frame_mat(3, 2), mat.m[15] = frame_mat(3, 3);
}

////////////////////////////////////////////////////////////////////////////////
void ITMDepthTracker::PrintPose(ITMPose& pose)
{
    Vector3f t, r;
    pose.GetParams(t, r);

    std::cout << std::setw(10) << t.x << " " << std::setw(10) << t.y << " " << std::setw(10) << t.z << " " << std::setw(10) << r.x << " " << std::setw(10) << r.y << " " << std::setw(10) << r.z << std::endl;
}

} // namespace Engine.

} // namespace ITMLib
