// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "ITMPixelUtils.h"

////////////////////////////////////////////////////////////////////////////////
template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth_Ab(THREADPTR(float) *A, THREADPTR(float) &b,
	const THREADPTR(int) & x, const THREADPTR(int) & y,
	const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
	const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
	const CONSTPTR(Vector4f) *normalsMap, float distThresh)
{
    // Check if depth is valid.
    if (depth <= 1e-8f)
        return false;

    // Compute s (expressed in frame k^z)
    // NOTE: inverse pinhole model (2D -> 3D).
    Vector4f s_kz;

    float fx = viewIntrinsics.x;
    float fy = viewIntrinsics.y;
    float cx = viewIntrinsics.z;
    float cy = viewIntrinsics.w;

    s_kz.x = depth * ((float(x) - cx) / fx);
    s_kz.y = depth * ((float(y) - cy) / fy);
    s_kz.z = depth;
    s_kz.w = 1.0f;
    
    // Express s in world frame.
    // NOTE: as we do not know frame k^z we approximate it by frame {k^(z-1)}.
    //       And the whole point for the optimization below is to make up for
    //       this approximation.
    // NOTE: approxInvPose = H_{k^(z-1)}_0.
    Vector4f s_0;

    s_0 = approxInvPose * s_kz;
    s_0.w = 1.0f; // Enforce w = 1.

    // Express s in frame k-1.
    // NOTE: scenePose = H_0_{k-1}.
    Vector4f s_km1;

    s_km1 = scenePose * s_0;

    if (s_km1.z <= 0.0f)
        return false;

    // Compute corresponding pixel in k-1.
    // NOTE: pinhole model.
    Vector2f d_2d;

    d_2d.x = sceneIntrinsics.x * s_km1.x / s_km1.z + sceneIntrinsics.z;
    d_2d.y = sceneIntrinsics.y * s_km1.y / s_km1.z + sceneIntrinsics.w;

    if (!((d_2d.x >= 0.0f) && (d_2d.x <= sceneImageSize.x - 2) && (d_2d.y >= 0.0f) && (d_2d.y <= sceneImageSize.y - 2)))
		return false;

    // Compute corresponding destination point d.
    // NOTE: it is expressed in frame 0.
    Vector4f d_0;

    d_0 = interpolateBilinear_withHoles(pointsMap, d_2d, sceneImageSize);

    if (d_0.w < 0.0f)
        return false;

    // Compute normal at d (expressed in frame 0).
    Vector4f n_0;
    n_0 = interpolateBilinear_withHoles(normalsMap, d_2d, sceneImageSize);

//    if (n_0.w < 0.0f)
//        return false;

    // Optimization: compute contribution to A and b.
    // NOTE: the optimization aligns s on d (via movement M).
    Vector3f ptDiff;

    ptDiff.x = d_0.x - s_0.x;
    ptDiff.y = d_0.y - s_0.y;
    ptDiff.z = d_0.z - s_0.z;

	float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

    if (dist > distThresh)
        return false;

    b = n_0.x * ptDiff.x + n_0.y * ptDiff.y + n_0.z * ptDiff.z;

	// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
	if (shortIteration)
	{
		if (rotationOnly)
		{
            A[0] = +s_0.z * n_0.y - s_0.y * n_0.z;
            A[1] = -s_0.z * n_0.x + s_0.x * n_0.z;
            A[2] = +s_0.y * n_0.x - s_0.x * n_0.y;
		}
        else { A[0] = n_0.x; A[1] = n_0.y; A[2] = n_0.z; }
	}
	else
	{
        A[0] = +s_0.z * n_0.y - s_0.y * n_0.z;
        A[1] = -s_0.z * n_0.x + s_0.x * n_0.z;
        A[2] = +s_0.y * n_0.x - s_0.x * n_0.y;
        A[!shortIteration ? 3 : 0] = n_0.x; A[!shortIteration ? 4 : 1] = n_0.y; A[!shortIteration ? 5 : 2] = n_0.z;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth(THREADPTR(float) *localNabla, THREADPTR(float) *localHessian, THREADPTR(float) &localF,
	const THREADPTR(int) & x, const THREADPTR(int) & y,
	const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
	const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
	const CONSTPTR(Vector4f) *normalsMap, float distThresh)
{
	const int noPara = shortIteration ? 3 : 6;
	float A[noPara];
	float b;

	bool ret = computePerPointGH_Depth_Ab<shortIteration,rotationOnly>(A, b, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);

	if (!ret) return false;

	localF = b * b;

#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
	for (int r = 0, counter = 0; r < noPara; r++)
	{
		localNabla[r] = b * A[r];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = A[r] * A[c];
	}

	return true;
}

