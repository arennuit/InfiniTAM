// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

////////////////////////////////////////////////////////////////////////////////
void ITMTrackingController::PreTrack(ITMTrackingState *trackingState, const ITMView *view)
{
    // Set pose_d and approxInvPose from mocap
    if (trackingState->age_pointCloud != -1)
        tracker->PreTrackCamera(trackingState, view);
}

////////////////////////////////////////////////////////////////////////////////
void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
    // Track.
    if (trackingState->age_pointCloud != -1)
        tracker->TrackCamera(trackingState, view);

    // Handle need for full rendering.
	trackingState->requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
}

////////////////////////////////////////////////////////////////////////////////
void ITMTrackingController::RayTracing(ITMTrackingState *trackingState, const ITMView *view, ITMRenderState *renderState)
{
    // Raycast for tracking and visualization.
	if (settings->trackerType == ITMLibSettings::TRACKER_COLOR)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		visualisationEngine->CreateExpectedDepths(&pose_rgb, &(view->calib->intrinsics_rgb), renderState);
		visualisationEngine->CreatePointCloud(view, trackingState, renderState, settings->skipPoints);
		trackingState->age_pointCloud = 0;
	}
	else
	{
        // Render range image.
        visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

		if (trackingState->requiresFullRendering)
        {
            visualisationEngine->CreateICPMaps(view, trackingState, renderState);
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

            // Handle age.
            if (trackingState->age_pointCloud == -1)
                trackingState->age_pointCloud = -2;
            else
                trackingState->age_pointCloud = 0;
		}
		else
		{
			visualisationEngine->ForwardRender(view, trackingState, renderState);
			trackingState->age_pointCloud++;
		}
	}
}
