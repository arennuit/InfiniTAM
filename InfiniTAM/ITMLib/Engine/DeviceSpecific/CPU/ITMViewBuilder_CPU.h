// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_CPU final : public ITMViewBuilder
		{
		protected:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const ITMIntrinsics *depthIntrinsics, 
				Vector2f disparityCalibParams);
			void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams);

			void DepthCorrection(ITMFloatImage *image) override;

			void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic);

		public:
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement);
            void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, Eigen::Framef* mocapMeasurement);

			ITMViewBuilder_CPU(const ITMRGBDCalib *calib);
			~ITMViewBuilder_CPU(void);
		};
	}
}
