// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMCalibIO.h"
#include "ITMDepthCorrectionIO.h"

#include <fstream>
#include <sstream>
#include <typeinfo>

using namespace ITMLib::Objects;

bool ITMLib::Objects::readIntrinsics(std::istream & src, ITMIntrinsics & dest)
{
	float sizeX, sizeY;
	float focalLength[2], centerPoint[2];

	src >> sizeX >> sizeY;
	src >> focalLength[0] >> focalLength[1];
	src >> centerPoint[0] >> centerPoint[1];

	if (src.fail()) return false;

	dest.SetFrom(focalLength[0], focalLength[1], centerPoint[0], centerPoint[1], sizeX, sizeY);

    // Display.
    std::cout << "Objects::readIntrinsics() : " << std::endl;
    std::cout << "sizeX : " << sizeX << std::endl;
    std::cout << "sizeY : " << sizeY << std::endl;
    std::cout << "focalLength[0] : " << focalLength[0] << std::endl;
    std::cout << "focalLength[1] : " << focalLength[1] << std::endl;
    std::cout << "centerPoint[0] : " << centerPoint[0] << std::endl;
    std::cout << "centerPoint[1] : " << centerPoint[1] << std::endl;
    std::cout << std::endl;

	return true;
}

bool ITMLib::Objects::readIntrinsics(const char *fileName, ITMIntrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::Objects::readIntrinsics(f, dest);
}

bool ITMLib::Objects::readExtrinsics(std::istream & src, ITMExtrinsics & dest)
{
	Matrix4f calib;
	src >> calib.m00 >> calib.m10 >> calib.m20 >> calib.m30;
	src >> calib.m01 >> calib.m11 >> calib.m21 >> calib.m31;
	src >> calib.m02 >> calib.m12 >> calib.m22 >> calib.m32;
	calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f; calib.m33 = 1.0f;
	if (src.fail()) return false;

	dest.SetFrom(calib);

    // Display.
    std::cout << "Objects::readExtrinsics() : " << std::endl;;
    std::cout << "calib : " << std::endl;
    std::cout << calib << std::endl;
    std::cout << std::endl;

	return true;
}

bool ITMLib::Objects::readExtrinsics(const char *fileName, ITMExtrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::Objects::readExtrinsics(f, dest);
}

bool ITMLib::Objects::readDisparityCalib(std::istream & src, ITMDisparityCalib & dest)
{
	std::string word;
	src >> word;
	if (src.fail()) return false;

	ITMDisparityCalib::TrafoType type = ITMDisparityCalib::TRAFO_KINECT;
	float a,b;

    if (word.compare("kinect") == 0)
    {
		type = ITMDisparityCalib::TRAFO_KINECT;
		src >> a;
    }
    else if (word.compare("affine") == 0)
    {
		type = ITMDisparityCalib::TRAFO_AFFINE;
		src >> a;
    }
    else
    {
		std::stringstream wordstream(word);
		wordstream >> a;
        if (wordstream.fail()) return false;
	}

	src >> b;
    if (src.fail()) return false;

    if ((a == 0.0f) && (b == 0.0f))
    {
		type = ITMDisparityCalib::TRAFO_AFFINE;
		a = 1.0f/1000.0f; b = 0.0f;
    }

	dest.SetFrom(a, b, type);

    // Display.
    std::cout << "Objects::readDisparityCalib() : " << std::endl;;
    std::cout << "   a: " << a << std::endl;
    std::cout << "   b: " << b << std::endl;
    std::cout << std::endl;

	return true;
}

bool ITMLib::Objects::readDisparityCalib(const char *fileName, ITMDisparityCalib & dest)
{
	std::ifstream f(fileName);
	return ITMLib::Objects::readDisparityCalib(f, dest);
}

bool ITMLib::Objects::readRGBDCalib(std::istream & src, ITMRGBDCalib & dest)
{
    // Display.
    std::cout << "------------------------------------------------------------------------" << std::endl;

    // Actually read calib.
    if (!ITMLib::Objects::readIntrinsics(src, dest.intrinsics_rgb)) return false;
    if (!ITMLib::Objects::readIntrinsics(src, dest.intrinsics_d)) return false;
    if (!ITMLib::Objects::readExtrinsics(src, dest.trafo_rgb_to_depth)) return false;
    if (!ITMLib::Objects::readDisparityCalib(src, dest.disparityCalib)) return false;
    if (!ITMLib::Objects::readExtrinsics(src, dest.m_h_cam_beacon)) return false;
    if (!ITMLib::Objects::readDepthCorrection(src, dest.depth_correction)) return false;
	return true;
}

bool ITMLib::Objects::readRGBDCalib(const char *fileName, ITMRGBDCalib & dest)
{
	std::ifstream f(fileName);
	return ITMLib::Objects::readRGBDCalib(f, dest);
}

template <typename CorrectionModel>
bool readCorrectionModel(std::istream & src, Size2 imageSize, CorrectionModel &dest)
{
    try {
        dest = CorrectionModelDeserializer<CorrectionModel>::deserialize(src, imageSize);
    } catch (std::exception &e) {
        std::clog << "Could not parse " << typeid(CorrectionModel).name() << ": " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool ITMLib::Objects::readDepthCorrection(std::istream & src, DepthCorrectionModel &dest)
{
	Size2 imageSize;

	if (!(src >> imageSize.x() >> imageSize.y())) {
		std::clog << "Could not read image size for depth correction" << std::endl;
		return false;
	}

	DepthCorrectionModel::GlobalModel globalModel;
    DepthCorrectionModel::LocalModel localModel;

    if (!readCorrectionModel<DepthCorrectionModel::GlobalModel>(src, imageSize, globalModel)
            || !readCorrectionModel<DepthCorrectionModel::LocalModel>(src, imageSize, localModel))
        return false;

    dest.setGlobalModel(std::move(globalModel));
    dest.setLocalModel(std::move(localModel));

	std::cout << "Objects::readDepthCorrection():\n"
              << "  dimensions: " << imageSize.x() << 'x' << imageSize.y() << '\n'
			  << "  global model:\n"
			  << "    " << dest.globalModel().polynomial(0, 0) << '\n'
			  << "    " << dest.globalModel().polynomial(1, 0) << '\n'
			  << "    " << dest.globalModel().polynomial(0, 1) << '\n'
			  << "    " << dest.globalModel().polynomial(1, 1) << '\n'
			  << "  local model: bin size is " << localModel.binSize().x() << ' ' << localModel.binSize().y() << '\n'
			  << std::flush;

	return true;
}

bool ITMLib::Objects::readDepthCorrection(const char *fileName, DepthCorrectionModel &dest)
{
	std::ifstream f(fileName);

	return readDepthCorrection(f, dest);
}

bool ITMLib::Objects::readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, const char *camInTrackerFile, const char *depthCorrectionFile, ITMRGBDCalib & dest)
{
    // Display.
    std::cout << "------------------------------------------------------------------------" << std::endl;

	bool ret = true;
	ret &= ITMLib::Objects::readIntrinsics(rgbIntrinsicsFile, dest.intrinsics_rgb);
	ret &= ITMLib::Objects::readIntrinsics(depthIntrinsicsFile, dest.intrinsics_d);
	ret &= ITMLib::Objects::readExtrinsics(extrinsicsFile, dest.trafo_rgb_to_depth);
	ret &= ITMLib::Objects::readDisparityCalib(disparityCalibFile, dest.disparityCalib);
    ret &= ITMLib::Objects::readExtrinsics(camInTrackerFile, dest.m_h_cam_beacon);
    ret &= ITMLib::Objects::readDepthCorrection(depthCorrectionFile, dest.depth_correction);
	return ret;
}

