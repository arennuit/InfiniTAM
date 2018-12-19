// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>

#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"

#include "Engine/OpenNIEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/RealSenseEngine.h"

#include "Engine/MocapFileEngine.h"
#include "Engine/MocapViveEngine.h"

using namespace InfiniTAM::Engine;

////////////////////////////////////////////////////////////////////////////////
/// Create a default source of depth images from the command line arguments.
static void CreateDefaultImageSource(ImageSourceEngine*& imageSource, IMUSourceEngine*& imuSource, MocapSourceEngine*& mocapSource, int & argc, char** argv)
{
    // Parse arguments.
    uint isCalib     = 0;
    uint isFiles     = 0;
    uint isMocap     = 0;
    uint isImu       = 0;
    uint isOpenNi    = 0;
    uint isUvc       = 0;
    uint isRealSense = 0;
    uint isKinect2   = 0;

    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--calib")
            isCalib = i;

        if (std::string(argv[i]) == "--files")
            isFiles = i;

        if (std::string(argv[i]) == "--mocap")
            isMocap = i;

        if (std::string(argv[i]) == "--imu")
            isImu = i;

        if (std::string(argv[i]) == "--openni")
            isOpenNi = i;

        if (std::string(argv[i]) == "--uvc")
            isUvc = i;

        if (std::string(argv[i]) == "--realsense")
            isRealSense = i;

        if (std::string(argv[i]) == "--kinect2")
            isKinect2 = i;
    }

    // Calibration.
    const char *calibFile = argv[isCalib + 1];

    if (isCalib)
    {
        std::cout << "Custom calibration." << std::endl;
        std::cout << "   using calib file: " << calibFile << std::endl;
    }
    else
        std::cout << "Default calibration." << std::endl;

    // Source.
    if (isFiles)
    {
        // Files source.
        const char *rgbFilepath   = argv[isFiles + 1];
        const char *depthFilepath = argv[isFiles + 2];

        std::cout << "Files source." << std::endl;
        std::cout << "   using rgb   files: " << rgbFilepath   << std::endl;
        std::cout << "   using depth files: " << depthFilepath << std::endl;

        if (isMocap)
        {
            // Fusion with mocap.
            const char *mocapFile = argv[isMocap + 1];

            std::cout << "Fusion with mocap" << std::endl;
            std::cout << "   using mocap file : " << mocapFile << std::endl;

            imageSource = new ImageFileReader(calibFile, rgbFilepath, depthFilepath);
            mocapSource = new MocapFileEngine(mocapFile);
        }
        else if(isImu)
        {
            // Fusion with IMU.
            const char *imuFile = argv[isImu + 1];

            std::cout << "Fusion with IMU." << std::endl;
            std::cout << "   using IMU file : " << imuFile << std::endl;

            imageSource = new RawFileReader(calibFile, rgbFilepath, depthFilepath, Vector2i(320, 240), 0.5f);
            imuSource   = new IMUSourceEngine(imuFile);
        }
        else
        {
            // No fusion.
            std::cout << "No fusion used." << std::endl;

            imageSource = new ImageFileReader(calibFile, rgbFilepath, depthFilepath);
        }
    }
    else if (isOpenNi && imageSource == NULL)
	{
        // OpenNI source.
        const char *deviceUri = 0;
        if (isOpenNi != (uint)(argc - 1) && std::string(argv[isOpenNi + 1]).substr(0, 2) != "--")
            deviceUri = argv[isOpenNi + 1];

        std::cout << "OpenNI device source." << std::endl;
        std::cout << "   device URI: " << std::string((deviceUri == 0) ? "<OpenNI default device>" : deviceUri) << std::endl;

        imageSource = new OpenNIEngine(calibFile, deviceUri);

        // Device not found.
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
    else if (isUvc && imageSource == NULL)
	{
        // UVC source.
        std::cout << "UVC device source." << std::endl;

		imageSource = new LibUVCEngine(calibFile);

        // Device not found.
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
    else if (isRealSense && imageSource == NULL)
	{
        // RealSense source.
        std::cout << "RealSence device source." << std::endl;

		imageSource = new RealSenseEngine(calibFile);

        // Device not found.
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
    else if (isKinect2 && imageSource == NULL)
	{
        // Kinect2 source.
        std::cout << "MS Kinect2 device source." << std::endl;

		imageSource = new Kinect2Engine(calibFile);

        // Device not found.
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}

    if (!isFiles && isMocap)
    {
        std::cout << "Vive device motion capture source" << std::endl;

        try
        {
            mocapSource = new MocapViveEngine();
        }
        catch (std::exception &e)
        {
            std::clog << "Impossible to connect mocap: " << e.what() << std::endl;
        }
    }

    // This is a hack to ensure backwards compatibility in certain configurations.
    if (imageSource == NULL)
        return;

	if (imageSource->calib.disparityCalib.params == Vector2f(0.0f, 0.0f))
	{
		imageSource->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
		imageSource->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
	}
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
try
{
    // Check command line.
    if (argc == 1)
    {
        std::cout << std::endl
                  << "usage:" << std::endl
                  << argv[0]
                  << " [--calib <calibFile>]"
                  << " [--files <rgbFile> <depthFile>]"
                  << " [--openni [<device URI>]] [--uvc] [--realsense] [--kinect2]"
                  << " [--imu [<imuFile>]] [--mocap [<mocapFile>]]"
                  << std::endl
                  << std::endl
                  << "examples:\n"
                  << "   1)" << argv[0] << " --calib ./Files/Teddy/calib.txt --files ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm --mocap ./Files/Teddy/mocap.txt" << std::endl
                  << "   2)" << argv[0] << " --calib ./Files/Teddy/calib.txt --openni" << std::endl
                  << "   3)" << argv[0] << " --openni" << std::endl
                  << "   4)" << argv[0] << " --calib ./Files/Teddy/calib.txt --openni --mocap" << std::endl
                  << std::endl;

        return 0;
	}

    // Create source engines (images, imu, mocap source engines).
    std::cout << "--------- Initialising ---------" << std::endl;
    ImageSourceEngine *imageSource = 0;
    IMUSourceEngine   *imuSource   = 0;
    MocapSourceEngine *mocapSource = 0;

    CreateDefaultImageSource(imageSource, imuSource, mocapSource, argc, argv);
    if (imageSource == NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

    // Create main engine.
	ITMLibSettings *internalSettings = new ITMLibSettings();
    if ( mocapSource )
    {
        internalSettings->trackerType = ITMLibSettings::TRACKER_ICP_MOCAP;
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Using TRACKER_ICP_MOCAP" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }
    else
    {
//        internalSettings->trackerType = ITMLibSettings::TRACKER_COLOR;
        internalSettings->trackerType = ITMLibSettings::TRACKER_ICP;
//        internalSettings->trackerType = ITMLibSettings::TRACKER_ICP_MOCAP;
//        internalSettings->trackerType = ITMLibSettings::TRACKER_REN;
//        internalSettings->trackerType = ITMLibSettings::TRACKER_IMU;
//        internalSettings->trackerType = ITMLibSettings::TRACKER_WICP;

        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Using TRACKER_ICP" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }

	ITMMainEngine *mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

    // Init and run loop.
    UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mocapSource, mainEngine, "../Files/Out", internalSettings->deviceType);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

