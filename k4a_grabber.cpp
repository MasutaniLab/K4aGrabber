#include <Eigen/Dense>

#include "k4a_grabber.h"

pcl::KinectAzureDKGrabber::KinectAzureDKGrabber(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_, bool alignToDepth_) :
	config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
	dev(nullptr),
	colorImage(nullptr),
	depthImage(nullptr),
	infraredImage(nullptr),
	running(false),
	quit(false),
	signal_PointXYZ(nullptr),
	signal_PointXYZI(nullptr),
	signal_PointXYZRGB(nullptr),
	signal_PointXYZRGBA(nullptr)
{
	setupDevice(device_id_, depth_mode_, color_format_, color_resolution_, alignToDepth_);

	signal_PointXYZ = createSignal<signal_KinectAzureDK_PointXYZ>();
	signal_PointXYZI = createSignal<signal_KinectAzureDK_PointXYZI>();
	signal_PointXYZRGB = createSignal<signal_KinectAzureDK_PointXYZRGB>();
	signal_PointXYZRGBA = createSignal<signal_KinectAzureDK_PointXYZRGBA>();
}

pcl::KinectAzureDKGrabber::~KinectAzureDKGrabber() throw()
{
	stop();

	disconnect_all_slots<signal_KinectAzureDK_PointXYZ>();
	disconnect_all_slots<signal_KinectAzureDK_PointXYZI>();
	disconnect_all_slots<signal_KinectAzureDK_PointXYZRGB>();
	disconnect_all_slots<signal_KinectAzureDK_PointXYZRGBA>();

	thread.join();

	if (dev)
	{
		transformation.destroy();
		dev.close();
	}
}

void pcl::KinectAzureDKGrabber::start()
{
	dev = k4a::device::open(device_id);
	dev.start_cameras(&config);
	calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
	transformation = k4a::transformation(calibration);

	running = true;

	thread = std::thread(&KinectAzureDKGrabber::threadFunction, this);
}
k4a::calibration pcl::KinectAzureDKGrabber::getCalibration()
{
	return calibration;
}
void pcl::KinectAzureDKGrabber::stop()
{
	std::unique_lock<std::mutex> lock(mutex);

	quit = true;
	running = false;

	lock.unlock();
}

bool pcl::KinectAzureDKGrabber::isRunning() const
{
	std::unique_lock<std::mutex> lock(mutex);

	return running;

	lock.unlock();
}

std::string pcl::KinectAzureDKGrabber::getName() const
{
	return std::string("KinectAzureDKGrabber: " + std::to_string(device_id));
}

float pcl::KinectAzureDKGrabber::getFramesPerSecond() const
{
	return config.camera_fps;
}

void pcl::KinectAzureDKGrabber::setupDevice(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_, bool alignToDepth_)
{
	device_id = device_id_;

	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = k4a_depth_mode_t(depth_mode_);
	config.color_format = k4a_image_format_t(color_format_);
	config.color_resolution = k4a_color_resolution_t(color_resolution_);
	config.synchronized_images_only = true;
	alignToDepth = alignToDepth_;
}
void pcl::KinectAzureDKGrabber::threadFunction()
{
	while (!quit)
	{
		std::unique_lock<std::mutex> lock(mutex);
		k4a::capture capture;
		if (!dev.get_capture(&capture, std::chrono::milliseconds(0)))
		{
			continue;
		}

		depthImage = capture.get_depth_image();
		if (depthImage == nullptr)
		{
			throw std::exception("Failed to get depth image from capture\n");
		}

		colorImage = capture.get_color_image();
		if (colorImage == nullptr)
		{
			throw std::exception("Failed to get color image from capture\n");
		}

		infraredImage = capture.get_ir_image();
		if (infraredImage == nullptr)
		{
			throw std::exception("Failed to get IR image from capture\n");
		}

		lock.unlock();

		if (signal_PointXYZ->num_slots() > 0)
		{
			signal_PointXYZ->operator()(convertDepthToPointXYZ());
		}

		if (signal_PointXYZI->num_slots() > 0)
		{
			signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI());
		}

		if (signal_PointXYZRGB->num_slots() > 0)
		{
			signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB());
		}

		if (signal_PointXYZRGBA->num_slots() > 0)
		{
			signal_PointXYZRGBA->operator()(convertRGBADepthToPointXYZRGBA());
		}
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::KinectAzureDKGrabber::convertDepthToPointXYZ()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());

	int width, height;
	k4a::image newDepthImage = nullptr;
	k4a::image newColorImage = nullptr;
	if (alignToDepth) {
		width = depthImage.get_width_pixels();
		height = depthImage.get_height_pixels();
		newDepthImage = depthImage;
		newColorImage = k4a::image::create(
			K4A_IMAGE_FORMAT_COLOR_BGRA32,
			width, height, width * 4 * (int)sizeof(uint8_t));
	} else {
		width = colorImage.get_width_pixels();
		height = colorImage.get_height_pixels();
		newDepthImage = k4a::image::create(
			K4A_IMAGE_FORMAT_DEPTH16,
			width, height, width * (int)sizeof(uint16_t));
		newColorImage = colorImage;
	}

	k4a::image pointCloudImage = nullptr;
	pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		width, height, width * 3 * (int)sizeof(int16_t));

	if (alignToDepth) {
		transformation.color_image_to_depth_camera(
			depthImage, colorImage, &newColorImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage);
	} else {
		transformation.depth_image_to_color_camera(
			depthImage, &newDepthImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage);
	}
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	int16_t* point_cloud_image_data = (int16_t*)(void*)pointCloudImage.get_buffer();

#ifdef VTK_VISUALIZATION
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
#endif

	for (int i = 0; i < width * height; ++i)
	{
		PointXYZ point;

		point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
		point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
		point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

		if (point.z == 0)
		{
			continue;
		}

#ifdef VTK_VISUALIZATION
		point.getVector3fMap() = m * point.getVector3fMap();
#endif

		cloud->points[i] = point;
	}
	return cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl::KinectAzureDKGrabber::convertInfraredDepthToPointXYZI()
{
	PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>());
	int width, height;
	k4a::image newDepthImage = nullptr;
	k4a::image newColorImage = nullptr;
	if (alignToDepth) {
		width = depthImage.get_width_pixels();
		height = depthImage.get_height_pixels();
		newDepthImage = depthImage;
		newColorImage = k4a::image::create(
			K4A_IMAGE_FORMAT_COLOR_BGRA32,
			width, height, width * 4 * (int)sizeof(uint8_t));
	} else {
		width = colorImage.get_width_pixels();
		height = colorImage.get_height_pixels();
		newDepthImage = k4a::image::create(
			K4A_IMAGE_FORMAT_DEPTH16,
			width, height, width * (int)sizeof(uint16_t));
		newColorImage = colorImage;
	}

	//depth_image_to_color_camera_custom()のためにK4A_IMAGE_FORMAT_IR16ではなく，
	//K4A_IMAGE_FORMAT_CUSTOM16にする
	k4a::image newInfraredImage = nullptr;
	newInfraredImage = k4a::image::create(
		K4A_IMAGE_FORMAT_CUSTOM16,
		width, height, width * (int)sizeof(uint16_t));

	k4a::image pointCloudImage = nullptr;
	pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		width, height, width * 3 * (int)sizeof(int16_t));

	if (alignToDepth) {
		transformation.color_image_to_depth_camera(
			depthImage, colorImage, &newColorImage);
		//赤外線画像を深度カメラに変換する方法が不明
		//transformation.color_image_to_depth_camera(
		//	depthImage, infraredImage, &newInfraredImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage);
	} else {
		//depth_image_to_color_camera_custom()がK4A_IMAGE_FORMAT_CUSTOM16にしか対応していないので，それに変換．
		k4a::image infraredImage2 = nullptr;
		infraredImage2 = k4a::image::create_from_buffer(
			K4A_IMAGE_FORMAT_CUSTOM16,
			infraredImage.get_width_pixels(), 
			infraredImage.get_height_pixels(),
			infraredImage.get_width_pixels() * (int)sizeof(uint16_t),
			infraredImage.get_buffer(),
			infraredImage.get_width_pixels() * infraredImage.get_height_pixels() * (int)sizeof(uint16_t),
			nullptr,
			nullptr
			);	

		transformation.depth_image_to_color_camera_custom(
			depthImage, infraredImage2, &newDepthImage, &newInfraredImage, 
			K4A_TRANSFORMATION_INTERPOLATION_TYPE_LINEAR, width * sizeof(uint16_t));
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage);
	}
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	int16_t* point_cloud_image_data = (int16_t*)(void*)pointCloudImage.get_buffer();
	int16_t* transformed_infrared_image_data = (int16_t*)(void*)newInfraredImage.get_buffer();

#ifdef VTK_VISUALIZATION
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
#endif

	for (int i = 0; i < width * height; ++i)
	{
		PointXYZI point;

		point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
		point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
		point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

		if (point.z == 0)
		{
			continue;
		}

		if (alignToDepth) {
			//赤外線画像の変換ができていないので仮の値
			point.intensity = 0xffff;
		} else {
			point.intensity = transformed_infrared_image_data[i];
		}
#ifdef VTK_VISUALIZATION
		point.getVector3fMap() = m * point.getVector3fMap();
#endif

		cloud->points[i] = point;
	}
	return cloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::KinectAzureDKGrabber::convertRGBDepthToPointXYZRGB()
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());

	int width, height;
	k4a::image newDepthImage = nullptr;
	k4a::image newColorImage = nullptr;
	if (alignToDepth) {
		width = depthImage.get_width_pixels();
		height = depthImage.get_height_pixels();
		newDepthImage = depthImage;
		newColorImage = k4a::image::create(
			K4A_IMAGE_FORMAT_COLOR_BGRA32,
			width, height, width * 4 * (int)sizeof(uint8_t));
	} else {
		width = colorImage.get_width_pixels();
		height = colorImage.get_height_pixels();
		newDepthImage = k4a::image::create(
			K4A_IMAGE_FORMAT_DEPTH16,
			width, height, width * (int)sizeof(uint16_t));
		newColorImage = colorImage;
	}

	k4a::image pointCloudImage = nullptr;
	pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		width, height, width * 3 * (int)sizeof(int16_t));

	if (alignToDepth) {
		transformation.color_image_to_depth_camera(
			depthImage, colorImage, &newColorImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage);
	} else {
		transformation.depth_image_to_color_camera(
			depthImage, &newDepthImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage);
	}
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	int16_t* point_cloud_image_data = (int16_t*)(void*)pointCloudImage.get_buffer();
	uint8_t* color_image_data = newColorImage.get_buffer();

#ifdef VTK_VISUALIZATION
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
#endif

	for (int i = 0; i < width * height; ++i)
	{
		PointXYZRGB point;

		point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
		point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
		point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;
		if (point.z == 0)
		{
			continue;
		}

#ifdef VTK_VISUALIZATION
		point.getVector3fMap() = m * point.getVector3fMap();
#endif

		point.b = color_image_data[4 * i + 0];
		point.g = color_image_data[4 * i + 1];
		point.r = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];
		if (point.b == 0 && point.g == 0 && point.r == 0 && alpha == 0)
		{
			continue;
		}
		cloud->points[i] = point;
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::KinectAzureDKGrabber::convertRGBADepthToPointXYZRGBA(/*RGBQUAD* colorBuffer, UINT16* depthBuffer*/)
{
	PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());

	int width, height;
	k4a::image newDepthImage = nullptr;
	k4a::image newColorImage = nullptr;
	if (alignToDepth) {
		width = depthImage.get_width_pixels();
		height = depthImage.get_height_pixels();
		newDepthImage = depthImage;
		newColorImage = k4a::image::create(
			K4A_IMAGE_FORMAT_COLOR_BGRA32,
			width, height, width * 4 * (int)sizeof(uint8_t));
	} else {
		width = colorImage.get_width_pixels();
		height = colorImage.get_height_pixels();
		newDepthImage = k4a::image::create(
			K4A_IMAGE_FORMAT_DEPTH16,
			width, height, width * (int)sizeof(uint16_t));
		newColorImage = colorImage;
	}

	k4a::image pointCloudImage = nullptr;
	pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
		width, height, width * 3 * (int)sizeof(int16_t));

	if (alignToDepth) {
		transformation.color_image_to_depth_camera(
			depthImage, colorImage, &newColorImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage);
	} else {
		transformation.depth_image_to_color_camera(
			depthImage, &newDepthImage);
		transformation.depth_image_to_point_cloud(
			newDepthImage, K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage);
	}
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	int16_t* point_cloud_image_data = (int16_t*)(void*)pointCloudImage.get_buffer();
	uint8_t* color_image_data = newColorImage.get_buffer();

#ifdef VTK_VISUALIZATION
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ());
#endif

	for (int i = 0; i < width * height; ++i)
	{
		PointXYZRGBA point;

		point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
		point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
		point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

		if (point.z == 0)
		{
			continue;
		}


#ifdef VTK_VISUALIZATION
		point.getVector3fMap() = m * point.getVector3fMap();
#endif

		point.b = color_image_data[4 * i + 0];
		point.g = color_image_data[4 * i + 1];
		point.r = color_image_data[4 * i + 2];
		point.a = color_image_data[4 * i + 3];

		if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
		{
			continue;
		}

		cloud->points[i] = point;
	}
	return cloud;
}