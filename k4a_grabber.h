#pragma once
#include <k4a/k4a.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZI;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;

	class KinectAzureDKGrabber : public pcl::Grabber
	{
	public:
		KinectAzureDKGrabber(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_);
		virtual ~KinectAzureDKGrabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		using signal_KinectAzureDK_PointXYZ = void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
		using signal_KinectAzureDK_PointXYZI = void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&);
		using signal_KinectAzureDK_PointXYZRGB = void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);
		using signal_KinectAzureDK_PointXYZRGBA = void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);
	
	protected:
		void setupDevice(const int &device_id_, const int &depth_mode_, const int &color_format_, const int &color_resolution_);
		
		boost::signals2::signal<signal_KinectAzureDK_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_KinectAzureDK_PointXYZI>* signal_PointXYZI;
		boost::signals2::signal<signal_KinectAzureDK_PointXYZRGB>* signal_PointXYZRGB;
		boost::signals2::signal<signal_KinectAzureDK_PointXYZRGBA>* signal_PointXYZRGBA;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ();
		pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA();

		std::thread thread;
		mutable std::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		k4a_device_configuration_t config;
		k4a::device dev;
		int device_id;

		k4a::calibration calibration;
		k4a::transformation transformation;

		int colorWidth;
		int colorHeight;
		k4a::image colorImage;

		int depthWidth;
		int depthHeight;
		k4a::image depthImage;

		int infraredWidth;
		int infraredHeight;
		k4a::image infraredImage;
	public:

		k4a::calibration getCalibration();
	};
}