#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

namespace arti {


class StereoCamera
{

public:

	/**
	 * @brief      { stereo camera driver }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
	StereoCamera(int resolution, double frame_rate): frame_rate_(30.0) {

		camera_ = new cv::VideoCapture(0);
		cv::Mat raw;
		cv::Mat left_image;
		cv::Mat right_image;
		setResolution(resolution);
		setFrameRate(frame_rate);

		std::cout << "Stereo Camera Set Resolution: " << camera_->get(WIDTH_ID) << "x" << camera_->get(HEIGHT_ID) << std::endl;
		std::cout << "Stereo Camera Set Frame Rate: " << camera_->get(FPS_ID) << std::endl;
	}

	~StereoCamera() {
		// std::cout << "Destroy the pointer" << std::endl;
		delete camera_;
	}

	/**
	 * @brief      Sets the resolution.
	 *
	 * @param[in]  type  The type
	 */
	void setResolution(int type) {

		if (type == 0) { width_ = 4416; height_ = 1242;} // 2K
		if (type == 1) { width_ = 3840; height_ = 1080;} // FHD
		if (type == 2) { width_ = 2560; height_ = 720;}  // HD
		if (type == 3) { width_ = 1344; height_ = 376;}  // VGA

		camera_->set(WIDTH_ID, width_);
		camera_->set(HEIGHT_ID, height_);

		// make sure that the number set are right from the hardware
		width_ = camera_->get(WIDTH_ID);
		height_ = camera_->get(HEIGHT_ID);

	}

	/**
	 * @brief      Sets the frame rate.
	 *
	 * @param[in]  frame_rate  The frame rate
	 */
	void setFrameRate(double frame_rate) {
		camera_->set(FPS_ID, frame_rate);
		frame_rate_ = camera_->get(FPS_ID);
	}

	/**
	 * @brief      Gets the images.
	 *
	 * @param      left_image   The left image
	 * @param      right_image  The right image
	 *
	 * @return     The images.
	 */
	bool getImages(cv::Mat& left_image, cv::Mat& right_image) {
		cv::Mat raw;
		if (camera_->grab()) {
			camera_->retrieve(raw);
			cv::Rect left_rect(0, 0, width_ / 2, height_);
			cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
			left_image = raw(left_rect);
			right_image = raw(right_rect);
			cv::waitKey(10);
			return true;
		} else {
			return false;
		}
	}

private:
	cv::VideoCapture* camera_;
	int width_;
	int height_;
	double frame_rate_;
	bool cv_three_;
};

/**
 * @brief       the camera ros warpper class
 */
class ZedCameraROS {
public:

	/**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
	 */
	ZedCameraROS() {
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
		// get ros param
		private_nh.param("resolution", resolution_, 1);
		private_nh.param("frame_rate", frame_rate_, 30.0);
		private_nh.param("config_file_location", config_file_location_, std::string("~/SN1267.conf"));
		private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
		private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
		private_nh.param("show_image", show_image_, false);

		camera_info_manager::CameraInfoManager info_manager(nh);
		info_manager.setCameraName("zed/left");
		info_manager.loadCameraInfo( "package://zed_cpu_ros/config/left.yaml");
		sensor_msgs::CameraInfo left_info = info_manager.getCameraInfo();

		info_manager.setCameraName("zed/right");
		info_manager.loadCameraInfo( "package://zed_cpu_ros/config/right.yaml");
		sensor_msgs::CameraInfo right_info = info_manager.getCameraInfo();

		// camera_info_manager::CameraInfoManager left_info_manager(nh, "zed/left", "package://zed_cpu_ros/config/left.yaml");
		// camera_info_manager::CameraInfoManager right_info_manager(nh, "zed/right", "package://zed_cpu_ros/config/right.yaml");

		// sensor_msgs::CameraInfo left_info = left_info_manager.getCameraInfo();
		// sensor_msgs::CameraInfo right_info = right_info_manager.getCameraInfo();

		std::cout << left_info << std::endl;
		std::cout << right_info << std::endl;

		// ROS_INFO("Try to initialize the camera");
		// // initialize camera
		// StereoCamera zed(resolution_, frame_rate_);
		// ROS_INFO("Initialized the camera");

		// // set up empty message pointer
		// sensor_msgs::CameraInfoPtr left_cam_info_msg_ptr(new sensor_msgs::CameraInfo());
		// sensor_msgs::CameraInfoPtr right_cam_info_msg_ptr(new sensor_msgs::CameraInfo());

		// // setup publisher stuff
		// image_transport::ImageTransport it(nh);
		// image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
		// image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

		// ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
		// ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

		// ROS_INFO("Try load camera calibration files");

		// //get camera info
		// try {
		// 	getCameraInfo(config_file_location_, resolution_, left_cam_info_msg_ptr, right_cam_info_msg_ptr);
		// }
		// catch (std::runtime_error& e) {
		// 	ROS_INFO("Can't load camera info");
		// 	throw e;
		// }

		// // ROS_INFO("Left Camera Info as following");
		// // std::cout << *left_cam_info_msg_ptr << std::endl;
		// // ROS_INFO("Right Camera Info as following");
		// // std::cout << *right_cam_info_msg_ptr << std::endl;

		// ROS_INFO("Got camera calibration files");

		// // loop to publish images;
		// cv::Mat left_image, right_image;
		// while (nh.ok()) {
		// 	ros::Time now = ros::Time::now();
		// 	if (!zed.getImages(left_image, right_image)) {
		// 		ROS_INFO_ONCE("Can't find camera");
		// 	} else {
		// 		ROS_INFO_ONCE("Success, found camera");
		// 	}
		// 	if (show_image_) {
		// 		cv::imshow("left", left_image);
		// 		cv::imshow("right", right_image);
		// 	}

		// 	if (left_image_pub.getNumSubscribers() > 0) {
		// 		publishImage(left_image, left_image_pub, "left_frame", now);
		// 	}
		// 	if (right_image_pub.getNumSubscribers() > 0) {
		// 		publishImage(right_image, right_image_pub, "right_frame", now);
		// 	}
		// 	if (left_cam_info_pub.getNumSubscribers() > 0) {
		// 		publishCamInfo(left_cam_info_pub, left_cam_info_msg_ptr, now);
		// 	}
		// 	if (right_cam_info_pub.getNumSubscribers() > 0) {
		// 		publishCamInfo(right_cam_info_pub, right_cam_info_msg_ptr, now);
		// 	}
		// 	// since the frame rate was set inside the camera, no need to do a ros sleep
		// }
	}

	/**
	 * @brief      Gets the camera information.
	 *
	 * @param[in]  config_file         The configuration file
	 * @param[in]  resolution          The resolution
	 * @param[in]  left_cam_info_msg   The left camera information message
	 * @param[in]  right_cam_info_msg  The right camera information message
	 */
	void getCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg) {
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(config_file, pt);
		std::string left_str = "LEFT_CAM_";
		std::string right_str = "RIGHT_CAM_";
		std::string reso_str = "";

		switch (resolution) {
		case 0: reso_str = "2K";
		case 1: reso_str = "FHD";
		case 2: reso_str = "HD";
		case 3: reso_str = "VGA";
		}
		// left value
		double l_cx = pt.get<double>(left_str + reso_str + ".cx");
		double l_cy = pt.get<double>(left_str + reso_str + ".cy");
		double l_fx = pt.get<double>(left_str + reso_str + ".fx");
		double l_fy = pt.get<double>(left_str + reso_str + ".fy");
		double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
		double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
		// right value
		double r_cx = pt.get<double>(right_str + reso_str + ".cx");
		double r_cy = pt.get<double>(right_str + reso_str + ".cy");
		double r_fx = pt.get<double>(right_str + reso_str + ".fx");
		double r_fy = pt.get<double>(right_str + reso_str + ".fy");
		double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
		double r_k2 = pt.get<double>(right_str + reso_str + ".k2");
		// conver mm to m
		double baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
		// assume zeros, maybe not right
		double p1 = 0, p2 = 0, k3 = 0;

		left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		left_cam_info_msg->D.resize(5);
		left_cam_info_msg->D[0] = l_k1;
		left_cam_info_msg->D[1] = l_k2;
		left_cam_info_msg->D[2] = k3;
		left_cam_info_msg->D[3] = p1;
		left_cam_info_msg->D[4] = p2;

		right_cam_info_msg->D.resize(5);
		right_cam_info_msg->D[0] = r_k1;
		right_cam_info_msg->D[1] = r_k2;
		right_cam_info_msg->D[2] = k3;
		right_cam_info_msg->D[3] = p1;
		right_cam_info_msg->D[4] = p2;

		left_cam_info_msg->K.fill(0.0);
		left_cam_info_msg->K[0] = l_fx;
		left_cam_info_msg->K[2] = l_cx;
		left_cam_info_msg->K[4] = l_fy;
		left_cam_info_msg->K[5] = l_cy;
		left_cam_info_msg->K[8] = 1.0;

		right_cam_info_msg->K.fill(0.0);
		right_cam_info_msg->K[0] = r_fx;
		right_cam_info_msg->K[2] = r_cx;
		right_cam_info_msg->K[4] = r_fy;
		right_cam_info_msg->K[5] = r_cy;
		right_cam_info_msg->K[8] = 1.0;

		left_cam_info_msg->R.fill(0.0);
		right_cam_info_msg->R.fill(0.0);

		left_cam_info_msg->P.fill(0.0);
		left_cam_info_msg->P[0] = l_fx;
		left_cam_info_msg->P[2] = l_cx;
		left_cam_info_msg->P[5] = l_fy;
		left_cam_info_msg->P[6] = l_cy;
		left_cam_info_msg->P[10] = 1.0;

		right_cam_info_msg->P.fill(0.0);
		right_cam_info_msg->P[0] = r_fx;
		right_cam_info_msg->P[2] = r_cx;
		right_cam_info_msg->P[5] = r_fy;
		right_cam_info_msg->P[6] = r_cy;
		right_cam_info_msg->P[10] = 1.0;
		right_cam_info_msg->P[3] = (-1 * l_fx * baseline);

		left_cam_info_msg->width = right_cam_info_msg->width = width_;
		left_cam_info_msg->height = right_cam_info_msg->height = height_;

		left_cam_info_msg->header.frame_id = left_frame_id_;
		right_cam_info_msg->header.frame_id = right_frame_id_;
	}

	/**
	 * @brief      { publish cameara info }
	 *
	 * @param[in]  pub_cam_info  The pub camera information
	 * @param[in]  cam_info_msg  The camera information message
	 * @param[in]  now           The now
	 */
	void publishCamInfo(ros::Publisher pub_cam_info, sensor_msgs::CameraInfoPtr cam_info_msg, ros::Time now) {
		cam_info_msg->header.stamp = now;
		pub_cam_info.publish(cam_info_msg);
	}

	/**
	 * @brief      { publish image }
	 *
	 * @param[in]  img           The image
	 * @param      img_pub       The image pub
	 * @param[in]  img_frame_id  The image frame identifier
	 * @param[in]  t             { parameter_description }
	 */
	void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
		cv_bridge::CvImage cv_image;
		cv_image.image = img;
		cv_image.encoding = sensor_msgs::image_encodings::BGR8;
		cv_image.header.frame_id = img_frame_id;
		cv_image.header.stamp = t;
		img_pub.publish(cv_image.toImageMsg());
	}

private:
	int resolution_;
	double frame_rate_;
	bool show_image_;
	double width_, height_;
	std::string left_frame_id_, right_frame_id_;
	std::string config_file_location_;

};

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "zed_camera");
	arti::ZedCameraROS zed_ros;
	return 0;
}