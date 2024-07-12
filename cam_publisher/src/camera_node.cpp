#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera_functions.h"

// ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"
#include "std_msgs/Header.h"

sensor_msgs::ImagePtr convertCvMatToRosImage(const cv::Mat& image, const std_msgs::Header& header) {
    /*
    @brief: Convert OpenCV image to ROS Image message
    @param: OpenCV image, ROS header
    @return: ROS Image message
    */
    sensor_msgs::ImagePtr ros_image(new sensor_msgs::Image);
    ros_image->header = header;
    ros_image->height = image.rows;
    ros_image->width = image.cols;
    ros_image->encoding = "bgr8";
    ros_image->is_bigendian = false;
    ros_image->step = image.step;
    size_t size = image.step * image.rows;
    ros_image->data.resize(size);
    memcpy(&ros_image->data[0], image.data, size);
    return ros_image;
}

cv::Size get_calib_params(const std::string& yaml_file, sensor_msgs::CameraInfoPtr cam_info) {
    /*
    @brief: Get calibration parameters for the camera and return the resolution
    @param: YAML file path, CameraInfo message
    @return: Camera resolution as cv::Size
    */
    cv::Size resolution;
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);

        if (config["image_width"] && config["image_height"]) {
            int width = config["image_width"].as<int>();
            int height = config["image_height"].as<int>();
            resolution = cv::Size(width, height);
        } else {
            ROS_ERROR("Failed to read image width and height from YAML");
            return resolution; // Return empty resolution if failed
        }

        if (config["camera_matrix"] && config["camera_matrix"]["data"]) {
            std::vector<double> K = config["camera_matrix"]["data"].as<std::vector<double>>();
            if (K.size() == 9) {
                for (int i = 0; i < 9; i++) {
                    cam_info->K[i] = K[i];
                }
            } else {
                ROS_ERROR("Invalid camera_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read camera_matrix from YAML");
        }

        if (config["distortion_model"]) {
            cam_info->distortion_model = config["distortion_model"].as<std::string>();
        } else {
            ROS_ERROR("Failed to read distortion_model from YAML");
        }

        if (config["distortion_coefficients"] && config["distortion_coefficients"]["data"]) {
            std::vector<double> D = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            cam_info->D.resize(D.size());
            if (D.size() > 0) {
                for (size_t i = 0; i < D.size(); i++) {
                    cam_info->D[i] = D[i];
                }
            } else {
                ROS_ERROR("Invalid distortion_coefficients size");
            }
        } else {
            ROS_ERROR("Failed to read distortion_coefficients from YAML");
        }

        if (config["rectification_matrix"] && config["rectification_matrix"]["data"]) {
            std::vector<double> R = config["rectification_matrix"]["data"].as<std::vector<double>>();
            if (R.size() == 9) {
                for (int i = 0; i < 9; i++) {
                    cam_info->R[i] = R[i];
                }
            } else {
                ROS_ERROR("Invalid rectification_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read rectification_matrix from YAML");
        }

        if (config["projection_matrix"] && config["projection_matrix"]["data"]) {
            std::vector<double> P = config["projection_matrix"]["data"].as<std::vector<double>>();
            if (P.size() == 12) {
                for (int i = 0; i < 12; i++) {
                    cam_info->P[i] = P[i];
                }
            } else {
                ROS_ERROR("Invalid projection_matrix size");
            }
        } else {
            ROS_ERROR("Failed to read projection_matrix from YAML");
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML Exception: %s", e.what());
    }

    return resolution;
}

void image_publisher(cv::Mat& cam_image, image_transport::CameraPublisher& cam_pub, const std::string& topic_name, std_msgs::Header& image_header, sensor_msgs::CameraInfoPtr& cam_info) {
    /*
    @brief: Publish image to ROS topic
    @param: Camera image, Camera publisher, Topic name, Image header, Image message, CameraInfo message
    */
    if (!cam_image.empty()) {
        std::string frame_id = topic_name + "_frame";
        image_header.frame_id = frame_id;

        // Undistorting image
        cv::Mat undistorted_image;
        cv::Mat K = cv::Mat(3, 3, CV_64F, cam_info->K.data());
        cv::Mat D = cv::Mat(cam_info->D.size(), 1, CV_64F, cam_info->D.data());

        cam_info->height = cam_image.rows;
        cam_info->width = cam_image.cols;
        cam_info->header = image_header;

        // Adjust camera info
        cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, cam_image.size(), 0);
        // Update the projection matrix
        sensor_msgs::CameraInfoPtr new_cam_info(new sensor_msgs::CameraInfo());
        new_cam_info->height = cam_image.rows;
        new_cam_info->width = cam_image.cols;
        new_cam_info->header = image_header;
        new_cam_info->distortion_model = cam_info->distortion_model;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                new_cam_info->K[i * 3 + j] = new_K.at<double>(i, j);
                new_cam_info->P[i * 4 + j] = new_K.at<double>(i, j);
                new_cam_info->R[i * 3 + j] = (i == j) ? 1.0 : 0.0;
            }
        }
        new_cam_info->P[3] = 0.0;
        new_cam_info->P[7] = 0.0;
        new_cam_info->P[11] = 1.0;
        new_cam_info->D.resize(5, 0.0);

        cv::undistort(cam_image, undistorted_image, K, D, new_K);
        sensor_msgs::ImagePtr cam_msg = convertCvMatToRosImage(undistorted_image, image_header);

        // Publish via image_transport
        cam_pub.publish(cam_msg, new_cam_info);
    } else {
        ROS_WARN("Captured empty frame, skipping...");
    }
}

int main(int argc, char **argv) {
    // Start ROS stuff
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    std::string yaml_path;
    n.getParam("camera_info_yaml", yaml_path);

    int deviceNode;
    n.getParam("device", deviceNode);
    int fps = 30; ///< default camera fps: 30

    bool invert_image;
    n.param("invert_image", invert_image, false);

    // Get the topic name as a parameter
    std::string topic_name;
    n.param<std::string>("topic_name", topic_name, "camera");

    if (deviceNode < 0) {
        ROS_ERROR("Invalid camera device node: %d", deviceNode);
        return 1;
    }

    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
    cv::Size frameSize = get_calib_params(yaml_path, cam_info);

    if (frameSize.width == 0 || frameSize.height == 0) {
        ROS_ERROR("Failed to get valid resolution from YAML");
        return 1;
    }

    Camera cam(nh, deviceNode, cv::CAP_V4L2);
    
    if (!cam.isOpened()) {
        ROS_ERROR("Failed to open the camera");
        return 1;
    }

    cam.setRawFrameSize(frameSize);
    cam.setRawFrameRate(fps);

    cam.startCapture();
    usleep(500000);

    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher cam_pub = it.advertiseCamera(topic_name + "/image_rect_color", 1);

    std_msgs::Header image_header;
    sensor_msgs::ImagePtr cam_msg;

    ros::Rate loop_rate(fps);
    while (ros::ok() && cam.isOpened()) {
        cv::Mat cam_image;
        cam.cap >> cam_image;

        if (cam_image.empty()) {
            ROS_WARN("Captured empty frame");
            continue;
        }

        if (invert_image) {
            cv::flip(cam_image, cam_image, -1);
        }

        image_header.stamp = ros::Time::now();
        image_publisher(cam_image, cam_pub, topic_name, image_header, cam_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cam.stopCapture();

    return 0;
}
