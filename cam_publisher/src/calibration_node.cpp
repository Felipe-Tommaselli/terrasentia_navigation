#include <cstdio>
#include <chrono>
#include <string>
#include <vector>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"

void image_publisher(cv::Mat cam_image, image_transport::Publisher cam_pub, ros::Publisher cam_info_pub, std::string topic_name, std_msgs::Header image_header, sensor_msgs::CameraInfoPtr cam_info) {
    /*
    @brief: Publish image and camera info to ROS topics
    @param: Camera image, Camera publisher, Camera info publisher, Topic name, Image header, CameraInfo message
    */
    if (!cam_image.empty()) {
        std::string frame_id = topic_name + "_frame";
        image_header.frame_id = frame_id;
        sensor_msgs::ImagePtr cam_msg = cv_bridge::CvImage(image_header, "bgr8", cam_image).toImageMsg();
        cam_pub.publish(cam_msg);

        cam_info->header = image_header;
        cam_info_pub.publish(cam_info);
    }
}

sensor_msgs::CameraInfoPtr create_camera_info(const cv::Size& frameSize) {
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
    cam_info->width = frameSize.width;
    cam_info->height = frameSize.height;
    // Preencher com informações básicas
    cam_info->K = {1.0, 0.0, frameSize.width / 2.0, 0.0, 1.0, frameSize.height / 2.0, 0.0, 0.0, 1.0};
    cam_info->P = {1.0, 0.0, frameSize.width / 2.0, 0.0, 0.0, 1.0, frameSize.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    return cam_info;
}

bool setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res, sensor_msgs::CameraInfoPtr cam_info) {
    *cam_info = req.camera_info;
    res.success = true;
    res.status_message = "Camera info set successfully.";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    int deviceNode;
    n.getParam("device", deviceNode);
    cv::Size frameSize(640, 480); // Default frame size
    int fps = 30; // Default camera fps

    // Get the topic name as a parameter
    std::string topic_name;
    n.param<std::string>("topic_name", topic_name, "camera");

    ROS_INFO("Opening camera device: %d", deviceNode);
    cv::VideoCapture cam(deviceNode, cv::CAP_V4L2);

    if (!cam.isOpened()) {
        ROS_ERROR("Failed to open camera");
        exit(EXIT_FAILURE);
    }

    cam.set(cv::CAP_PROP_FRAME_WIDTH, frameSize.width);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, frameSize.height);
    cam.set(cv::CAP_PROP_FPS, fps);

    usleep(500000); // Wait for camera to initialize

    image_transport::ImageTransport it(nh);
    image_transport::Publisher cam_pub = it.advertise(topic_name + "/image_raw", 1);
    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>(topic_name + "/camera_info", 1);

    std_msgs::Header image_header;
    sensor_msgs::CameraInfoPtr cam_info = create_camera_info(frameSize);

    // Adicione o serviço
    ros::ServiceServer service = nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response>(
        topic_name + "/set_camera_info",
        boost::bind(setCameraInfoCallback, _1, _2, cam_info)
    );

    ros::Rate loop_rate(fps);
    while (ros::ok() && cam.isOpened()) {
        cv::Mat cam_image;
        cam >> cam_image;

        cv::flip(cam_image, cam_image, -1);

        image_header.stamp = ros::Time::now();
        image_publisher(cam_image, cam_pub, cam_info_pub, topic_name, image_header, cam_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cam.release(); // Stop camera capturing

    return 0;
}
