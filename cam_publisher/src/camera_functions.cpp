#include "camera_functions.h"

Camera::Camera(ros::NodeHandle& nh, int deviceNode, int backend) 
{
    if (!cap.open(deviceNode, backend)) {
        throw std::runtime_error("Failed to open camera");
    }
}

Camera::~Camera()
{
    cap.release();
}

bool Camera::isOpened() const
{
    return cap.isOpened();
}

void Camera::startCapture() {
    bool success = true;

    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, rawFrameSize.width)) {
        success = false;
    }

    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, rawFrameSize.height)) {
        success = false;
    }

    if (!cap.set(cv::CAP_PROP_FPS, rawFrameRate)) {
        success = false;
    }

    if (!success) {
        throw std::runtime_error("Failed to set camera properties");
    }
}

void Camera::stopCapture()
{
    cap.release();
}

void Camera::setRawFrameSize(cv::Size frameSize)
{
    rawFrameSize = frameSize;
}

void Camera::setRawFrameRate(int fps)
{
    rawFrameRate = fps;
}
