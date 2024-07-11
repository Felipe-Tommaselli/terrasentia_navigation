#include <opencv2/opencv.hpp>
#include <ros/ros.h>

/*
@class Camera
@brief Class to handle camera operations
*/
class Camera {
public:
    /*
    @brief: Constructor
    @param: ROS NodeHandle, Device node, Backend
    */
    Camera(ros::NodeHandle& nh, int deviceNode, int backend = cv::CAP_ANY);  
    ~Camera();

    bool isOpened() const;
    /*
    @brief: Set camera output log level
    @details: log level means that different kind of information will be printed out
    @return: true or false, depending on the success of the operation
    */
    void startCapture();
    /*
    @brief: start camera capture
    */
    void stopCapture();
    /*
    @brief: stop camera capture
    */
    void setRawFrameSize(cv::Size frameSize);
    /*
    @brief: Set camera frame size
    @param: frame size
    */
    void setRawFrameRate(int fps);
    /*
    @brief: Set camera frame rate
    @param: frame rate
    */

    cv::Mat intrinsicMatrix;
    cv::Mat distortionCoefficients;
    cv::VideoCapture cap;

private:
    cv::Size rawFrameSize;
    int rawFrameRate;
};
