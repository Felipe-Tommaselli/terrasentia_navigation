#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <signal.h>
#include <sys/stat.h>

#include <mpc-controller/run-mpc-controller.h>
//#include <mpc-controller/ros-types.h>

#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class MpcDev
{
    public:
        MpcDev(ros::NodeHandle* n);
        void debugPubs(MpcOutput);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void run();
        void wpCb(const nav_msgs::Path::ConstPtr& msg);  

    private:
        double              _yaw;
        nav_msgs::Odometry  _odom;
        double              _last_ts;
        ros::NodeHandle*    _n;
        ros::Publisher      _pub_output;
        ros::Publisher      _pub_pred_vals;
        ros::Publisher      _pub_pts_car;
        ros::Publisher      _pub_mpc_pts;
        ros::Publisher      _pub_twist;
        ros::Publisher      _pub_log;
        ros::Subscriber     _sub_odom;
        ros::Subscriber     _sub_wp;
        RunMpcController*   _run_mpc;

        std::vector<double> _wps_x;
        std::vector<double> _wps_y;

        bool                _b_running; 
};
geometry_msgs::Quaternion getQuaternionForPoints(int i, double x0, double x1, double y0, double y1);