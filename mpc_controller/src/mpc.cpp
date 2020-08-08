#include <ros/package.h>
#include <string>
#include <mpc/mpc.h>

using namespace std;

MpcDev::MpcDev(ros::NodeHandle* n)
{
    _n = n;

    ros::NodeHandle param_n("~");
    string wp_topic_name, output_topic_name, pred_vals_topic_name, mpc_config_path, twist_topic_name;
    string odom_topic_name;
    param_n.param<std::string>("wp_topic_name", wp_topic_name, "wp_topic");
    param_n.param<std::string>("output_topic_name", output_topic_name, "mpc_output");
    param_n.param<std::string>("pred_vals_topic_name", pred_vals_topic_name, "mpc_pred_vals");
    param_n.param<std::string>("mpc_config_path", mpc_config_path, "mpc.config");
    param_n.param<std::string>("twist_topic_name", twist_topic_name, "cmd_vel");
    param_n.param<std::string>("odom_topic_name", odom_topic_name, "odom");

    // Subscribers:
	_sub_odom =         _n->subscribe(odom_topic_name, 1, &MpcDev::odomCallback, this, ros::TransportHints().tcpNoDelay(true));
    _sub_wp =           _n->subscribe(wp_topic_name, 1, &MpcDev::wpCb, this, ros::TransportHints().tcpNoDelay(true));
    // Publishers:
    _pub_output =       _n->advertise<std_msgs::Float32MultiArray>( output_topic_name, 1 );
    _pub_pred_vals =    _n->advertise<geometry_msgs::PoseArray>( pred_vals_topic_name, 1 );
    _pub_pts_car =      _n->advertise<geometry_msgs::PoseArray>( "/pts_car", 1 );
    _pub_mpc_pts =      _n->advertise<nav_msgs::Path>( "/mpc_pts", 1 );
    _pub_twist =        _n->advertise<geometry_msgs::TwistStamped>( twist_topic_name, 1 );

    ROS_INFO_STREAM("\nwp_topic_name:\t\t" << wp_topic_name <<
                    "\noutput_topic_name:\t" << output_topic_name <<
                    "\ntwist_topic_name:\t" << twist_topic_name);

    //string coinfig_path = ros::package::getPath("mpc_controller") + "/" + mpc_config_path;
    string coinfig_path = "/home/companion/catkin_ws/src/terrasentia_navigation/mpc_controller/" + mpc_config_path;

    ROS_INFO_STREAM("mpc_controller pkg path: " << coinfig_path);

    // Create MPC controller object
    _run_mpc = new RunMpcController(coinfig_path);

    _last_ts = ros::Time::now().toSec();
    _b_running = false;
}

geometry_msgs::Quaternion getQuaternionForPoints(int i, double x0, double x1, double y0, double y1)
{
    double heading = 0;
    geometry_msgs::Quaternion q;
    if(x1!=x0) heading = atan2(y1-y0, x1-x0);
    tf2::Quaternion quat;
    quat.setRPY(0,0, heading);
    q = tf2::toMsg(quat);
    return q;
}

void MpcDev::debugPubs(MpcOutput mpc_output)
{
    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray pa;
    double heading;

    pa.header.frame_id="base_link";
    for (int i = 0; i < mpc_output.x_pred_vals.size(); i++)
    {
        pose.position.x = mpc_output.x_pred_vals[i];
        pose.position.y = mpc_output.y_pred_vals[i];
        if(i < mpc_output.x_pred_vals.size() - 1) pose.orientation = getQuaternionForPoints(i,  mpc_output.x_pred_vals[i], mpc_output.x_pred_vals[i+1], mpc_output.y_pred_vals[i], mpc_output.y_pred_vals[i+1]);

        pa.poses.push_back(pose);
    }
    _pub_pred_vals.publish(pa);
    /*
    pa.poses.clear();
    for (int i = 0; i < mpc_output.ptsx_car.size(); i++)
    {
        pose.position.x = mpc_output.ptsx_car(i);
        pose.position.y = mpc_output.ptsy_car(i);
        if (i < mpc_output.ptsx_car.size() - 1) pose.orientation = getQuaternionForPoints(i, mpc_output.ptsx_car(i), mpc_output.ptsx_car(i+1), mpc_output.ptsy_car(i), mpc_output.ptsy_car(i+1));

        pa.poses.push_back(pose);
    }
    _pub_pts_car.publish(pa);
    
    pa.poses.clear();
    for (int i = 0; i < mpc_output.mpc_ptsx.size(); i++)
    {
        pose.position.x = mpc_output.mpc_ptsx(i);
        pose.position.y = mpc_output.mpc_ptsy(i);
        if (i < mpc_output.mpc_ptsx.size()-1) pose.orientation = getQuaternionForPoints(i, mpc_output.mpc_ptsx(i), mpc_output.mpc_ptsx(i+1), mpc_output.mpc_ptsy(i), mpc_output.mpc_ptsy(i+1));

        pa.poses.push_back(pose);
    }
    _pub_mpc_pts.publish(pa);
    */
    std_msgs::Float32MultiArray f32ma;
    f32ma.data.push_back(mpc_output.vx);
    f32ma.data.push_back(mpc_output.wz);
    _pub_output.publish(f32ma);
}

void MpcDev::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO_STREAM("mpc_dev odomCb");
    double roll, pitch, yaw;
    _odom.header = msg->header;
    _odom.pose = msg->pose;
    _odom.twist = msg->twist;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(_odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    _yaw = yaw;
}

void MpcDev::run()
{
    if(_wps_x.size() > 0)
    {     
        _b_running = true;   
        MpcInput in;
        MpcOutput mpc_output;
        in.wp_x = _wps_x;
        in.wp_y = _wps_y;
        double cur_ts = ros::Time::now().toSec();
        in.dt = cur_ts - _last_ts;
        _last_ts = cur_ts;
        ros::Time init_time = _odom.header.stamp;
        in.x = _odom.pose.pose.position.x;
        in.y = _odom.pose.pose.position.y;
        in.theta = _yaw;
        in.vx = _odom.twist.twist.linear.x;
        in.wz = _odom.twist.twist.angular.z;
        ROS_INFO_STREAM("MpcInput x " << in.x << " y " << in.y << " theta " << in.theta << " vx " << in.vx << " wz " << in.wz);

        // Run MPC algorithm
        mpc_output = _run_mpc->mpcController(in);

        this->debugPubs(mpc_output);
        geometry_msgs::TwistStamped mpc_cmd;
        mpc_cmd.header.stamp = init_time;
        mpc_cmd.twist.linear.x = 0.6;
        mpc_cmd.twist.angular.z = mpc_output.wz;
        _pub_twist.publish(mpc_cmd);
    }

}

void MpcDev::wpCb(const nav_msgs::Path::ConstPtr& msg)
{
    _wps_x.clear();
    _wps_y.clear();
    ROS_INFO_STREAM("mpc_dev wpCb received size " << msg->poses.size());
    for(int i = 0; i < msg->poses.size(); i++)
    {
        _wps_x.push_back(msg->poses[i].pose.position.x);
        _wps_y.push_back(msg->poses[i].pose.position.y);
    }
}
