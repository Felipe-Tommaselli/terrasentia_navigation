#include <mpc/mpc.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
// Replacement SIGINT handler
void mySigIntHandler(int sig){ g_request_shutdown = 1; }

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "mpc_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("mpc_node");
    MpcDev mpc_dev(&nh);
    ROS_INFO_STREAM("mpc_node back");
    double mpc_loop_rate;
    nh.param("loop_rate", mpc_loop_rate, 100.0);
    ros::WallRate loop_rate(mpc_loop_rate);
        
    while(!g_request_shutdown)
    {
        loop_rate.sleep();
        mpc_dev.run();
        ros::spinOnce();
    }
}