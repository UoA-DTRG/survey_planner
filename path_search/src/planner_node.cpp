#include "ros/ros.h"
#include "planner_class.h"
#include "consts.h"
#include "glog/logging.h"

using namespace std;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle n("~");
    tf::TransformListener l;

    fiesta::Fiesta<FIESTA_DEPTH_TYPE, geometry_msgs::TransformStamped::ConstPtr> fiesta(n, &l);
    planner_class planner(&fiesta, &l);

    // Setup timer
    ros::NodeHandle nh;
    //ros::CallbackQueue high_priority_queue;
    //nh.setCallbackQueue(&high_priority_queue);

    ros::Timer timer = nh.createTimer(ros::Duration(0.05), &planner_class::fsm_tick, &planner);
    ros::Timer replan_timer = nh.createTimer(ros::Duration(2.0), &planner_class::replan_tick, &planner);

    // The timer tasks are more important than async mapping tasks.
    // We will put the timer tasks on a separate queue for speed.
    // This might produce race conditions with how the map is updated, but should _hopefully_ be caught during replan ticks
    ros::AsyncSpinner main_spinner(1); // Main must be single threaded to not break FIESTA
    //ros::AsyncSpinner high_priority_spinner(1, &high_priority_queue); // FSM and replan ticks are mutex gated anyway.

    main_spinner.start();
    //high_priority_spinner.start();
    ros::waitForShutdown();

    //ros::spin();
    // Use MultiThreadedSpinner to async call our timers and callback tasks
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
}