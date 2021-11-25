/** 
 *
 *  Planner Requirement Publisher
 */
#include <ros/ros.h>
#include <can/e2o_status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <stdio.h>

int curr_velocity = 0;
geometry_msgs::Pose plan;
//TODO efficiency
//vector<double> pl_reqt;

void e2o_callback(const can::e2o_status::ConstPtr& e2o_info)
{
    curr_velocity = e2o_info->velocity;
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    plan.position.z = msg->pose.position.z;
    plan.position.x = msg->pose.position.x;
    plan.position.y = msg->pose.position.y;
    
    tf::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    plan.orientation.x = roll;
    plan.orientation.y = pitch;
    plan.orientation.z = yaw;
    plan.orientation.w = curr_velocity;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_odom");
    ros::NodeHandle n;
    ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

    ros::Subscriber vel_sub = n.subscribe<can::e2o_status>("/can_node/e2o_info", 1, e2o_callback, noDelay);

    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/lidarPose/floam_odom_estimation_node", 1, pose_callback, noDelay);

    ros::Publisher planner_pub = n.advertise<geometry_msgs::Pose>("/pl_pose", 1);

    while(ros::ok())
    {
        ros::spinOnce();
        planner_pub.publish(plan);
    }
}

