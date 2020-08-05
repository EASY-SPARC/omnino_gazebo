#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class odometryGroup {
  private:
    long double duration;
    std::string hasbString;
    std::string originString = "origin";

    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist twist_msg;
    nav_msgs::Odometry odom;
    ros::Publisher publisherWorld;

    ros::Time timeCurrent;
    ros::Time timePrevious;

    tf::TransformBroadcaster odom_broadcaster;
  
    ros::NodeHandle node;
    ros::Subscriber subscriber;

  public:
    odometryGroup(std::string argument) {
        hasbString = argument;
        subscriber = node.subscribe("/gazebo/link_states", 1, &odometryGroup::onGazeboMessage,this);
        publisherWorld  = node.advertise<nav_msgs::Odometry>("odom" , 1);
    }

    void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
        for (uint i = 0; i < input->name.size(); i++) {
            if (((input->name[i]).find(hasbString) == std::string::npos) || ((input->name[i]).find(originString) == std::string::npos)) {
                continue;
            }

        ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";

        pose_msg = input->pose[i];
        twist_msg = input->twist[i];

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "world";
        odom_trans.child_frame_id = "odom";

        odom_trans.transform.translation.x = pose_msg.position.x;
        odom_trans.transform.translation.y = pose_msg.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation =  pose_msg.orientation;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //set the position
        odom.pose.pose.position.x = pose_msg.position.x;
        odom.pose.pose.position.y = pose_msg.position.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = pose_msg.orientation;

        //set the velocity
        odom.child_frame_id = "odom";
        odom.twist.twist.linear.x = twist_msg.linear.x;
        odom.twist.twist.linear.y = twist_msg.linear.x;
        odom.twist.twist.angular.z = twist_msg.angular.x;

        publisherWorld.publish(odom);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    std::string argument;
    argument = std::string(argv[1]);
    odometryGroup odometry(argument);
    
    ros::spin();
    return 0;
}
