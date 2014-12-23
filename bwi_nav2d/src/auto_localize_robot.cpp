#include <ros/ros.h>
#include <bwi_nav2d/AutoLocalizeRobot.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/SendCommand.h>
#include <nav2d_navigator/commands.h>

ros::Publisher mCommandPublisher;

bool add(bwi_nav2d::AutoLocalizeRobot::Request  &req,
         bwi_nav2d::AutoLocalizeRobot::Response &res)
{
    ROS_WARN("Moving right");
    nav2d_operator::cmd cmd;
    cmd.Turn = 0.5;
    cmd.Velocity = 10.0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AutoLocalizeRobot");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("AutoLocalizeRobot", add);
  mCommandPublisher = n.advertise<nav2d_operator::cmd>("cmd", 1);

  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
