#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class RobotDriver
{
  public:
    RobotDriver()
    {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // ball_chaser/command_robot service
        command_robot_service = n.advertiseService("ball_chaser/command_robot", &RobotDriver::HandleDriveRequest, this);
    }

    /// This function should publish the requested linear x and angular velocities to the robot wheel joints
    /// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
    bool HandleDriveRequest(ball_chaser::DriveToTarget::Request &req,
                            ball_chaser::DriveToTarget::Response &res)
    {
        // ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
        const auto& linearX = req.linear_x;
        const auto& angularZ = req.angular_z;

        const auto commandMsg = MakeCommandVel(linearX, angularZ);
        cmd_vel_publisher.publish(commandMsg);

        res.msg_feedback = "linear velocity: " + std::to_string(linearX) + ", angular velocity: " + std::to_string(angularZ);
        // ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

  private:
    geometry_msgs::Twist MakeCommandVel(double linearVelocity, double angularVelocity)
    {
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities
        motor_command.linear.x = linearVelocity;
        motor_command.angular.z = angularVelocity;
        return motor_command;
    }

    ros::NodeHandle n;

    ros::Publisher cmd_vel_publisher;
    ros::ServiceServer command_robot_service;
};

int main(int argc, char **argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    RobotDriver robotDriver;

    // Handle ROS communication events
    ros::spin();

    return 0;
}