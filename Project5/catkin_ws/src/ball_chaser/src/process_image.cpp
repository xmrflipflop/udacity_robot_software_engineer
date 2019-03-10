#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <tuple>
#include <limits>
#include <utility>

struct Boundingbox
{
    /// (x, y)
    using point_unit = float;
    using point2d = std::pair<point_unit, point_unit>;

    Boundingbox()
        : min(std::numeric_limits<point_unit>::max(), std::numeric_limits<point_unit>::max()),
          max(std::numeric_limits<point_unit>::min(), std::numeric_limits<point_unit>::min())
    {
    }

    void Adjust(const point2d &p)
    {
        min.first = std::min(min.first, p.first);
        min.second = std::min(min.second, p.second);
        max.first = std::max(max.first, p.first);
        max.second = std::max(max.second, p.second);
    }

    point_unit GetArea() const
    {
        const auto &height = max.second - min.second;
        const auto &width = max.first - min.first;
        return width * height;
    }

    point2d GetCenter() const
    {
        return point2d{(min.first + max.first) * 0.5F, (min.second + max.second) * 0.5F};
    }

    point2d min;
    point2d max;
};

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // ROS_INFO_STREAM("drive: " << lin_x << " " << ang_z);

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// Calculates drive command based on where boundingBox is in relation to camera image
// @return (linearX, angularZ)
std::pair<float, float> calculate_drive_command(const Boundingbox &boundingBox)
{
    // ROS_DEBUG_STREAM("ball size: " << boundingBox.GetArea() << " min " << boundingBox.min.first << "," << boundingBox.min.second << " max " << boundingBox.max.first << ","
    //                                << boundingBox.max.second);

    constexpr auto MAX_SPEED = 0.5F;
    constexpr auto MAX_STEER = 0.5F;

    // Steer left/right depending on which side of the image the balls is on.
    // Left = positive steer.
    auto steerScale = (0.5F - boundingBox.GetCenter().first) * 2.0F; // [-1, +1]
    // Add a deadband around 0.01 to stop steering
    if (std::abs(steerScale) < 0.01) {
        steerScale = 0.0F;
    }
    const auto steer = MAX_STEER * steerScale;

    // Scale speed to depend on how far the ball is.
    // boundingBox is a scaled ratio of ball to camera frame.
    // The closer the ball is, the larger the bb area will be. Area is (0, 1]
    // We want to robot to stop when the ball is covering ~30% of
    // height and width -> 0.3 * 0.3 = 0.09 ~= 0.1.
    const auto speedScaleOnDistance = 1.0F - std::min(boundingBox.GetArea(), 0.1F) / 0.1F; // (0, 1]
    // We also want the robot to move less when a large steering effort is required.
    const auto speed = MAX_SPEED * speedScaleOnDistance * (1.0 - std::abs(steerScale));

    return {speed, steer};
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // image.encoding = rgb8
    constexpr auto whitePixel = std::tuple<uint8_t, uint8_t, uint8_t>{255, 255, 255};
    constexpr auto channels = 3;

    auto ballBoundingBox = Boundingbox{};
    bool ballFound = false;
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < img.height; ++i)
    {
        // Look through the row
        for (int j = 0; j < img.width; ++j)
        {
            if (whitePixel == std::make_tuple(
                                  img.data[i * img.step + j * channels],
                                  img.data[i * img.step + j * channels + 1],
                                  img.data[i * img.step + j * channels + 2]))
            {
                ballFound = true;
                ballBoundingBox.Adjust({static_cast<float>(j) / static_cast<float>(img.width),
                                        static_cast<float>(i) / static_cast<float>(img.height)});
            }
        }
    }

    if (ballFound)
    {
        const auto driveCommands = calculate_drive_command(ballBoundingBox);
        drive_robot(driveCommands.first, driveCommands.second);
    }
    else
    {
        drive_robot(0.0F, 0.0F);
    }
    
}

int main(int argc, char **argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}