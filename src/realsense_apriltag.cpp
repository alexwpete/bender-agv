#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>

class TfPoseListener : public rclcpp::Node
{
public:
  TfPoseListener()
  : Node("tf_pose_listener"), first_tf_stored_(false)
  {
    tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/april_pose", 10, std::bind(&TfPoseListener::tf_callback, this, std::placeholders::_1));

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/orbslam3/pose", 10, std::bind(&TfPoseListener::pose_callback, this, std::placeholders::_1));
  }

private:
  std::vector<tf2::Transform> transform_samples_;
  bool first_tf_stored_;

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if (!msg->transforms.empty())
    {
      for (const auto &transform : msg->transforms)
      {
        auto translation = transform.transform.translation;
        auto rotation = transform.transform.rotation;

        // Convert the incoming message's translation and rotation to tf2::Transform
        tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
        tf2::Vector3 trans(translation.x, translation.y, translation.z);
        tf2::Transform current_transform(quat, trans);

        if (!first_tf_stored_)
        {
          transform_samples_.push_back(current_transform);

          if (transform_samples_.size() == 30)
          {
            // Average the samples
            tf2::Transform averaged_transform = average_transforms(transform_samples_);
            first_transform_ = averaged_transform;
            first_tf_stored_ = true;
            RCLCPP_INFO(this->get_logger(), "Stored the first averaged transform as ^W T_A.");
          }
        }

        if (first_tf_stored_)
        {
          // Calculate ^W T_C (Camera in the world frame)
          tf2::Transform inverse_current_transform = current_transform.inverse();
          tf2::Transform camera_in_world = first_transform_ * inverse_current_transform;

          // Print the camera's pose in the world frame
          tf2::Vector3 camera_translation = camera_in_world.getOrigin();
          tf2::Quaternion camera_rotation = camera_in_world.getRotation();

          //RCLCPP_INFO(this->get_logger(), "Camera Pose in World Frame:");
          RCLCPP_INFO(this->get_logger(), "Translation: x=%.6f, \t  y=%.6f, \t z=%.6f",
                      camera_translation.x(), camera_translation.y(), camera_translation.z());
          //RCLCPP_INFO(this->get_logger(), "Rotation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
//                      camera_rotation.x(), camera_rotation.y(), camera_rotation.z(), camera_rotation.w());

          // Convert current quaternion to Euler angles (roll, pitch, yaw)
          tf2::Matrix3x3 m(camera_rotation);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          // Convert radians to degrees
          roll = roll * (180.0 / M_PI);
          pitch = pitch * (180.0 / M_PI);
          yaw = yaw * (180.0 / M_PI);

          //RCLCPP_INFO(this->get_logger(), "Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f",
//                      roll, pitch, yaw);
        }
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received an empty TFMessage.");
    }
  }

  tf2::Transform average_transforms(const std::vector<tf2::Transform> &transforms)
  {
    tf2::Vector3 avg_translation(0, 0, 0);
    tf2::Quaternion avg_quaternion(0, 0, 0, 0);

    for (const auto &transform : transforms)
    {
      avg_translation += transform.getOrigin();
      tf2::Quaternion quat = transform.getRotation();
      avg_quaternion += quat;
    }

    // Average the translation
    avg_translation /= transforms.size();

    // Average and normalize the quaternion
    avg_quaternion.normalize();

    return tf2::Transform(avg_quaternion, avg_translation);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto position = msg->pose.position;
    auto orientation = msg->pose.orientation;

    RCLCPP_INFO(this->get_logger(), "PoseStamped - Position: x=%.6f, \t y=%.6f, \t z=%.6f",
                position.x, position.y, position.z);
    // RCLCPP_INFO(this->get_logger(), "Orientation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
    //             orientation.x, orientation.y, orientation.z, orientation.w);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert radians to degrees
    roll = roll * (180.0 / M_PI);
    pitch = pitch * (180.0 / M_PI);
    yaw = yaw * (180.0 / M_PI);

    // RCLCPP_INFO(this->get_logger(), "Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f",
    //             roll, pitch, yaw);
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  tf2::Transform first_transform_; // Store the first valid transform as ^W T_A
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfPoseListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}