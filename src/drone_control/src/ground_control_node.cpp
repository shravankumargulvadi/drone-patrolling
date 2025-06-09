#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <vector>
#include <tuple>

class GCSNode : public rclcpp::Node
{
public:
    GCSNode() : Node("gcs_node")
    {
        this->declare_parameter<std::vector<std::string>>("drone_ids", {"drone1"});
        this->get_parameter("drone_ids", drone_ids_);

        for (const auto &id : drone_ids_)
        {
            std::string topic = "/" + id + "/trajectory_upload";
            auto pub = this->create_publisher<geometry_msgs::msg::PoseArray>(topic, 10);
            publishers_.emplace_back(pub);
        }

        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&GCSNode::send_trajectory, this));
    }

private:
    std::vector<std::string> drone_ids_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool sent_ = false;

    void send_trajectory()
    {
        if (sent_) return;

        geometry_msgs::msg::PoseArray traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        std::vector<std::tuple<float, float, float>> waypoints = {
            {0.0f, 0.0f, -8.0f},
            {10.0f, 0.0f, -8.0f},
            {10.0f, 10.0f, -8.0f},
            {0.0f, 10.0f, -8.0f},
            {0.0f, 0.0f, -8.0f}};

        for (auto &[x, y, z] : waypoints)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            traj.poses.push_back(pose);
        }

        for (size_t i = 0; i < publishers_.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing trajectory to %s", drone_ids_[i].c_str());
            publishers_[i]->publish(traj);
        }

        sent_ = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GCSNode>());
    rclcpp::shutdown();
    return 0;
}
