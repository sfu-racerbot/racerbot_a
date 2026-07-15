#include "gap_follow_node.hpp"

GapFollowNode::GapFollowNode() : Node("gap_follow_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Follow node started");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollowNode>());
    rclcpp::shutdown();
    return 0;
}
