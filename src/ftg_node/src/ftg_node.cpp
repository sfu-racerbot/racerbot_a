#include "ftg_node.hpp"

FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
}

std::vector<int> FollowTheGapNode::find_best_gap(const std::vector<float>& ranges)
{
    std::vector<int> best_gap = {0,0};
    float best_total = 0;

    int gap_start = 0;
    float gap_total = 0;

    for (size_t _i = 0; _i < ranges.size(); _i++) {

        if (ranges[_i] < gap_threshold_) {

            if (gap_total == 0) { // previous range was also less than threshold
                
                continue;

            } else if (gap_total > best_total && (_i - gap_start - 1) > gap_min_width_) {

                best_gap[0] = gap_start;
                best_gap[1] = (_i - 1);
                best_total = gap_total;

            }

            gap_total = 0;

        } else {

            if (gap_total == 0) gap_start = _i; // start of gap
            gap_total += ranges[_i];

        }
    }

    // in case the loop runs through without ending the gap
    if (gap_total > best_total && (_i - gap_start - 1) > gap_min_width_) {
        best_gap[0] = gap_start;
        best_gap[1] = (ranges.size() - 1);
        best_total = gap_total;
    }

    return best_gap;

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}