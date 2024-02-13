#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <leg_tracker_ros2/laser_processor.hpp>
#include <opencv2/opencv.hpp>

#include "leg_tracker_ros2/msg/leg.hpp"
#include "leg_tracker_ros2/msg/leg_array.hpp"

class DetectLegClusters : public rclcpp::Node{
public:
 DetectLegClusters() : Node("detect_leg_clusters") { 
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
    cluster_dist_euclid_ = this->declare_parameter<double>("cluster_dist_euclid", 0.13);
    min_points_per_cluster_ = this->declare_parameter<int>("min_points_per_cluster", 3);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DetectLegClusters::laserCallback, this, std::placeholders::_1));
    // random forestの読み込み
    std::string forest_file = "/root/ros2_ws/src/leg_tracker_ros2/config/trained_leg_detector_res=0.33.yaml";
    forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
    feat_count_ = forest->getVarCount(); // データ数をカウント, 17個
 }

private: 
 std::string fixed_frame_;
 cv::Ptr< cv::ml::RTrees > forest = cv::ml::RTrees::create();
 int feat_count_;
 double cluster_dist_euclid_;
 int min_points_per_cluster_;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) { 
    laser_processor::ScanProcessor processor(*scan);
    processor.splitConnected(cluster_dist_euclid_);
    processor.removeLessThan(min_points_per_cluster_);
    // random forestに必要なOpenCV行列
    cv::Mat tmp_mat(1, feat_count_, CV_32FC1);
    leg_tracker_ros2::msg::LegArray detected_leg_clusters; 
    detected_leg_clusters.header.frame_id = scan->header.frame_id;
    detected_leg_clusters.header.stamp = scan->header.stamp;
 }       
};
     

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectLegClusters>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}