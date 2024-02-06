#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#define UNKNOWN 0.5
#define MIN_PROB 0.001
#define MAX_PROB 1-MIN_PROB

class OccupancyGridMapping : public rclcpp::Node {
    public:
        // コンストラクタ
        OccupancyGridMapping() : Node("occupancy_grid_mapping") {
            fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
            base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
            auto local_map_topic = this->declare_parameter<std::string>("local_map_topic", "local_map");
            auto resolution_ = this->declare_parameter<double>("local_map_resolution", 0.05);
            auto unseen_is_freespace_ = this->declare_parameter<bool>("unseen_is_freespace", true);
            use_scan_header_stamp_for_tfs_ = this->declare_parameter<bool>("use_scan_header_stamp_for_tfs", false);

            // 占有格子マップの初期化処理
            l0_ = logit(UNKNOWN);
            l_min_ = logit(MIN_PROB);
            l_max_ = logit(MAX_PROB);
            l_.resize(width_*width_);
            for (int i = 0; i < width_; i++) {
                for (int j = 0; j < width_; j++) {
                    if (unseen_is_freespace_) {
                        l_[i + width_*j] = l_min_;
                    }
                    else {
                        l_[i + width_*j] = l0_;
                    }
                }
            }
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&OccupancyGridMapping::laserAndLegCallback, this, std::placeholders::_1));
            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(local_map_topic, 10);
            // markers_pub_ = this->create_publisher<visualization_msgs::Marker>("visualization_marker", 20);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
    private:
        std::string scan_topic_;
        std::string fixed_frame_;
        std::string base_frame_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

        double l0_;
        std::vector<double> l_;
        double l_min_;
        double l_max_;

        double resolution_;
        int width_;
        bool use_scan_header_stamp_for_tfs_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_{nullptr};

        double logit(double p) {
            return log(p/(1-p));
        }
        void laserAndLegCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
            bool transform_available;
            rclcpp::Time tf_time;
            if (use_scan_header_stamp_for_tfs_) {
                tf_time = scan_msg->header.stamp;
                try {
                    tf_buffer_->lookupTransform(fixed_frame_, scan_msg->header.frame_id, tf_time);
                    transform_available = true;
                    std::cout << "success transform check" << std::endl;
                    
                } catch (tf2::TransformException & ex) {
                    RCLCPP_INFO(
                        this->get_logger(), "could not transform %s to %s: %s",
                           fixed_frame_.c_str(), scan_msg->header.frame_id.c_str(), ex.what());
                    transform_available = false;
                }
            }
            else {
                tf_time = rclcpp::Time(0);
                transform_available = tf_buffer_->canTransform(fixed_frame_, scan_msg->header.frame_id, tf_time);
            }

            if (transform_available) {
                RCLCPP_INFO(this->get_logger(), "Transform Available");
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridMapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}