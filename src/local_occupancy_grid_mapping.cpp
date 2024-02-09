#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#define ALPHA 0.2
#define BETA 0.1
#define OBSTACLE 0.7
#define FREE_SPACE 0.4
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
    resolution_ = this->declare_parameter<double>("local_map_resolution", 0.05);
    invalid_measurements_are_free_space_ = this->declare_parameter<bool>("invalid_measurements_are_free_space", false);
    reliable_inf_range_ = this->declare_parameter<double>("reliable_inf_range", 5.0);
    unseen_is_freespace_ = this->declare_parameter<bool>("unseen_is_freespace", true);
    use_scan_header_stamp_for_tfs_ = this->declare_parameter<bool>("use_scan_header_stamp_for_tfs", false);
    shift_threshold_ = this->declare_parameter<double>("shift_threshold", 1.0);
    width_ = this->declare_parameter<int>("local_map_cells_per_side", 400);

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

 bool grid_center_pos_found_;
 double grid_center_pos_x_;
 double grid_center_pos_y_;
 double shift_threshold_;

 bool unseen_is_freespace_;
 bool invalid_measurements_are_free_space_;
 double reliable_inf_range_;
 
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
        // TODO: 追跡された足の座標変換、どのscanデータが人に対応するかを決定し、それらのエリアはfree-space１とする
        // TODO: non_legsを定義
        std::vector<bool> is_sample_human;                 
        is_sample_human.resize(scan_msg->ranges.size(), false);
        sensor_msgs::msg::LaserScan scan = *scan_msg;
        // TODO: non_legsをもとに、人の足に一致するスキャンデータがどれか割り当てる

        // ステップ2：占有格子マップを更新する
        // fixed_frameから見たscan_dataを取得する
        bool transform_succesful;
        geometry_msgs::msg::PoseStamped init_pose;
        geometry_msgs::msg::PoseStamped laser_pose_fixed_frame;
        init_pose.header.frame_id = scan.header.frame_id;
        init_pose.pose.orientation = this->createQuaternionMsgFromYaw(0.0);
        init_pose.header.stamp = tf_time;    
        try {
            laser_pose_fixed_frame = tf_buffer_->transform(init_pose, fixed_frame_, tf2::durationFromSec(0.0));
            transform_succesful = true;
        }
        catch (tf2::TransformException ex) {        
            RCLCPP_ERROR(this->get_logger(), "Local map tf error: %s", ex.what());
            transform_succesful = false;
        }

        if (transform_succesful) {
            // scanデータの座標を取得
            double laser_x = laser_pose_fixed_frame.pose.position.x;
            double laser_y = laser_pose_fixed_frame.pose.position.y;
            double laser_yaw = tf2::getYaw(laser_pose_fixed_frame.pose.orientation);
            // fixed frameからの相対的な占有格子マップ座標を取得 
            if(grid_center_pos_found_ == false) {
                grid_center_pos_found_ = true;
                grid_center_pos_x_ = laser_x;
                grid_center_pos_y_ = laser_y;
            }

            // lidar中心にlocal gridを寄せる必要が有るか確認
            // lidarの現在位置と地図の中心との距離を計算
            if (sqrt(pow(grid_center_pos_x_ - laser_x, 2) + pow(grid_center_pos_y_ - laser_y, 2)) > shift_threshold_) {
                // 何セル分ずれているかを計算
                int translate_x = -(int)round((grid_center_pos_x_ - laser_x)/resolution_); 
                int translate_y = -(int)round((grid_center_pos_y_ - laser_y)/resolution_); 

                // 必要であれば、後で最適化するために変換する
                std::vector<double> l_translated;
                l_translated.resize(width_*width_);
                for (int i = 0; i < width_; i++) { 
                    for (int j = 0; j < width_; j++) {
                        int translated_i = i + translate_x;
                        int translated_j = j + translate_y;
                        if (translated_i >= 0 and translated_i < width_ and translated_j >= 0 and translated_j < width_) {
                            l_translated[i + width_*j] = l_[translated_i + width_*translated_j];
                        }
                        else {
                            if (unseen_is_freespace_) 
                            l_translated[i + width_*j] = l_min_;
                            else
                            l_translated[i + width_*j] = l0_;                
                        }
                    }
                }
                l_ = l_translated;
                grid_center_pos_x_ = laser_x;
                grid_center_pos_y_ = laser_y;
            }

            // 占有格子マップを更新
            for (int i = 0; i < width_; i++) { 
                for (int j = 0; j < width_; j++) {
                    double m_update;

                    // find dist and angle of current cell to laser position
                    double dist = sqrt(pow(i*resolution_ + grid_center_pos_x_ - (width_/2.0)*resolution_ - laser_x, 2.0) + pow(j*resolution_ + grid_center_pos_y_ - (width_/2.0)*resolution_ - laser_y, 2.0));
                    double angle = betweenPIandNegPI(atan2(j*resolution_ + grid_center_pos_y_ - (width_/2.0)*resolution_ - laser_y, i*resolution_ + grid_center_pos_x_ - (width_/2.0)*resolution_ - laser_x) - laser_yaw);
                    bool is_human; 

                    if (angle > scan.angle_min - scan.angle_increment/2.0 and angle < scan.angle_max + scan.angle_increment/2.0) {
                        // find applicable laser measurement
                        double closest_beam_angle = round(angle/scan.angle_increment)*scan.angle_increment;
                        int idx_without_bounds_check = round(angle/scan.angle_increment) + scan.ranges.size()/2;
                        int closest_beam_idx = std::max(0, std::min(static_cast<int>(scan.ranges.size() - 1), idx_without_bounds_check));
                        is_human = is_sample_human[closest_beam_idx]; 

                        // 有効なデータであるかをチェック 
                        bool valid_measurement;
                        if(scan.range_min <= scan.ranges[closest_beam_idx] && scan.ranges[closest_beam_idx] <= scan.range_max) { 
                            // this is a valid measurement.
                            valid_measurement = true;
                        } 
                        else if( !std::isfinite(scan.ranges[closest_beam_idx]) && scan.ranges[closest_beam_idx] < 0) {
                            // object too close to measure.
                            valid_measurement = false;
                        }
                        else if( !std::isfinite(scan.ranges[closest_beam_idx] ) && scan.ranges[closest_beam_idx] > 0) {
                            // no objects detected in range.
                            valid_measurement = true;
                        } 
                        else if( std::isnan(scan.ranges[closest_beam_idx]) ) {
                            // this is an erroneous, invalid, or missing measurement.
                            valid_measurement = false;
                        } 
                        else {
                            // the sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
                            valid_measurement = false;
                        }

                        if (valid_measurement) {
                            double dist_rel = dist - scan.ranges[closest_beam_idx];
                            double angle_rel = angle - closest_beam_angle;
                            if (dist > scan.range_max 
                                or dist > scan.ranges[closest_beam_idx] + ALPHA/2.0 
                                or fabs(angle_rel)>BETA/2 
                                or (!std::isfinite(scan.ranges[closest_beam_idx]) and dist > reliable_inf_range_)) {
                                m_update = UNKNOWN;
                            }
                            else if (scan.ranges[closest_beam_idx] < scan.range_max and fabs(dist_rel)<ALPHA/2 and !is_human) {
                                m_update = OBSTACLE;
                            } 
                            else {
                                m_update = FREE_SPACE;
                            } 
                        }
                        else {
                            // assume cells corresponding to erroneous measurements are either in freespace or UNKNOWN
                            if (invalid_measurements_are_free_space_) {
                                m_update = FREE_SPACE; 
                            }
                            else { 
                                m_update = UNKNOWN;
                            }
                        }
                    }
                    else {
                        m_update = UNKNOWN;
                    }

                    // update l_ using m_update
                    l_[i + width_*j] = (l_[i + width_*j] + logit(m_update) - l0_);
                    if (l_[i + width_*j] < l_min_) {
                        l_[i + width_*j] = l_min_;
                    }
                    else if (l_[i + width_*j] > l_max_) {
                    l_[i + width_*j] = l_max_;
                    }
                }
            }

            // OccupancyGrid msgを作成
            nav_msgs::msgs::OccupancyGrid m_msg;
            m_msg.header.stamp = scan_msg->header.stamp; //ros::Time::now();
            m_msg.header.frame_id = fixed_frame_;
            m_msg.info.resolution = resolution_;
            m_msg.info.width = width_;
            m_msg.info.height = width_;
            m_msg.info.origin.position.x = grid_centre_pos_x_ - (width_/2.0)*resolution_;
            m_msg.info.origin.position.y = grid_centre_pos_y_ - (width_/2.0)*resolution_;
            for (int i = 0; i < width_; i++) {        
                for (int j = 0; j < width_; j++) {
                    m_msg.data.push_back((int)(inverseLogit(l_[width_*i + j])*100));
                }
            }

            // Publish!
            map_pub_.publish(m_msg);
        }
    }
 }

 double logit(double p) {
    return log(p/(1-p));
 }

 double inverseLogit(double p) {
     return exp(p)/(1+exp(p));
 }

 double betweenPIandNegPI(double angle_in) {
    double between_0_and_2PI = fmod(angle_in, 2*M_PI);
    if (between_0_and_2PI < M_PI) {
        return between_0_and_2PI;
    } 
    else {
        return between_0_and_2PI - 2*M_PI;
    }
 }
 
 auto createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q); // tf2 msgをgeometry_msgs型に変換
 }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridMapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}