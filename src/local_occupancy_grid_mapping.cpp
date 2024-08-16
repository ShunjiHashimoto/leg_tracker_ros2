#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>

// Custom messages
#include "leg_tracker_ros2/msg/leg.hpp"
#include "leg_tracker_ros2/msg/leg_array.hpp"
#include "leg_tracker_ros2/laser_processor.hpp"

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
    OccupancyGridMapping() : Node("occupancy_grid_mapping"), 
                            scan_sub_(this, std::string("scan")), 
                            non_legs_sub_(this, "non_leg_clusters"),
                            sync(scan_sub_, non_legs_sub_, 100)
    {
        scan_topic = this->declare_parameter<std::string>("scan_topic", "/updated_scan");
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
        cluster_dist_euclid_ = this->declare_parameter<double>("cluster_dist_euclid",  0.13);
        min_points_per_cluster_ = this->declare_parameter<int>("min_points_per_cluster",  3);

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
        
        // 2つのトピックを同期してSubする
        sync.registerCallback(std::bind(&OccupancyGridMapping::laserAndLegCallback, this, std::placeholders::_1, std::placeholders::_2));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // カスタムタイマーインターフェースを使用することで、tf2_ros::Bufferはノードから定期的にコールバックを受け取り、
        // トランスフォーム（座標変換情報）が利用可能かどうかをテストを行う
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(local_map_topic, 10);
        // markers_pub_ = this->create_publisher<visualization_msgs::Marker>("visualization_marker", 20);
    }

private:
    std::string scan_topic;
    std::string fixed_frame_;
    std::string base_frame_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<leg_tracker_ros2::msg::LegArray> non_legs_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, leg_tracker_ros2::msg::LegArray> sync;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;

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

    double cluster_dist_euclid_;
    int min_points_per_cluster_;
 
    void laserAndLegCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg, const leg_tracker_ros2::msg::LegArray::ConstSharedPtr& non_leg_clusters) {
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
            // RCLCPP_INFO(this->get_logger(), "Transform Available");
            std::vector<geometry_msgs::msg::Point> non_legs;
            for(long unsigned int i = 0; i < non_leg_clusters->legs.size(); i++){
                leg_tracker_ros2::msg::Leg leg = non_leg_clusters->legs[i];
                geometry_msgs::msg::Point p;
                p.x = leg.position.x;
                p.y = leg.position.y;
                p.z = 0.0;
                geometry_msgs::msg::PointStamped ps;
                ps.header.frame_id = fixed_frame_;
                ps.header.stamp = tf_time;
                ps.point = p;
                try {
                    tf_buffer_->transform(ps, scan_msg->header.frame_id, tf2::durationFromSec(0.0));
                    geometry_msgs::msg::Point temp_p = ps.point;
                    non_legs.push_back(temp_p);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(this->get_logger(),"Local map tf error: %s", ex.what());
                }
            }
            // 追跡された足の座標変換、どのscanデータが人に対応するかを決定し、それらのエリアはfree-space１とする
            std::vector<bool> is_sample_human;                 
            is_sample_human.resize(scan_msg->ranges.size(), false);
            sensor_msgs::msg::LaserScan scan = *scan_msg;
            laser_processor::ScanProcessor processor(scan);
            processor.splitConnected(cluster_dist_euclid_);
            processor.removeLessThan(min_points_per_cluster_);
            for (std::list<laser_processor::SampleSet*>::iterator c_iter = processor.getClusters().begin(); c_iter != processor.getClusters().end(); ++c_iter) {
                bool is_cluster_human = true;
                tf2::Vector3 c_pos = (*c_iter)->getPosition();
                // Check every point in the <non_legs> message to see
                // if the scan cluster is within an epsilon distance of the cluster
                for (std::vector<geometry_msgs::msg::Point>::iterator non_leg = non_legs.begin(); non_leg != non_legs.end(); ++non_leg) {
                    double dist = sqrt(pow((c_pos.x() - (*non_leg).x), 2) + pow((c_pos.y() - (*non_leg).y), 2));
                    if (dist < 0.1) {
                        non_legs.erase(non_leg);
                        is_cluster_human = false;
                        break;
                    }
                }
                // Set all scan samples in the cluster to <is_cluster_human>
                for (laser_processor::SampleSet::iterator s_iter = (*c_iter)->begin();
                    s_iter != (*c_iter)->end();
                    ++s_iter) {
                    is_sample_human[(*s_iter)->index] = is_cluster_human;        
                }
            }
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
                nav_msgs::msg::OccupancyGrid m_msg;
                m_msg.header.stamp = scan_msg->header.stamp; //ros::Time::now();
                m_msg.header.frame_id = fixed_frame_;
                m_msg.info.resolution = resolution_;
                m_msg.info.width = width_;
                m_msg.info.height = width_;
                m_msg.info.origin.position.x = grid_center_pos_x_ - (width_/2.0)*resolution_;
                m_msg.info.origin.position.y = grid_center_pos_y_ - (width_/2.0)*resolution_;
                for (int i = 0; i < width_; i++) {        
                    for (int j = 0; j < width_; j++) {
                        m_msg.data.push_back((int)(inverseLogit(l_[width_*i + j])*100));
                    }
                }

                // Publish!
                map_pub_->publish(m_msg);
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
 
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
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
