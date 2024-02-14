#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <leg_tracker_ros2/laser_processor.hpp>
#include <leg_tracker_ros2/cluster_features.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "leg_tracker_ros2/msg/leg.hpp"
#include "leg_tracker_ros2/msg/leg_array.hpp"

class DetectLegClusters : public rclcpp::Node{
public:
 DetectLegClusters() : Node("detect_leg_clusters") { 
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
    detection_threshold_ = this->declare_parameter<double>("detection_threshold", -1.0);
    cluster_dist_euclid_ = this->declare_parameter<double>("cluster_dist_euclid", 0.13);
    min_points_per_cluster_ = this->declare_parameter<int>("min_points_per_cluster", 3);
    max_detect_distance_ = this->declare_parameter<double>("max_detect_distance", 10.0);
    use_scan_header_stamp_for_tfs_ = this->declare_parameter<bool>("use_scan_header_stamp_for_tfs", false);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DetectLegClusters::laserCallback, this, std::placeholders::_1));
    // random forestの読み込み
    std::string forest_file = "/root/ros2_ws/src/leg_tracker_ros2/config/trained_leg_detector_res=0.33.yaml";
    forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
    feat_count_ = forest->getVarCount(); // データ数をカウント, 17個
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 }

private: 
 cv::Ptr< cv::ml::RTrees > forest = cv::ml::RTrees::create();

 ClusterFeatures cf_;

 std::string fixed_frame_;
 double detection_threshold_;
 int feat_count_;
 double cluster_dist_euclid_;
 int min_points_per_cluster_;
 double max_detect_distance_;
 bool use_scan_header_stamp_for_tfs_;

 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
 std::shared_ptr<tf2_ros::TransformListener> tfl_{nullptr};
 
 void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) { 
   laser_processor::ScanProcessor processor(*scan);
   processor.splitConnected(cluster_dist_euclid_);
   processor.removeLessThan(min_points_per_cluster_);
   // random forestに必要なOpenCV行列
   cv::Mat tmp_mat(1, feat_count_, CV_32FC1);
   leg_tracker_ros2::msg::LegArray detected_leg_clusters; 
   detected_leg_clusters.header.frame_id = scan->header.frame_id;
   detected_leg_clusters.header.stamp = scan->header.stamp;

   bool transform_available;
   rclcpp::Time tf_time;
   if (use_scan_header_stamp_for_tfs_) {
      tf_time = scan->header.stamp;
      try {
         tf_buffer_->lookupTransform(fixed_frame_, scan->header.frame_id, tf_time);
         transform_available = true;
         std::cout << "success transform check" << std::endl;
         
      } catch (tf2::TransformException & ex) {
         RCLCPP_INFO(
               this->get_logger(), "could not transform %s to %s: %s",
                  fixed_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
         transform_available = false;
      }
   }
   else {
      tf_time = rclcpp::Time(0);
      transform_available = tf_buffer_->canTransform(fixed_frame_, scan->header.frame_id, tf_time);
   }
   // レーザースキャナーまでの相対距離に従って処理された脚を保存する
   std::set <leg_tracker_ros2::msg::Leg, CompareLegs> leg_set;
   if (!transform_available) {
      RCLCPP_INFO(this->get_logger(), "Not publishing detected leg clusters because no tf was available");
   }
   else {
      for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
       cluster != processor.getClusters().end();
       cluster++) {
         // Get position of cluster in laser frame
         geometry_msgs::msg::PointStamped position;
         position.point.x = (*cluster)->getPosition().x();
         position.point.y = (*cluster)->getPosition().y();
         position.point.z = (*cluster)->getPosition().z();
         position.header.stamp = tf_time;
         position.header.frame_id = scan->header.frame_id;
         float rel_dist = pow(position.point.x*position.point.x + position.point.y*position.point.y, 1./2.);
         // Only consider clusters within max_distance. 
         if (rel_dist < max_detect_distance_) {
            // Classify cluster using random forest classifier
            std::vector<float> f = cf_.calcClusterFeatures(*cluster, *scan);
            for (int k = 0; k < feat_count_; k++)
               tmp_mat.at<float>(k) = (float)(f[k]);
            cv::Mat result;
            forest->getVotes(tmp_mat, result, 0);
            int positive_votes = result.at<int>(1, 1);
            int negative_votes = result.at<int>(1, 0);
            float probability_of_leg = positive_votes / static_cast<double>(positive_votes + negative_votes);
            // Consider only clusters that have a confidence greater than detection_threshold_                 
            if (probability_of_leg > detection_threshold_) { 
               // Transform cluster position to fixed frame
               // This should always be succesful because we've checked earlier if a tf was available
               bool transform_successful_2;
               try {
                  tf_buffer_->transform(position, position, fixed_frame_);
                  transform_successful_2= true;
               } catch (tf2::TransformException & ex) {
                  RCLCPP_INFO(this->get_logger(), "%s", ex.what());
                  transform_successful_2 = false;
               }
               if (transform_successful_2) {  
                  // Add detected cluster to set of detected leg clusters, along with its relative position to the laser scanner
                  leg_tracker_ros2::msg::Leg new_leg;
                  new_leg.position.x = position.point.x;
                  new_leg.position.y = position.point.y;
                  new_leg.confidence = probability_of_leg;
                  leg_set.insert(new_leg);
               }
            }
         }
      }
   }       
   // TODO: Publisher
 }
// LiDARから遠い方の脚の相対距離を求める
class CompareLegs {
public:
 bool operator ()(const leg_tracker_ros2::msg::Leg &a, const leg_tracker_ros2::msg::Leg &b) const{
   float rel_dist_a = pow(a.position.x*a.position.x + a.position.y*a.position.y, 1./2.);
   float rel_dist_b = pow(b.position.x*b.position.x + b.position.y*b.position.y, 1./2.);          
   return rel_dist_a < rel_dist_b;
 }
};

};  

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectLegClusters>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}