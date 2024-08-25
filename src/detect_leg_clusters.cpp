/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
    num_prev_markers_published_ = 0;
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
    detection_threshold_ = this->declare_parameter<double>("detection_threshold", 0.1);
    cluster_dist_euclid_ = this->declare_parameter<double>("cluster_dist_euclid", 0.13);
    min_points_per_cluster_ = this->declare_parameter<int>("min_points_per_cluster", 3);
    max_points_per_cluster_ = this->declare_parameter<int>("max_points_per_cluster", 100);
    max_detect_distance_ = this->declare_parameter<double>("max_detect_distance", 2.5);
    max_detected_clusters_ = this->declare_parameter<int>("max_detected_clusters", -1);
    use_scan_header_stamp_for_tfs_ = this->declare_parameter<bool>("use_scan_header_stamp_for_tfs", false);
    debug_ = this->declare_parameter<bool>("debug", false);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DetectLegClusters::laserCallback, this, std::placeholders::_1));
    markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 20);
    detected_leg_clusters_pub_ = this->create_publisher<leg_tracker_ros2::msg::LegArray>("detected_leg_clusters", 20);
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
 int max_points_per_cluster_;
 double max_detect_distance_;
 bool use_scan_header_stamp_for_tfs_;
 int max_detected_clusters_;

 int num_prev_markers_published_;
 bool found_cluster_in_front_within_range = false;
 geometry_msgs::msg::PointStamped first_cluster_position;
 bool is_checked_first_cluster = false;
 bool debug_ = false;
 
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;
 rclcpp::Publisher<leg_tracker_ros2::msg::LegArray>::SharedPtr detected_leg_clusters_pub_;
 std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
 std::shared_ptr<tf2_ros::TransformListener> tfl_{nullptr};

 bool isClusterInFrontWithinRange(const geometry_msgs::msg::PointStamped& position) {
   // Define the angular and distance thresholds
   float min_angle_threshold = -0.261799; // -15 degrees
   float max_angle_threshold = 0.261799;  // 15 degrees
   float max_distance_threshold = max_detect_distance_; // Use the class member

   // Calculate the angle of the cluster relative to the robot
   float angle_to_cluster = atan2(position.point.y, position.point.x);

   // Calculate the distance of the cluster from the robot
   float distance_to_cluster = sqrt(pow(position.point.x, 2.0f) + pow(position.point.y, 2.0f));

   // Check if the cluster is within the specified angular range and within the specified distance
   return (angle_to_cluster >= min_angle_threshold && 
            angle_to_cluster <= max_angle_threshold &&
            distance_to_cluster <= max_distance_threshold);
 }
 
 void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) { 
   laser_processor::ScanProcessor processor(*scan);
   processor.splitConnected(cluster_dist_euclid_);
   processor.removeLessThan(min_points_per_cluster_, max_points_per_cluster_);
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
   // 比較関数を用いて、leg_setに変数が追加されるたびにsortが実行される。LiDARとの距離が近い順にsortされる。   
   std::set <leg_tracker_ros2::msg::Leg, CompareLegs> leg_set;
   if (!transform_available) {
      RCLCPP_INFO(this->get_logger(), "Not publishing detected leg clusters because no tf was available");
   }
   else {

      for (auto cluster = processor.getClusters().begin(); cluster != processor.getClusters().end(); ++cluster) {
         geometry_msgs::msg::PointStamped position;
         position.point.x = (*cluster)->getPosition().x();
         position.point.y = (*cluster)->getPosition().y();
         position.point.z = (*cluster)->getPosition().z();
         position.header.stamp = tf_time;
         position.header.frame_id = scan->header.frame_id;

         // クラスタが指定された角度と距離範囲内にあるかチェック
         if (isClusterInFrontWithinRange(position)) {
               found_cluster_in_front_within_range = true;
               first_cluster_position = position;
               break;
         }
      }
      if(debug_) {
         if (!found_cluster_in_front_within_range) {
            RCLCPP_INFO(this->get_logger(), "No clusters found in the specified angular and distance range.");
            return;
         }
         else {
            RCLCPP_INFO(this->get_logger(), "Success to clusters found in the specified angular and distance range.");
         }
      }

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
         if (rel_dist < max_detect_distance_ && (first_cluster_position.point.x == position.point.x || is_checked_first_cluster)) {
            // Classify cluster using random forest classifier
            std::vector<float> f = cf_.calcClusterFeatures(*cluster, *scan);
            for (int k = 0; k < feat_count_; k++)
               tmp_mat.at<float>(k) = (float)(f[k]);
            cv::Mat result;
            forest->getVotes(tmp_mat, result, 0);
            int positive_votes = result.at<int>(1, 1);
            int negative_votes = result.at<int>(1, 0);
            float probability_of_leg = positive_votes / static_cast<double>(positive_votes + negative_votes);

             // クラスタの点群数と推論結果を表示
             // TODO: クラスタの最初の登録処理を追加する
            RCLCPP_INFO(this->get_logger(), "Cluster with %ld points, probability of leg: %f", (*cluster)->size(), probability_of_leg);
            // Consider only clusters that have a confiodence greater than detection_threshold_                 
            // probability_of_leg = probability_of_leg - rel_dist/max_detect_distance_;
            if(!is_checked_first_cluster){
               // TODO: まず人の足として認識されていないクラスタのパラメータ調整
               probability_of_leg += 1.0;
            }
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
            if(!is_checked_first_cluster){
               is_checked_first_cluster = true;
               if(debug_){
                  RCLCPP_INFO(this->get_logger(), "Success to first cluster check");
                  RCLCPP_INFO(this->get_logger(), "iCluster with %ld points, probability of leg: %f", (*cluster)->size(), probability_of_leg);
               }
               break;
            }
         }
         else {
            RCLCPP_INFO(this->get_logger(), "Not yet first cluster check or far distance");
         }

      for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
       cluster != processor.getClusters().end();
       cluster++) {
         // Get position of cluster in laser frame
         geometry_msgs::msg::PointStamped position;
         position.point.x = (*cluster)->getPosition().x();
         position.point.y = (*cluster)->getPosition().y();
         position.point.z = (*cluster)->getPosition().z();
         }
      }
   }       
   // Publish detected legs to /detected_leg_clusters and to rviz
   // They are ordered from closest to the laser scanner to furthest  
   int clusters_published_counter = 0;
   int id_num = 1;     
   for (std::set<leg_tracker_ros2::msg::Leg>::iterator it = leg_set.begin(); it != leg_set.end(); ++it) {
      // Publish to /detected_leg_clusters topic
      leg_tracker_ros2::msg::Leg leg = *it;
      detected_leg_clusters.legs.push_back(leg);
      clusters_published_counter++;

      // Publish marker to rviz
      visualization_msgs::msg::Marker m;
      m.header.stamp = scan->header.stamp;
      m.header.frame_id = fixed_frame_;
      m.ns = "LEGS";
      m.id = id_num++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = leg.position.x ;
      m.pose.position.y = leg.position.y;
      m.pose.position.z = 0.2;
      m.scale.x = 0.13;
      m.scale.y = 0.13;
      m.scale.z = 0.13;
      m.color.a = 1;
      m.color.r = 0;
      m.color.g = 0;
      m.color.b = leg.confidence;
      markers_pub_->publish(m);
      
      if(debug_) RCLCPP_INFO(this->get_logger(), "Marker ID: %d, pos_x: %lf, pos_y: %lf, Probability: %f", m.id, m.pose.position.x, m.pose.position.y, leg.confidence);
      // Comparison using '==' and not '>=' is important, as it allows <max_detected_clusters_>=-1 
      // to publish infinite markers
      if (clusters_published_counter == max_detected_clusters_) 
         break;
   }
   num_prev_markers_published_ = id_num; // For the next callback

   detected_leg_clusters_pub_->publish(detected_leg_clusters);
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