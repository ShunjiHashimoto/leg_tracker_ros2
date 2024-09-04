#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker,  MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point, Quaternion
import tf2_geometry_msgs
import tf2_ros
import tf2_py
import tf2_kdl
import copy
import timeit
import message_filters

from leg_tracker_ros2.msg import Leg, LegArray, Person, PersonArray
from config import LegTrackerParam

import sys
import signal
import numpy as np
import random
import math 
import json
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from pykalman import KalmanFilter
from logging import getLogger, config

# コンストラクタ
class DetectedCluster:
    def __init__(self, pos_x, pos_y, confidence, in_free_space):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.confidence = confidence
        self.in_free_space = in_free_space
        self.in_free_space_bool = None

# 物体追跡、人の足、人全体、もしくは任意の物体を追跡する
class ObjectTracked:
    new_leg_id_num = 1
    
    def __init__(self, x, y, now, confidence, is_person, in_free_space):        
        self.id_num = ObjectTracked.new_leg_id_num
        ObjectTracked.new_leg_id_num += 1
        self.color = (random.random(), random.random(), random.random())
        self.last_seen = now # ROSのタイムスタンプ
        self.seen_in_current_scan = True
        self.times_seen = 1
        self.confidence = confidence
        self.dist_travelled = 0.
        self.is_person = is_person
        self.deleted = False
        self.in_free_space = in_free_space
        self.vel_history = []  # 速度の履歴を保存するリスト
        self.is_static = False  # 静的かどうかのフラグ
        self.is_single_leg = False  # 片足のみのトラックかどうかのフラグ 追加
        self.avg_speed = 0.0

        # 定常速度モデルのカルマンフィルタを使って人を追跡する  
        # LiDARデータの値は正確であるため観測値をより信頼するように式を立てる  
        # TODO: 使用するLiDARによってパラメータ変更が必要
        scan_frequency = 40
        # scan_frequency = 7.5 
        delta_t = 1./scan_frequency
        if scan_frequency > 7.49 and scan_frequency < 7.51:
            std_process_noise = 0.06666
        elif scan_frequency > 9.99 and scan_frequency < 10.01:
            std_process_noise = 0.05
        # elif scan_frequency > 14.99 and scan_frequency < 15.01:
        elif scan_frequency > 14.99:
            std_process_noise = 0.03333
        else:
            print ("Scan frequency needs to be either 7.5, 10 or 15 or the standard deviation of the process noise needs to be tuned to your scanner frequency")
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1
        # 位置、速度、観測値の分散
        var_pos = std_pos**2
        var_vel = std_vel**2
        # 観測ノイズは、カルマンフィルタを更新するときとデータの関連付けを行うときで異なると仮定
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2
        self.filtered_state_means = np.array([x, y, 0, 0])
        self.pos_x = x
        self.pos_y = y
        self.vel_x = 0
        self.vel_y = 0
        self.filtered_state_covariances = 0.5*np.eye(4) 

        # 状態方程式
        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])
        # 観測方程式 
        observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])
        # 遷移共分散行列
        transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])
        # 観測共分散行列
        observation_covariance =  var_obs_local*np.eye(2)
        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )
 
    def update(self, observations):
        self.filtered_state_means, self.filtered_state_covariances = (
            self.kf.filter_update (
                self.filtered_state_means,
                self.filtered_state_covariances,
                observations
            )
        )

        # 移動した距離を足し合わせる。些細な変化は無視するようにif文を追加
        delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.)
        if delta_dist_travelled > 0.01: 
            self.dist_travelled += delta_dist_travelled

        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]
        
        # 静的障害物の判定
        current_speed = math.sqrt(self.vel_x**2 + self.vel_y**2)
        self.vel_history.append(current_speed)
        # 速度の履歴が一定数を超えたら、最も古いデータを削除
        if len(self.vel_history) > LegTrackerParam.vel_history_size: 
            self.vel_history.pop(0)
        # 速度の平均値を計算
        self.avg_speed = sum(self.vel_history) / len(self.vel_history)
        # 平均速度が閾値以下であれば静的障害物と判定
        self.is_static = self.avg_speed < LegTrackerParam.obstacle_speed_threshold
        #if not self.is_static:
            #print(f"Object {self.id_num} is considered moving based on average speed {avg_speed}")

class KalmanMultiTrackerNode(Node):    
    max_cost = 9999999

    def __init__(self):      
        super().__init__("KalmanMultiTracker")
        self.objects_tracked = []
        self.potential_leg_pairs = set()
        self.potential_leg_pair_initial_dist_travelled = {}
        self.people_tracked = []
        self.prev_track_marker_id = 0
        self.prev_person_marker_id = 0
        self.prev_time = None
        self.buffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.buffer, self)
        self.local_map = None
        self.new_local_map_received = True
        self.prev_person_id = None
        random.seed(1) 
        with open('/root/ros2_ws/src/leg_tracker_ros2/json/log_config.json', 'r') as f:
            log_conf = json.load(f)
        self.logger = getLogger(__name__)
        if LegTrackerParam.debug:
            log_conf["handlers"]["fileHandler"]["level"] = "DEBUG"
            log_conf["handlers"]["consoleHandler"]["level"] = "DEBUG"
        config.dictConfig(log_conf)
        self.logger.debug('joint leg tracker node start')

        # ROSパラメータ
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("fixed_frame", "laser")
        self.declare_parameter("scan_frequency", 7.5)
        self.declare_parameter("max_leg_pairing_dist", 0.8)
        self.declare_parameter("dist_travelled_together_to_initiate_leg_pair", 0.5)
        self.declare_parameter("debug", False)
        self.declare_parameter("confidence_percentile", 0.9)
        self.declare_parameter("confidence_threshold_to_maintain_track", 0.1)
        self.declare_parameter("max_std", 0.9)
        self.declare_parameter("in_free_space_threshold", 0.06)
        self.scan_topic = self.get_parameter("scan_topic").value
        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.scan_frequency = self.get_parameter("scan_frequency").value
        self.max_leg_pairing_dist = self.get_parameter("max_leg_pairing_dist").value
        self.dist_travelled_together_to_initiate_leg_pair = self.get_parameter("dist_travelled_together_to_initiate_leg_pair").value
        self.debug = self.get_parameter("debug").value
        self.confidence_percentile = self.get_parameter("confidence_percentile").value
        self.confidence_threshold_to_maintain_track = self.get_parameter("confidence_threshold_to_maintain_track").value
        self.max_std = self.get_parameter("max_std").value

        self.publish_occluded = self.get_parameter_or("publish_occluded", True)
        self.publish_people_frame = self.get_parameter_or("publish_people_frame", self.fixed_frame)
        self.use_scan_header_stamp_for_tfs = self.get_parameter_or("use_scan_header_stamp_for_tfs", False)
        self.publish_detected_people = self.get_parameter_or("display_detected_people", False)
        self.in_free_space_threshold = self.get_parameter_or("in_free_space_threshold", 0.06).value

        self.mahalanobis_dist_gate = scipy.stats.norm.ppf (1.0 - (1.0-self.confidence_percentile)/2., 0, 1.0)
        self.max_cov = self.max_std**2
        self.latest_scan_header_stamp_with_tf_available = self.get_clock().now()

        # ROS publishers
        self.people_tracked_pub = self.create_publisher(PersonArray, "people_tracked", 10)
        self.follow_target_person_pub = self.create_publisher(Person, "follow_target_person", 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
        self.non_leg_clusters_pub = self.create_publisher(LegArray, "non_leg_clusters", 10)

        # ROS subscribers 
        self.detected_clusters_sub = self.create_subscription(LegArray, "detected_leg_clusters", self.detected_clusters_callback, 10)
        self.local_map_sub = self.create_subscription(OccupancyGrid, "local_map", self.local_map_callback, 10)

        rclpy.spin(self)
        
    def local_map_callback(self, map):
        self.local_map = map
        self.new_local_map_received = True
      
    # ローカルマップをもとに、ある座標がfree-spaceである確度を計算する
    def how_much_in_free_space(self, x, y):
        # ローカルマップがなければ、free-spaceがないとする 
        if self.local_map == None:
            return self.in_free_space_threshold*2.0
        # ローカルマップ座標のx,y座標を取得する
        map_x = int(round((x - self.local_map.info.origin.position.x)/self.local_map.info.resolution))
        map_y = int(round((y - self.local_map.info.origin.position.y)/self.local_map.info.resolution))
        # カーネルサイズに基づいて、(map_x, map_y)の中心にあるローカルマップの値を取得する
        sum = 0
        kernel_size = 2
        for i in range(map_x-kernel_size, map_x+kernel_size):
            for j in range(map_y-kernel_size, map_y+kernel_size):
                if i + j*self.local_map.info.height < len(self.local_map.data):
                    sum += self.local_map.data[i + j*self.local_map.info.height]
                else:  
                    # 地図から外れた、ローカルマップの端に近いため、free-spaceがないとする
                    return self.in_free_space_threshold*2
        percent = sum/(((2.*kernel_size + 1)**2.)*100.)
        return percent

    # 各観測が既存のどのトラックに対応するかを決める    
    def match_detections_to_tracks_GNN(self, objects_tracked, objects_detected):
        matched_tracks = {}
        # 全ての検出結果とトラッキング結果の間のマハラノビス距離に基づいて行列を作成する
        match_dist = [] # 全ての検出オブジェクトと全てのトラック間のマハラノビス距離が格納
        eligable_detections = [] # at_leaste_one_trackに含まれれば、計算高速化のために検出結果を格納する  
        for detect in objects_detected:
            at_least_one_track_in_range = False
            new_row = []
            for track in objects_tracked:
                # 人と、非自由空間における障害物がマッチングすることを防ぐ
                if track.is_person and not detect.in_free_space_bool:
                    print(f"cost is max: {track.id_num}")
                    cost = self.max_cost
                else:
                    # マッチングのためにマハラノビス距離を使う
                    cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                    mahalanobis_dist = math.sqrt(((detect.pos_x-track.pos_x)**2 + (detect.pos_y-track.pos_y)**2)/cov) # = scipy.spatial.distance.mahalanobis(u,v,inv_cov)**2
                    if mahalanobis_dist < self.mahalanobis_dist_gate:
                        cost = mahalanobis_dist
                        at_least_one_track_in_range = True
                    else:
                        cost = self.max_cost
                new_row.append(cost)
            # 検出結果があるトラック結果に少なくとも1つでも含まれれば、munkresマッチングにおけるeligable detectionとして追加する
            if at_least_one_track_in_range: 
                match_dist.append(new_row)
                eligable_detections.append(detect)
        # コストが低いもの同士でマッチするように割当を行う
        if match_dist:
            elig_detect_indexes, track_indexes = linear_sum_assignment(match_dist)
            for elig_detect_idx, track_idx in zip(elig_detect_indexes, track_indexes):
                if match_dist[elig_detect_idx][track_idx] < self.mahalanobis_dist_gate:
                    detect = eligable_detections[elig_detect_idx]
                    track = objects_tracked[track_idx]
                    matched_tracks[track] = detect
        # 何番目のトラッキング結果が何番目の検出結果が紐付けられた情報を返す
        return matched_tracks

    # detect_leg_clustersがPublishされるたびに呼ばれるコールバック関数
    # 前フレームからのトラックされたクラスタと新しく検出されたクラスタをマッチする
    def detected_clusters_callback(self, detected_clusters_msg):
        # 処理前に、ローカルマップがPublishされるのを待つ
        if self.use_scan_header_stamp_for_tfs:
            wait_iters = 0
            while self.new_local_map_received == False and wait_iters < 10:
                #insert sleep command
                wait_iters += 1
            if wait_iters >= 10:
                self.get_logger().debug("no new local_map received. Continuing anyways")
            else:
                self.new_local_map_received = False
        now = detected_clusters_msg.header.stamp

        detected_clusters = []
        detected_clusters_set = set()
        for cluster in detected_clusters_msg.legs:
            new_detected_cluster = DetectedCluster(
                cluster.position.x,
                cluster.position.y,
                cluster.confidence,
                in_free_space = self.how_much_in_free_space(cluster.position.x, cluster.position.y)
            )
            if new_detected_cluster.in_free_space < self.in_free_space_threshold:
                new_detected_cluster.in_free_space_bool = True
            else:
                new_detected_cluster.in_free_space_bool = False
            detected_clusters.append(new_detected_cluster)
            detected_clusters_set.add(new_detected_cluster)
        to_duplicate = set()
        # propogated: 伝搬された
        propogated = copy.deepcopy(self.objects_tracked)
        for propogated_track in propogated:
            # masked_array(data=[0, 0],mask=[True, True],fill_value=999999)をupdate関数に渡すことで、観測値なしの更新となる
            propogated_track.update(np.ma.masked_array(np.array([0, 0]), mask=[1,1]))
            if propogated_track.is_person:
                to_duplicate.add(propogated_track)
        # 人々のトラッキング結果をマッチングにしようするため複製する
        duplicates = {}
        for propogated_track in to_duplicate:
            propogated.append(copy.deepcopy(propogated_track))
            duplicates[propogated_track] = propogated[-1]
        # propogated: object_tracked, detected_clusters: object_detectedをそれぞれ紐付ける
        matched_tracks = self.match_detections_to_tracks_GNN(propogated, detected_clusters)

        # ローカルマップでしようするための非人クラスタ情報をPublishする
        non_legs_msg = LegArray()
        non_legs_msg.header = detected_clusters_msg.header
        leg_clusters = set()
        for track, detect in matched_tracks.items():
            if track.is_person:
                leg_clusters.add(detect)
        # track.is_person以外の要素をnon_leg_clustersとする
        non_leg_clusters = detected_clusters_set.difference(leg_clusters)
        for detect in non_leg_clusters:
            non_leg = Leg()
            non_leg.position.x = detect.pos_x
            non_leg.position.y = detect.pos_y
            non_leg.position.z = 0.0
            non_leg.confidence = 1.0
            non_legs_msg.legs.append(non_leg)
        self.non_leg_clusters_pub.publish(non_legs_msg)

        # 観測結果をもとにすべてのトラッキングされた情報を更新する
        tracks_to_delete = set()
        for idx, track in enumerate(self.objects_tracked):
            print(f"track person idx{idx}, person{track.is_person}, idnum{track.id_num}")
            # propogated: object_tracked, detected_clusters: object_detected
            propogated_track = propogated[idx] # Get the corresponding propogated track
            if propogated_track.is_person:
                # matched_tracksの中に同じ2つの値に紐付けられた異なるobject_detectedの値が存在する。これらは右足、左足として認識され、その中心を追跡対象とする
                if propogated_track in matched_tracks and duplicates[propogated_track] in matched_tracks:
                    propogated_track.is_single_leg = False
                    md_1 = matched_tracks[propogated_track]
                    md_2 = matched_tracks[duplicates[propogated_track]]
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., (md_1.confidence+md_2.confidence)/2., (md_1.in_free_space+md_2.in_free_space)/2.)
                # matched_tracksの中に一つの値でしか紐付けられていない場合は、それを片足とみなし、もう一つはtracksを更新した予測値をもとに平均値を取る
                elif propogated_track in matched_tracks:
                    propogated_track.is_single_leg = True
                    md_1 = matched_tracks[propogated_track]
                    md_2 = duplicates[propogated_track]
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., md_1.confidence, md_1.in_free_space)
                # 同様にして、片足しか検出されない場合は、検出された片足と、予測値をもとに平均値を取る
                elif duplicates[propogated_track] in matched_tracks:
                    propogated_track.is_single_leg = True
                    md_1 = matched_tracks[duplicates[propogated_track]]
                    md_2 = propogated_track
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., md_1.confidence, md_1.in_free_space)
                else:
                    # マッチされなかった場合 
                    matched_detection = None
            else:
                if propogated_track in matched_tracks:
                    # 人の足ではないが、マッチが成立した場合
                    matched_detection = matched_tracks[propogated_track]
                else:
                    matched_detection = None
            
            if matched_detection:
                observations = np.array([matched_detection.pos_x, 
                                         matched_detection.pos_y])
                track.in_free_space = 0.8*track.in_free_space + 0.2*matched_detection.in_free_space
                track.confidence = 0.95*track.confidence + 0.05*matched_detection.confidence
                track.times_seen += 1
                track.last_seen = now
                track.seen_in_current_scan = True
            else:
                # マッチが成立しなかった場合は、観測値なしで更新する
                observations = np.ma.masked_array(np.array([0, 0]), mask=[1,1]) 
                track.seen_in_current_scan = False

            # 更新する
            track.update(observations)

            # 追跡対象の信頼度が低ければtrackから削除する、信頼度は対象が自由空間にあれば高くなり、また検出結果とマッチしていたかによって左右される
            if  track.is_person and track.confidence < self.confidence_threshold_to_maintain_track:
                tracks_to_delete.add(track)
                self.logger.debug("deleting due to low confidence")
            else:
                # 共分散行列が大きくなりすぎていれば削除
                cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                # if cov > self.max_cov or track.avg_speed < 0.002:
                if cov > self.max_cov:
                    tracks_to_delete.add(track)
                    # rospy.loginfo("deleting because unseen for %.2f", (now - track.last_seen).to_sec())
        
        # トラッキング結果を削除
        for track in tracks_to_delete:
            track.deleted = True 
            self.objects_tracked.remove(track)

        # 検出結果がマッチしなければ新たにtrackを作成する
        for detect in detected_clusters:
            if not detect in matched_tracks.values():
                self.objects_tracked.append(ObjectTracked(detect.pos_x, detect.pos_y, now, detect.confidence, is_person=False, in_free_space=detect.in_free_space))

        # 新たな足のペア候補を生成する
        for track_1 in self.objects_tracked:
            for track_2 in self.objects_tracked:
                if (track_1 != track_2 
                    and track_1.id_num > track_2.id_num 
                    and (not track_1.is_person or not track_2.is_person) 
                    and (track_1, track_2) not in self.potential_leg_pairs
                ):
                    self.potential_leg_pairs.add((track_1, track_2))
                    self.potential_leg_pair_initial_dist_travelled[(track_1, track_2)] = (track_1.dist_travelled, track_2.dist_travelled)

        potential_leg_pairs_list = list(self.potential_leg_pairs)
        potential_leg_pairs_list.sort(key=lambda tup: (tup[0].id_num, tup[1].id_num))

        # ペア候補から削除すべきペアを計算する
        leg_pairs_to_delete = set()
        for track_1, track_2 in potential_leg_pairs_list:
            # 2つの足が離れすぎていないか、すでにペアとして登録されていないか、削除されるものとして登録されていないか、ある足がしばらく検出されていなかったかどうかなどに基づいて削除する
            dist = ((track_1.pos_x - track_2.pos_x)**2 + (track_1.pos_y - track_2.pos_y)**2)**(1./2.)
            if (dist > self.max_leg_pairing_dist 
                or track_1.deleted or track_2.deleted
                or (track_1.is_person and track_2.is_person) 
                or track_1.confidence < self.confidence_threshold_to_maintain_track 
                or track_2.confidence < self.confidence_threshold_to_maintain_track
                or (track_1.is_static and track_2.is_static)  # 静的障害物のチェックを追加
                ):
                    leg_pairs_to_delete.add((track_1, track_2))
                    continue
            
            # ペアを生成するか判断する
            # 現在のscanデータにクラスタが含まれるかどうか
            if track_1.seen_in_current_scan and track_2.seen_in_current_scan:
                track_1_initial_dist, track_2_initial_dist = self.potential_leg_pair_initial_dist_travelled[(track_1, track_2)]
                dist_travelled = min(track_1.dist_travelled - track_1_initial_dist, track_2.dist_travelled - track_2_initial_dist)
                # 2つのトラックでペアが形成されてからの移動距離がしきい値以上であれば人の足としてみなす
                # free-spaceにあれば人の足としてみなす
                if (dist_travelled > self.dist_travelled_together_to_initiate_leg_pair 
                    and (track_1.in_free_space < self.in_free_space_threshold or track_2.in_free_space < self.in_free_space_threshold)
                    ):
                        if not track_1.is_person  and not track_2.is_person     :
                            self.objects_tracked.append(
                                ObjectTracked(
                                    (track_1.pos_x+track_2.pos_x)/2.,
                                    (track_1.pos_y+track_2.pos_y)/2., now,
                                    (track_1.confidence+track_2.confidence)/2.,
                                    is_person=True,
                                    in_free_space=0.)
                                )
                            track_1.deleted = True
                            track_2.deleted = True
                            self.objects_tracked.remove(track_1)
                            self.objects_tracked.remove(track_2)
                        elif track_1.is_person:
                            # track_1が人として検出され、track_2は人ではない場合、track_2のみ追跡対象から削除し、track_1は引き続き追跡し、次の検出サイクルで適切に足として見つかることを期待する
                            track_2.deleted = True
                            self.objects_tracked.remove(track_2)
                        else: # track_2.is_person:
                            track_1.deleted = True
                            self.objects_tracked.remove(track_1)
                        leg_pairs_to_delete.add((track_1, track_2)) 

        # 足候補のペアを削除する
        for leg_pair in leg_pairs_to_delete:
            self.potential_leg_pairs.remove(leg_pair)

        self.publish_tracked_objects(now)
        self.publish_tracked_people(now)

    def publish_tracked_objects(self, now):
        if self.use_scan_header_stamp_for_tfs:
            tf_time = now
            try:
                transform1 = self.buffer.lookup_transform(self.publish_people_frame, self.fixed_frame, tf_time, rclpy.duration(1.0))
                transform_available = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform_available = False
        else :
            tf_time = self.get_clock().now().to_msg()
            transform_available = self.buffer.can_transform(self.publish_people_frame, self.fixed_frame, tf_time)

        marker_id = 0
        markers = MarkerArray()
        ns = "objects_tracked"
        frame_id = self.publish_people_frame
        
        if not transform_available:
            self.get_logger().debug("Person tracker: tf not avaiable. Not publishing people")
        else:
            for track in self.objects_tracked:
                if track.is_static:
                    continue
                # 人でない場合は、円柱マーカは表示しない
                #if track.is_person:
                #    continue

                if self.publish_occluded or track.seen_in_current_scan:  
                    ps = PointStamped()
                    ps.header.frame_id = self.fixed_frame
                    ps.header.stamp = tf_time
                    ps.point.x = track.pos_x
                    ps.point.y = track.pos_y
                    try:
                        ps = self.buffer.transform(ps, self.publish_people_frame)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

                    # Publish Rviz Marker
                    color = [0.0, 0.0, 0.0, 1.0]
                    if track.in_free_space < self.in_free_space_threshold:
                        color[0] = track.color[0]
                        color[1] = track.color[1]
                        color[2] = track.color[2]
                    position = [track.pos_x, track.pos_y, 0.15]
                    scale = [0.05, 0.05, 0.10]
                    marker = self.create_marker(marker_id, frame_id, ns, Marker.CYLINDER, position, scale, color, now)
                    markers.markers.append(marker)
                    marker_id += 1

            self.marker_array_pub.publish(markers)
                    

    """
    Publish markers of tracked people to Rviz and to <people_tracked> topic
    """ 
    def publish_tracked_people(self, now):
        people_tracked_msg = PersonArray()
        people_tracked_msg.header.stamp = now
        people_tracked_msg.header.frame_id = self.publish_people_frame
        marker_id = 0
        is_same_id_flag = False

        # Make sure we can get the required transform first:
        if self.use_scan_header_stamp_for_tfs:
            tf_time = now
            try:
                transform = self.buffer.lookup_transform(self.fixed_frame, self.publish_people_frame, tf_time, rclpy.time.Duration(1.0))
                transform_available = True
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform_available = False
        else:
            tf_time = self.get_clock().now()
            transform_available = self.buffer.can_transform(self.fixed_frame, self.publish_people_frame, tf_time)

        marker_id = 0
        highest_score = -float('inf')
        best_person = None
        highest_score = 0
        is_person  = False

        if not transform_available:
            self.get_logger().debug("Person tracker: tf not avaiable. Not publishing people")
        else :
            for person in self.objects_tracked:
                if person.is_person == True:
                    if self.publish_occluded or person.seen_in_current_scan: # Only publish people who have been seen in current scan, unless we want to publish occluded people
                        # Get position in the <self.publish_people_frame> frame 
                        is_person = True
                        ps = PointStamped()
                        ps.header.frame_id = self.fixed_frame
                        ps.header.stamp = tf_time.to_msg()
                        ps.point.x = person.pos_x
                        ps.point.y = person.pos_y
                        try:
                            ps = self.buffer.transform(ps, self.publish_people_frame)
                        except:
                            self.get_logger().error("Not publishing people due to no transform from fixed_frame-->publish_people_frame")
                            continue

                        # pulish to people_tracked topic
                        new_person = Person()
                        new_person.pose.position.x = ps.point.x
                        new_person.pose.position.y = ps.point.y 
                        yaw = math.atan2(person.vel_y, person.vel_x)
                        quaternion = self.ToQuaternion(yaw, 0, 0)
                        new_person.pose.orientation.x = quaternion.x
                        new_person.pose.orientation.y = quaternion.y
                        new_person.pose.orientation.z = quaternion.z
                        new_person.pose.orientation.w = quaternion.w
                        new_person.id = person.id_num
                        new_person.covariance = person.filtered_state_covariances[0][0]
                        new_person.var_obs = person.var_obs
                        people_tracked_msg.people.append(new_person)
                        
                        # 人らしさのスコアを計算
                        speed = math.sqrt(person.vel_x ** 2 + person.vel_y ** 2)

                        # スコアの初期値は信頼度に基づく
                        score = person.confidence
                        self.logger.debug(f"first score: {score}")
                        
                        # 片足しか検出されない場合はべナルティ
                        if person.is_single_leg:
                            score += LegTrackerParam.single_leg_penalty
                            self.logger.debug(f"single leg penalty applied: {score}")
                        
                        # 静的である場合はペナルティ
                        if  person.is_static:
                            score += LegTrackerParam.static_penalty 
                            self.logger.debug(f"is static score  {score}")
                        
                        # 同じIDの場合、大きくスコアを加算する
                        if self.prev_person_id == new_person.id:
                            score += LegTrackerParam.id_weight
                            self.logger.debug(f"same id score  {score}")
                        else:
                            self.logger.debug(f"change id")

                        # 距離に基づくペナルティ
                        distance = math.sqrt(math.pow(ps.point.x, 2) + math.pow(ps.point.y, 2))
                        score -= distance * LegTrackerParam.distance_weight
                        self.logger.debug(f"distance score: {score}")
                        
                        # 速度に基づくスコア加算
                        score += speed * LegTrackerParam.speed_weight
                        self.logger.debug(f"speed score: {score}")
                        self.logger.debug(f"\033[91mfinally score: {score}\033[0m")

                        # 最高スコアの人物を選択
                        if score > highest_score:
                            highest_score = score
                            best_person = new_person
            
            # if best_person:
                        target_person = new_person
                        self.logger.debug(f"\033[94mFollowing the best-scored person id: {target_person.id}, pos_x: {target_person.pose.position.x}, pos_y: {target_person.pose.position.y}, prev_person_id = {self.prev_person_id}, score: {highest_score}\033[0m")

                        self.follow_target_person_pub.publish(target_person)
                        self.prev_person_id = target_person.id

                        # publish rviz markers
                        ns = "follow_target_person_marker"
                        frame_id = self.publish_people_frame
                        markers = MarkerArray()
                        # Cylinder for body
                        body_marker = self.create_marker(marker_id, frame_id, ns, Marker.CYLINDER,
                                                        [target_person.pose.position.x, target_person.pose.position.y, 0.8],
                                                        [0.2, 0.2, 1.2], [0.0, 0.39, 0.0, 1.0], now)
                        markers.markers.append(body_marker)
                        marker_id += 1
                        
                        # Sphere for head shape
                        head_marker = self.create_marker(marker_id, frame_id, ns, Marker.SPHERE,
                                                        [target_person.pose.position.x, target_person.pose.position.y, 1.5],
                                                        [0.2, 0.2, 0.2], [0.0, 0.39, 0.0, 1.0], now)
                        markers.markers.append(head_marker)
                        marker_id += 1
                        
                        # Text showing person's ID number
                        text_marker = self.create_marker(marker_id, frame_id, ns, Marker.TEXT_VIEW_FACING,
                                                        [target_person.pose.position.x, target_person.pose.position.y, 1.7],
                                                        [0.0, 0.0, 0.2], [1.0, 1.0, 1.0, 1.0], now, text=str(target_person.id)) 
                        markers.markers.append(text_marker)
                        marker_id += 1
                        
                        # <self.confidence_percentile>% confidence bounds of person's position as an ellipse:
                        cov = target_person.covariance + target_person.var_obs # cov_xx == cov_yy == cov
                        std = cov**(1./2.)
                        gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, std)
                        confidence_marker = self.create_marker(marker_id, frame_id, ns, Marker.SPHERE,
                                                        [target_person.pose.position.x, target_person.pose.position.y, 0.0],
                                                        [2*gate_dist_euclid, 2*gate_dist_euclid, 0.01], [0.0, 0.39, 0.0, 0.2], now)
                        markers.markers.append(confidence_marker)
                        marker_id += 1
                        self.marker_array_pub.publish(markers)
                
                        # Publish people tracked message
                        self.people_tracked_pub.publish(people_tracked_msg)                    

    def create_marker(self, marker_id, frame_id, ns, marker_type, position, scale, color, now, text=None):
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        if text:
            marker.text = text
        return marker

    def ToQuaternion (self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = 0.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = KalmanMultiTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
