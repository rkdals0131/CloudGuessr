/**
 * @file round_manager_node.cpp
 * @brief CloudGuessr Round Manager - 라운드 관리, 클릭 수신, 채점
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "cloudguessr/backend/io.hpp"
#include "cloudguessr/backend/preprocess.hpp"
#include "cloudguessr/backend/roi.hpp"
#include "cloudguessr/backend/icp.hpp"
#include "cloudguessr/backend/scoring.hpp"
#include "cloudguessr/backend/round_dataset.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>

using json = nlohmann::json;

enum class GameState
{
  IDLE,
  LOADING,
  WAITING_CLICK,
  SCORING,
  SHOWING_RESULT
};

class RoundManagerNode : public rclcpp::Node
{
public:
  RoundManagerNode()
  : Node("cloudguessr_round_manager"),
    state_(GameState::IDLE),
    current_round_idx_(0),
    last_click_initialized_(false)
  {
    // Parameters
    this->declare_parameter("map_file", "");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("rounds_dir", "");
    this->declare_parameter("roi_radius", 20.0);
    this->declare_parameter("voxel_size", 0.5);
    this->declare_parameter("icp_max_iter", 50);
    this->declare_parameter("icp_max_corr_dist", 2.0);
    this->declare_parameter("fail_min_fitness", 0.3);
    this->declare_parameter("fail_max_rmse", 2.0);
    this->declare_parameter("yaw_candidates_deg",
      std::vector<int64_t>{0, 45, 90, 135, 180, 225, 270, 315});
    this->declare_parameter("use_xy_distance", true);
    this->declare_parameter("click_debounce_ms", 300);
    this->declare_parameter("auto_advance", false);
    this->declare_parameter("result_display_sec", 5.0);

    loadParameters();

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "   CloudGuessr 라운드 매니저 시작");
    RCLCPP_INFO(this->get_logger(), "========================================");

    // Load map for scoring
    RCLCPP_INFO(this->get_logger(), "[초기화] 채점용 맵 로딩: %s", map_file_.c_str());
    score_map_ = cloudguessr::io::loadPointCloud(map_file_);
    if (!score_map_ || score_map_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "[오류] 채점용 맵 로드 실패");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[초기화] 채점용 맵 로드 완료: %zu 포인트", score_map_->size());

    // Scan rounds
    round_dirs_ = cloudguessr::round_dataset::scanRoundsDirectory(rounds_dir_);
    if (round_dirs_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[오류] 라운드를 찾을 수 없음: %s", rounds_dir_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[초기화] 총 %zu개 라운드 발견", round_dirs_.size());
    std::sort(round_dirs_.begin(), round_dirs_.end());

    // Publishers
    score_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cloudguessr/score", 10);
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/cloudguessr/result", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cloudguessr/markers", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/cloudguessr/status", 10);
    hmi_log_pub_ = this->create_publisher<std_msgs::msg::String>("/cloudguessr/hmi_log", 10);

    // Query publisher with transient_local QoS (late subscribers receive last message)
    rclcpp::QoS query_qos(1);
    query_qos.transient_local();
    query_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloudguessr/query", query_qos);

    // Aligned query publisher with transient_local QoS (late subscribers receive last result)
    rclcpp::QoS aligned_query_qos(1);
    aligned_query_qos.transient_local();
    aligned_query_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloudguessr/aligned_query", aligned_query_qos);

    // Subscribers
    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&RoundManagerNode::onClickedPoint, this, std::placeholders::_1));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cloudguessr/command", 10,
      std::bind(&RoundManagerNode::onCommand, this, std::placeholders::_1));

    viewer_ready_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cloudguessr/viewer_ready", 10,
      std::bind(&RoundManagerNode::onViewerReady, this, std::placeholders::_1));

    // Timer for status updates
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RoundManagerNode::publishStatus, this));

    RCLCPP_INFO(this->get_logger(), "========================================");
    hmiLog("CloudGuessr 게임을 시작합니다!");

    // Start first round
    loadRound(0);
  }

private:
  void loadParameters()
  {
    map_file_ = this->get_parameter("map_file").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    rounds_dir_ = this->get_parameter("rounds_dir").as_string();
    roi_radius_ = this->get_parameter("roi_radius").as_double();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    icp_max_iter_ = this->get_parameter("icp_max_iter").as_int();
    icp_max_corr_dist_ = this->get_parameter("icp_max_corr_dist").as_double();
    fail_min_fitness_ = this->get_parameter("fail_min_fitness").as_double();
    fail_max_rmse_ = this->get_parameter("fail_max_rmse").as_double();
    use_xy_distance_ = this->get_parameter("use_xy_distance").as_bool();
    click_debounce_ms_ = this->get_parameter("click_debounce_ms").as_int();
    auto_advance_ = this->get_parameter("auto_advance").as_bool();
    result_display_sec_ = this->get_parameter("result_display_sec").as_double();

    yaw_candidates_.clear();
    auto yaw_candidates_param = this->get_parameter("yaw_candidates_deg").as_integer_array();
    for (auto v : yaw_candidates_param) {
      yaw_candidates_.push_back(static_cast<double>(v));
    }
    if (yaw_candidates_.empty()) {
      yaw_candidates_ = {0, 45, 90, 135, 180, 225, 270, 315};
    }
  }

  void loadRound(size_t idx)
  {
    if (idx >= round_dirs_.size()) {
      RCLCPP_WARN(this->get_logger(), "[안내] 모든 라운드 완료! 처음으로 돌아갑니다.");
      idx = 0;
    }

    state_ = GameState::LOADING;
    current_round_idx_ = idx;

    std::string round_dir = round_dirs_[idx];

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "┌──────────────────────────────────────┐");
    RCLCPP_INFO(this->get_logger(), "│     라운드 %zu/%zu 시작                │",
      idx + 1, round_dirs_.size());
    RCLCPP_INFO(this->get_logger(), "└──────────────────────────────────────┘");
    hmiLog("===== 라운드 " + std::to_string(idx + 1) + "/" + std::to_string(round_dirs_.size()) + " 시작 =====");

    // Load metadata
    std::string yaml_path = cloudguessr::round_dataset::getRoundYamlPath(round_dir);
    current_meta_ = cloudguessr::round_dataset::loadRoundMetadata(yaml_path);

    // Load query (원본, 다운샘플 전)
    std::string query_path = cloudguessr::round_dataset::getQueryPath(round_dir);
    current_query_original_ = cloudguessr::io::loadPointCloud(query_path);

    if (!current_query_original_ || current_query_original_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "[오류] Query 로드 실패: %s", query_path.c_str());
      return;
    }

    // Preprocess query for scoring
    if (voxel_size_ > 0) {
      current_query_ = cloudguessr::preprocess::voxelDownsample(current_query_original_, voxel_size_);
    } else {
      current_query_ = current_query_original_;
    }

    // Difficulty 한국어 변환
    std::string diff_kr = current_meta_.difficulty;
    if (diff_kr == "easy") diff_kr = "쉬움";
    else if (diff_kr == "medium") diff_kr = "보통";
    else if (diff_kr == "hard") diff_kr = "어려움";

    RCLCPP_INFO(this->get_logger(), "[라운드 정보]");
    RCLCPP_INFO(this->get_logger(), "  - 난이도: %s", diff_kr.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Query 포인트: %zu개", current_query_original_->size());
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "▶ Open3D 창에서 Query를 관찰하세요");
    RCLCPP_INFO(this->get_logger(), "▶ RViz에서 맵을 클릭하여 위치를 추측하세요");
    RCLCPP_INFO(this->get_logger(), "  (상단 Publish Point 도구 사용)");
    RCLCPP_INFO(this->get_logger(), "");
    hmiLog("난이도: " + diff_kr + " | Query: " + std::to_string(current_query_original_->size()) + "개 포인트");
    hmiLog("Open3D에서 Query를 관찰하고 RViz에서 위치를 클릭하세요!");

    state_ = GameState::WAITING_CLICK;
    clearMarkers();
    publishStatus();
    publishQuery();
  }

  void publishQuery()
  {
    if (!current_query_original_) return;

    sensor_msgs::msg::PointCloud2 query_msg;
    pcl::toROSMsg(*current_query_original_, query_msg);
    query_msg.header.frame_id = "query:" + std::to_string(current_round_idx_);
    query_msg.header.stamp = this->now();
    query_pub_->publish(query_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "[Query 발행] 라운드 %zu, %zu 포인트",
      current_round_idx_,
      current_query_original_->size());
  }

  void onViewerReady(const std_msgs::msg::String::SharedPtr /*msg*/)
  {
    RCLCPP_INFO(this->get_logger(), "[Viewer Ready] Query 재발행");
    publishQuery();
  }

  void publishAlignedQuery(const Eigen::Matrix4f& transform)
  {
    if (!current_query_original_) return;

    // Apply ICP transform to query cloud
    cloudguessr::PointCloudPtr aligned_cloud(new cloudguessr::PointCloud);
    pcl::transformPointCloud(*current_query_original_, *aligned_cloud, transform);

    // Publish as PointCloud2 (in map frame for RViz visualization)
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned_cloud, aligned_msg);
    aligned_msg.header.frame_id = map_frame_;
    aligned_msg.header.stamp = this->now();
    aligned_query_pub_->publish(aligned_msg);

    RCLCPP_INFO(this->get_logger(), "[시각화] 정합된 Query 발행: %zu 포인트", aligned_cloud->size());
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (state_ == GameState::SHOWING_RESULT) {
      // 결과 표시 중 클릭은 무시 (다음 라운드는 명시적 명령으로만 진행)
      (void)msg;
      RCLCPP_WARN(this->get_logger(), "[대기] 결과 확인 중입니다. next_round 명령으로 다음 라운드를 시작하세요.");
      return;
    }

    if (state_ != GameState::WAITING_CLICK) {
      RCLCPP_WARN(this->get_logger(), "[대기] 아직 준비 중입니다. 잠시 후 다시 클릭하세요.");
      return;
    }

    if (msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[입력 무효] clicked_point frame_id(%s) != map_frame(%s)",
        msg->header.frame_id.c_str(),
        map_frame_.c_str());
      publishFailResult("clicked_point frame_id mismatch", msg->point.x, msg->point.y, msg->point.z);
      return;
    }

    auto now = std::chrono::steady_clock::now();
    if (last_click_initialized_) {
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_click_time_).count();
      if (elapsed_ms < click_debounce_ms_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[디바운스] %lldms 이내 중복 클릭 무시 (임계: %dms)",
          static_cast<long long>(elapsed_ms),
          click_debounce_ms_);
        return;
      }
    }
    last_click_initialized_ = true;
    last_click_time_ = now;

    state_ = GameState::SCORING;

    double clicked_x = msg->point.x;
    double clicked_y = msg->point.y;
    double clicked_z = msg->point.z;

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "────────────────────────────────────────");
    RCLCPP_INFO(this->get_logger(), "[클릭 수신] 위치: (%.1f, %.1f, %.1f)", clicked_x, clicked_y, clicked_z);
    RCLCPP_INFO(this->get_logger(), "[채점 중] ICP 정합 수행 중...");

    char buf[128];
    snprintf(buf, sizeof(buf), "클릭 위치: (%.1f, %.1f, %.1f)", clicked_x, clicked_y, clicked_z);
    hmiLog(buf);
    hmiLog("채점 중... ICP 정합 수행 중...");

    // Run scoring pipeline
    auto start = std::chrono::high_resolution_clock::now();

    // 1. ROI crop
    double radius = current_meta_.roi_radius > 0 ? current_meta_.roi_radius : roi_radius_;
    Eigen::Vector3f center(clicked_x, clicked_y, clicked_z);
    auto roi = cloudguessr::roi::cropSphere(score_map_, center, radius);

    if (!roi || roi->size() < 100) {
      publishFailResult("ROI 영역에 포인트가 부족합니다", clicked_x, clicked_y, clicked_z);
      return;
    }

    // 2. Downsample ROI
    if (voxel_size_ > 0) {
      roi = cloudguessr::preprocess::voxelDownsample(roi, voxel_size_);
    }

    // 3. Yaw sweep ICP
    auto sweep_result = cloudguessr::icp::yawSweepAlign(
      roi, current_query_, center, yaw_candidates_,
      icp_max_iter_, icp_max_corr_dist_);

    auto end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    // 4. Calculate distance error first (primary scoring factor)
    std::vector<double> clicked = {clicked_x, clicked_y, clicked_z};
    std::vector<double> gt = {current_meta_.gt_x, current_meta_.gt_y, current_meta_.gt_z};
    double dist_error = use_xy_distance_
      ? cloudguessr::scoring::calculateDistanceError2D(clicked, gt)
      : cloudguessr::scoring::calculateDistanceError(clicked, gt);

    // 5. Classify result (for FAIL detection) and compute distance-based score
    auto score_result = cloudguessr::scoring::classifyResult(
      sweep_result.best_alignment.fitness,
      sweep_result.best_alignment.rmse,
      sweep_result.best_alignment.converged,
      roi->size(),
      current_query_->size(),
      fail_min_fitness_,
      fail_max_rmse_);

    // Override score with distance-based calculation (if not FAIL)
    if (score_result.status == "OK") {
      score_result.score = cloudguessr::scoring::computeCompositeScore(
        dist_error,
        sweep_result.best_alignment.fitness,
        sweep_result.best_alignment.rmse);
    }

    // 6. Publish aligned query for visualization
    publishAlignedQuery(sweep_result.best_alignment.transform);

    // 7. Publish results
    publishResults(score_result, sweep_result, clicked, gt, dist_error, elapsed_ms);

    state_ = GameState::SHOWING_RESULT;

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "▶ 다음 라운드는 Game Console의 Next Round 버튼으로 진행하세요");
    RCLCPP_INFO(this->get_logger(), "────────────────────────────────────────");

    // Auto advance
    if (auto_advance_) {
      result_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(result_display_sec_),
        [this]() {
          result_timer_->cancel();
          loadRound(current_round_idx_ + 1);
        });
    }
  }

  void publishResults(
    const cloudguessr::ScoreResult & score_result,
    const cloudguessr::YawSweepResult & sweep_result,
    const std::vector<double> & clicked,
    const std::vector<double> & gt,
    double dist_error,
    double elapsed_ms)
  {
    // Score
    std_msgs::msg::Int32 score_msg;
    score_msg.data = score_result.score;
    score_pub_->publish(score_msg);

    // JSON result
    json result_json;
    result_json["round_id"] = current_meta_.round_id;
    result_json["clicked_xyz"] = clicked;
    result_json["gt_xyz"] = gt;
    result_json["dist_error_m"] = dist_error;
    result_json["best_yaw_deg"] = sweep_result.best_yaw_deg;
    result_json["fitness"] = sweep_result.best_alignment.fitness;
    result_json["rmse"] = sweep_result.best_alignment.rmse;
    result_json["score"] = score_result.score;
    result_json["elapsed_ms"] = elapsed_ms;
    result_json["status"] = score_result.status;
    result_json["reason"] = score_result.reason;

    std_msgs::msg::String result_msg;
    result_msg.data = result_json.dump();
    result_pub_->publish(result_msg);

    // Markers
    publishMarkers(clicked, gt);

    // 결과 출력
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════╗");
    RCLCPP_INFO(this->get_logger(), "║           채  점  결  과             ║");
    RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════╣");
    RCLCPP_INFO(this->get_logger(), "║  점수: %5d / 5000                  ║", score_result.score);
    RCLCPP_INFO(this->get_logger(), "║  거리 오차: %6.1f m                  ║", dist_error);
    RCLCPP_INFO(this->get_logger(), "║  정합 품질: %.1f%%                     ║", sweep_result.best_alignment.fitness * 100);
    RCLCPP_INFO(this->get_logger(), "║  처리 시간: %.0f ms                   ║", elapsed_ms);
    RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════╝");

    // HMI 로그 - 채점 결과
    char score_buf[256];
    snprintf(score_buf, sizeof(score_buf),
      "점수: %d점 | 거리 오차: %.1fm | 정합 품질: %.1f%%",
      score_result.score, dist_error, sweep_result.best_alignment.fitness * 100);
    hmiLog(score_buf);

    if (score_result.status != "OK") {
      RCLCPP_WARN(this->get_logger(), "[결과] 실패 - %s", score_result.reason.c_str());
      hmiLog("실패: " + score_result.reason);
    } else if (score_result.score >= 4000) {
      RCLCPP_INFO(this->get_logger(), "[결과] 훌륭합니다! 거의 정확한 위치입니다!");
      hmiLog("훌륭합니다! 거의 정확한 위치입니다!");
    } else if (score_result.score >= 2500) {
      RCLCPP_INFO(this->get_logger(), "[결과] 좋습니다! 꽤 가까운 위치입니다.");
      hmiLog("좋습니다! 꽤 가까운 위치입니다.");
    } else if (score_result.score >= 1000) {
      RCLCPP_INFO(this->get_logger(), "[결과] 아쉽네요. 조금 멀었습니다.");
      hmiLog("아쉽네요. 조금 멀었습니다.");
    } else {
      RCLCPP_INFO(this->get_logger(), "[결과] 많이 멀었네요. 다음 라운드에 도전하세요!");
      hmiLog("많이 멀었네요. 다음 라운드에 도전하세요!");
    }
    hmiLog("다음 라운드는 Game Console의 Next Round 버튼으로 진행하세요.");
  }

  void publishFailResult(const std::string & reason, double x, double y, double z)
  {
    RCLCPP_WARN(this->get_logger(), "[실패] %s", reason.c_str());
    hmiLog("실패: " + reason);

    std_msgs::msg::Int32 score_msg;
    score_msg.data = 0;
    score_pub_->publish(score_msg);

    json result_json;
    result_json["round_id"] = current_meta_.round_id;
    result_json["clicked_xyz"] = {x, y, z};
    result_json["gt_xyz"] = {current_meta_.gt_x, current_meta_.gt_y, current_meta_.gt_z};
    result_json["score"] = 0;
    result_json["status"] = "FAIL";
    result_json["reason"] = reason;

    std_msgs::msg::String result_msg;
    result_msg.data = result_json.dump();
    result_pub_->publish(result_msg);

    state_ = GameState::SHOWING_RESULT;

    RCLCPP_INFO(this->get_logger(), "▶ 다음 라운드는 Game Console의 Next Round 버튼으로 진행하세요");
    hmiLog("다음 라운드는 Game Console의 Next Round 버튼으로 진행하세요.");
  }

  void publishMarkers(const std::vector<double> & clicked, const std::vector<double> & gt)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Clicked position (red)
    visualization_msgs::msg::Marker click_marker;
    click_marker.header.frame_id = map_frame_;
    click_marker.header.stamp = this->now();
    click_marker.ns = "cloudguessr";
    click_marker.id = 0;
    click_marker.type = visualization_msgs::msg::Marker::SPHERE;
    click_marker.action = visualization_msgs::msg::Marker::ADD;
    click_marker.pose.position.x = clicked[0];
    click_marker.pose.position.y = clicked[1];
    click_marker.pose.position.z = clicked[2];
    click_marker.pose.orientation.w = 1.0;
    click_marker.scale.x = click_marker.scale.y = click_marker.scale.z = 3.0;
    click_marker.color.r = 1.0;
    click_marker.color.a = 0.8;
    markers.markers.push_back(click_marker);

    // GT position (green)
    visualization_msgs::msg::Marker gt_marker = click_marker;
    gt_marker.id = 1;
    gt_marker.pose.position.x = gt[0];
    gt_marker.pose.position.y = gt[1];
    gt_marker.pose.position.z = gt[2];
    gt_marker.color.r = 0.0;
    gt_marker.color.g = 1.0;
    markers.markers.push_back(gt_marker);

    // Line between them
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = map_frame_;
    line_marker.header.stamp = this->now();
    line_marker.ns = "cloudguessr";
    line_marker.id = 2;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.5;
    line_marker.color.r = 1.0;
    line_marker.color.g = 1.0;
    line_marker.color.a = 0.6;
    geometry_msgs::msg::Point p1, p2;
    p1.x = clicked[0]; p1.y = clicked[1]; p1.z = clicked[2];
    p2.x = gt[0]; p2.y = gt[1]; p2.z = gt[2];
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    markers.markers.push_back(line_marker);

    marker_pub_->publish(markers);
  }

  void clearMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);
    marker_pub_->publish(markers);
  }

  void onCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & command = msg->data;
    if (command == "next_round") {
      if (state_ == GameState::SCORING) {
        RCLCPP_WARN(this->get_logger(), "[명령] SCORING 중에는 next_round를 처리하지 않습니다.");
        return;
      }
      loadRound(current_round_idx_ + 1);
      return;
    }

    if (command == "reset_round") {
      if (state_ == GameState::SCORING) {
        RCLCPP_WARN(this->get_logger(), "[명령] SCORING 중에는 reset_round를 처리하지 않습니다.");
        return;
      }
      loadRound(current_round_idx_);
      return;
    }

    RCLCPP_WARN(this->get_logger(), "[명령] 알 수 없는 command: %s", command.c_str());
  }

  void publishStatus()
  {
    std_msgs::msg::String status_msg;
    json status;
    status["state"] = static_cast<int>(state_);
    status["round_idx"] = current_round_idx_;
    status["total_rounds"] = round_dirs_.size();
    status["round_id"] = current_meta_.round_id;
    status["difficulty"] = current_meta_.difficulty;
    status["round_notes"] = current_meta_.notes;
    status["query_points"] = current_query_original_ ? current_query_original_->size() : 0;

    std::string state_str;
    switch (state_) {
      case GameState::IDLE: state_str = "IDLE"; break;
      case GameState::LOADING: state_str = "LOADING"; break;
      case GameState::WAITING_CLICK: state_str = "WAITING_CLICK"; break;
      case GameState::SCORING: state_str = "SCORING"; break;
      case GameState::SHOWING_RESULT: state_str = "SHOWING_RESULT"; break;
    }
    status["state_str"] = state_str;

    status_msg.data = status.dump();
    status_pub_->publish(status_msg);
    // Note: Query는 라운드 시작 시 1회만 publish (transient_local QoS로 새 구독자도 수신 가능)
  }

  // Parameters
  std::string map_file_;
  std::string map_frame_;
  std::string rounds_dir_;
  double roi_radius_;
  double voxel_size_;
  int icp_max_iter_;
  double icp_max_corr_dist_;
  double fail_min_fitness_;
  double fail_max_rmse_;
  bool use_xy_distance_;
  int click_debounce_ms_;
  bool auto_advance_;
  double result_display_sec_;

  // State
  GameState state_;
  size_t current_round_idx_;
  cloudguessr::RoundMetadata current_meta_;
  cloudguessr::PointCloudPtr current_query_;
  cloudguessr::PointCloudPtr current_query_original_;  // 다운샘플 전 원본
  cloudguessr::PointCloudPtr score_map_;
  std::vector<std::string> round_dirs_;
  std::vector<double> yaw_candidates_;
  std::chrono::steady_clock::time_point last_click_time_;
  bool last_click_initialized_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr query_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_query_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmi_log_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr viewer_ready_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr result_timer_;

  // HMI 로그 발행 헬퍼
  void hmiLog(const std::string & msg)
  {
    std_msgs::msg::String log_msg;
    log_msg.data = msg;
    hmi_log_pub_->publish(log_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoundManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
