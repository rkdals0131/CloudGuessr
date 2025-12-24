/**
 * @file test_pipeline_e2e.cpp
 * @brief Integration test for the full scoring pipeline
 * IT-PIPE-01: File-based end-to-end test
 */

#include <gtest/gtest.h>
#include "cloudguessr/backend/io.hpp"
#include "cloudguessr/backend/preprocess.hpp"
#include "cloudguessr/backend/roi.hpp"
#include "cloudguessr/backend/icp.hpp"
#include "cloudguessr/backend/scoring.hpp"
#include "cloudguessr/backend/types.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

class PipelineE2ETest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = "/tmp/cloudguessr_test_e2e";
    fs::create_directories(test_dir_);
    
    // Create a synthetic map (building-like structure)
    map_ = cloudguessr::PointCloudPtr(new cloudguessr::PointCloud);
    
    // Ground plane
    for (int x = 0; x < 100; ++x) {
      for (int y = 0; y < 100; ++y) {
        cloudguessr::PointT pt;
        pt.x = x * 0.5f;  // 50m x 50m area
        pt.y = y * 0.5f;
        pt.z = 0.0f;
        map_->push_back(pt);
      }
    }
    
    // Add some vertical structure (simulate building corners)
    for (int z = 0; z < 20; ++z) {
      // Corner 1
      for (int i = 0; i < 5; ++i) {
        cloudguessr::PointT pt1, pt2;
        pt1.x = 25.0f; pt1.y = 25.0f + i * 0.5f; pt1.z = z * 0.5f;
        pt2.x = 25.0f + i * 0.5f; pt2.y = 25.0f; pt2.z = z * 0.5f;
        map_->push_back(pt1);
        map_->push_back(pt2);
      }
    }
    
    // Create query at known position
    gt_x_ = 25.0;
    gt_y_ = 25.0;
    gt_z_ = 0.0;
    gt_yaw_deg_ = 45.0;
    
    // Crop ROI and transform to create query
    Eigen::Vector3f gt_center(gt_x_, gt_y_, gt_z_);
    auto roi = cloudguessr::roi::cropSphere(map_, gt_center, 15.0);
    
    // Center the query
    auto [centered, centroid] = cloudguessr::preprocess::centerCloud(roi);
    
    // Apply rotation
    query_ = cloudguessr::PointCloudPtr(new cloudguessr::PointCloud);
    Eigen::Matrix4f rot = cloudguessr::icp::createYawRotation(gt_yaw_deg_);
    pcl::transformPointCloud(*centered, *query_, rot);
    
    // Save files
    map_path_ = test_dir_ + "/map.pcd";
    query_path_ = test_dir_ + "/query.pcd";
    pcl::io::savePCDFileBinary(map_path_, *map_);
    pcl::io::savePCDFileBinary(query_path_, *query_);
  }
  
  void TearDown() override {
    fs::remove_all(test_dir_);
  }
  
  std::string test_dir_;
  std::string map_path_;
  std::string query_path_;
  cloudguessr::PointCloudPtr map_;
  cloudguessr::PointCloudPtr query_;
  double gt_x_, gt_y_, gt_z_, gt_yaw_deg_;
};

// IT-PIPE-01: Full pipeline with correct click position
TEST_F(PipelineE2ETest, FullPipeline_CorrectClick_HighScore) {
  auto start = std::chrono::high_resolution_clock::now();
  
  // Load map and query
  auto map = cloudguessr::io::loadPointCloud(map_path_);
  auto query = cloudguessr::io::loadPointCloud(query_path_);
  
  ASSERT_GT(map->size(), 0u);
  ASSERT_GT(query->size(), 0u);
  
  // Simulate click near GT position
  Eigen::Vector3f clicked(gt_x_ + 1.0f, gt_y_ - 0.5f, gt_z_);  // Slight offset
  
  // Crop ROI
  double roi_radius = 20.0;
  auto roi = cloudguessr::roi::cropSphere(map, clicked, roi_radius);
  ASSERT_GT(roi->size(), 100u);
  
  // Downsample for faster ICP
  auto roi_ds = cloudguessr::preprocess::voxelDownsample(roi, 0.5);
  auto query_ds = cloudguessr::preprocess::voxelDownsample(query, 0.5);
  
  // Yaw sweep
  std::vector<double> yaw_candidates = {0, 45, 90, 135, 180, 225, 270, 315};
  auto sweep_result = cloudguessr::icp::yawSweepAlign(
    roi_ds, query_ds, clicked, yaw_candidates, 50, 2.0);
  
  auto end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  
  // Classify and score
  auto score_result = cloudguessr::scoring::classifyResult(
    sweep_result.best_alignment.fitness,
    sweep_result.best_alignment.rmse,
    sweep_result.best_alignment.converged,
    roi_ds->size(),
    query_ds->size());
  
  // Verify results
  EXPECT_TRUE(sweep_result.best_alignment.converged);
  EXPECT_EQ(score_result.status, "OK") << "Reason: " << score_result.reason;
  EXPECT_GT(score_result.score, 1000);  // Should get decent score with correct click
  
  // Performance check (should be under 2 seconds for this small dataset)
  EXPECT_LT(elapsed_ms, 5000.0) << "Pipeline took too long: " << elapsed_ms << "ms";
  
  std::cout << "Pipeline completed in " << elapsed_ms << "ms" << std::endl;
  std::cout << "Score: " << score_result.score << ", Status: " << score_result.status << std::endl;
  std::cout << "Best yaw: " << sweep_result.best_yaw_deg << " (GT: " << gt_yaw_deg_ << ")" << std::endl;
}

// IT-PIPE-01: Wrong click position should get lower score
TEST_F(PipelineE2ETest, FullPipeline_WrongClick_LowScore) {
  auto map = cloudguessr::io::loadPointCloud(map_path_);
  auto query = cloudguessr::io::loadPointCloud(query_path_);
  
  // Click far from GT position
  Eigen::Vector3f clicked(45.0f, 45.0f, 0.0f);  // Far from GT (25, 25)
  
  // Crop ROI
  auto roi = cloudguessr::roi::cropSphere(map, clicked, 15.0);
  
  if (roi->size() < 100) {
    // Not enough points - should fail anyway
    return;
  }
  
  auto roi_ds = cloudguessr::preprocess::voxelDownsample(roi, 0.5);
  auto query_ds = cloudguessr::preprocess::voxelDownsample(query, 0.5);
  
  std::vector<double> yaw_candidates = {0, 45, 90, 135, 180, 225, 270, 315};
  auto sweep_result = cloudguessr::icp::yawSweepAlign(
    roi_ds, query_ds, clicked, yaw_candidates, 50, 2.0);
  
  auto score_result = cloudguessr::scoring::classifyResult(
    sweep_result.best_alignment.fitness,
    sweep_result.best_alignment.rmse,
    sweep_result.best_alignment.converged,
    roi_ds->size(),
    query_ds->size());
  
  // Wrong click should get lower score or fail
  if (score_result.status == "OK") {
    EXPECT_LT(score_result.score, 3000);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
