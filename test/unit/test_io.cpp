/**
 * @file test_io.cpp
 * @brief Unit tests for cloudguessr::io module
 * UT-IO-01: Load/save point cloud, verify point count and no NaN
 */

#include <gtest/gtest.h>
#include "cloudguessr/backend/io.hpp"
#include "cloudguessr/backend/types.hpp"
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <cmath>

namespace fs = std::filesystem;

class IoTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temporary test directory
    test_dir_ = "/tmp/cloudguessr_test_io";
    fs::create_directories(test_dir_);
    
    // Create a simple test point cloud
    test_cloud_ = cloudguessr::PointCloudPtr(new cloudguessr::PointCloud);
    for (int i = 0; i < 100; ++i) {
      cloudguessr::PointT pt;
      pt.x = static_cast<float>(i % 10);
      pt.y = static_cast<float>(i / 10);
      pt.z = 0.0f;
      test_cloud_->push_back(pt);
    }
    
    // Save test cloud
    test_pcd_path_ = test_dir_ + "/test.pcd";
    pcl::io::savePCDFileBinary(test_pcd_path_, *test_cloud_);
  }
  
  void TearDown() override {
    fs::remove_all(test_dir_);
  }
  
  std::string test_dir_;
  std::string test_pcd_path_;
  cloudguessr::PointCloudPtr test_cloud_;
};

// UT-IO-01: Load PCD file, verify point count matches
TEST_F(IoTest, LoadPCD_PointCountMatches) {
  auto loaded = cloudguessr::io::loadPointCloud(test_pcd_path_);
  
  ASSERT_NE(loaded, nullptr);
  EXPECT_EQ(loaded->size(), test_cloud_->size());
}

// UT-IO-01: Loaded cloud has no NaN points
TEST_F(IoTest, LoadPCD_NoNaNPoints) {
  auto loaded = cloudguessr::io::loadPointCloud(test_pcd_path_);
  
  ASSERT_NE(loaded, nullptr);
  size_t nan_count = cloudguessr::io::countNaNPoints(loaded);
  EXPECT_EQ(nan_count, 0u);
}

// Test loading non-existent file returns empty cloud
TEST_F(IoTest, LoadNonExistent_ReturnsEmpty) {
  auto loaded = cloudguessr::io::loadPointCloud("/nonexistent/path.pcd");
  
  ASSERT_NE(loaded, nullptr);
  EXPECT_TRUE(loaded->empty());
}

// Test save and reload
TEST_F(IoTest, SaveAndReload_PointCountMatches) {
  std::string save_path = test_dir_ + "/saved.pcd";
  
  bool saved = cloudguessr::io::savePointCloud(test_cloud_, save_path);
  EXPECT_TRUE(saved);
  
  auto reloaded = cloudguessr::io::loadPointCloud(save_path);
  EXPECT_EQ(reloaded->size(), test_cloud_->size());
}

// Test removeNaNPoints
TEST_F(IoTest, RemoveNaN_RemovesNaNPoints) {
  // Create cloud with NaN points
  cloudguessr::PointCloudPtr cloud_with_nan(new cloudguessr::PointCloud);
  for (int i = 0; i < 10; ++i) {
    cloudguessr::PointT pt;
    pt.x = static_cast<float>(i);
    pt.y = 0.0f;
    pt.z = 0.0f;
    cloud_with_nan->push_back(pt);
  }
  // Add NaN point (all three coordinates NaN to ensure it's detected)
  cloudguessr::PointT nan_pt;
  nan_pt.x = std::nanf("");
  nan_pt.y = std::nanf("");
  nan_pt.z = std::nanf("");
  cloud_with_nan->push_back(nan_pt);
  
  EXPECT_GE(cloudguessr::io::countNaNPoints(cloud_with_nan), 1u);
  
  auto cleaned = cloudguessr::io::removeNaNPoints(cloud_with_nan);
  // After removing NaN, should have fewer or equal points
  EXPECT_LE(cleaned->size(), cloud_with_nan->size());
  // The cleaned cloud should have no NaN points
  EXPECT_EQ(cloudguessr::io::countNaNPoints(cleaned), 0u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
