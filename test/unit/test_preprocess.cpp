/**
 * @file test_preprocess.cpp
 * @brief Unit tests for cloudguessr::preprocess module
 * UT-PRE-01: Voxel downsample produces fewer points (monotonic decrease)
 */

#include <gtest/gtest.h>
#include "cloudguessr/backend/preprocess.hpp"
#include "cloudguessr/backend/types.hpp"
#include <pcl/common/centroid.h>

class PreprocessTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a dense grid of points
    grid_cloud_ = cloudguessr::PointCloudPtr(new cloudguessr::PointCloud);
    
    // 10x10x10 grid with 0.1m spacing = 1000 points
    for (int x = 0; x < 10; ++x) {
      for (int y = 0; y < 10; ++y) {
        for (int z = 0; z < 10; ++z) {
          cloudguessr::PointT pt;
          pt.x = x * 0.1f;
          pt.y = y * 0.1f;
          pt.z = z * 0.1f;
          grid_cloud_->push_back(pt);
        }
      }
    }
  }
  
  cloudguessr::PointCloudPtr grid_cloud_;
};

// UT-PRE-01: Voxel downsample reduces point count
TEST_F(PreprocessTest, VoxelDownsample_ReducesPoints) {
  size_t original_size = grid_cloud_->size();
  
  // Downsample with voxel size larger than grid spacing
  auto downsampled = cloudguessr::preprocess::voxelDownsample(grid_cloud_, 0.2);
  
  ASSERT_NE(downsampled, nullptr);
  EXPECT_LT(downsampled->size(), original_size);
  EXPECT_GT(downsampled->size(), 0u);
}

// UT-PRE-01: Larger voxel size -> fewer points (monotonic)
TEST_F(PreprocessTest, VoxelDownsample_MonotonicDecrease) {
  auto ds_small = cloudguessr::preprocess::voxelDownsample(grid_cloud_, 0.15);
  auto ds_medium = cloudguessr::preprocess::voxelDownsample(grid_cloud_, 0.3);
  auto ds_large = cloudguessr::preprocess::voxelDownsample(grid_cloud_, 0.5);
  
  // Larger voxel -> fewer points
  EXPECT_GE(ds_small->size(), ds_medium->size());
  EXPECT_GE(ds_medium->size(), ds_large->size());
}

// Test empty cloud handling
TEST_F(PreprocessTest, VoxelDownsample_EmptyCloud) {
  cloudguessr::PointCloudPtr empty(new cloudguessr::PointCloud);
  auto result = cloudguessr::preprocess::voxelDownsample(empty, 0.1);
  
  EXPECT_TRUE(result->empty());
}

// Test invalid voxel size
TEST_F(PreprocessTest, VoxelDownsample_InvalidVoxelSize) {
  auto result = cloudguessr::preprocess::voxelDownsample(grid_cloud_, 0.0);
  EXPECT_TRUE(result->empty());
  
  result = cloudguessr::preprocess::voxelDownsample(grid_cloud_, -1.0);
  EXPECT_TRUE(result->empty());
}

// Test centerCloud
TEST_F(PreprocessTest, CenterCloud_CentroidAtOrigin) {
  auto [centered, original_centroid] = cloudguessr::preprocess::centerCloud(grid_cloud_);
  
  ASSERT_NE(centered, nullptr);
  EXPECT_EQ(centered->size(), grid_cloud_->size());
  
  // Calculate new centroid - should be near origin
  Eigen::Vector4f new_centroid;
  pcl::compute3DCentroid(*centered, new_centroid);
  
  EXPECT_NEAR(new_centroid.x(), 0.0f, 1e-5);
  EXPECT_NEAR(new_centroid.y(), 0.0f, 1e-5);
  EXPECT_NEAR(new_centroid.z(), 0.0f, 1e-5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
