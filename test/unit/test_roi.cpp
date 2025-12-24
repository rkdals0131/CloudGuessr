/**
 * @file test_roi.cpp
 * @brief Unit tests for cloudguessr::roi module
 * UT-ROI-01: cropSphere returns points within radius
 */

#include <gtest/gtest.h>
#include "cloudguessr/backend/roi.hpp"
#include "cloudguessr/backend/types.hpp"
#include <cmath>

class RoiTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a cloud spread over a large area
    // Centered around (50, 50, 0)
    cloud_ = cloudguessr::PointCloudPtr(new cloudguessr::PointCloud);
    
    // Create points in a 100x100 grid
    for (int x = 0; x < 100; ++x) {
      for (int y = 0; y < 100; ++y) {
        cloudguessr::PointT pt;
        pt.x = static_cast<float>(x);
        pt.y = static_cast<float>(y);
        pt.z = 0.0f;
        cloud_->push_back(pt);
      }
    }
  }
  
  cloudguessr::PointCloudPtr cloud_;
};

// UT-ROI-01: All cropped points are within radius
TEST_F(RoiTest, CropSphere_AllPointsWithinRadius) {
  Eigen::Vector3f center(50.0f, 50.0f, 0.0f);
  double radius = 10.0;
  
  auto cropped = cloudguessr::roi::cropSphere(cloud_, center, radius);
  
  ASSERT_NE(cropped, nullptr);
  EXPECT_GT(cropped->size(), 0u);
  
  // Verify all points are within radius
  for (const auto& pt : cropped->points) {
    double dx = pt.x - center.x();
    double dy = pt.y - center.y();
    double dz = pt.z - center.z();
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    EXPECT_LE(dist, radius + 1e-6) << "Point at (" << pt.x << ", " << pt.y << ", " << pt.z 
                                    << ") is " << dist << "m from center";
  }
}

// UT-ROI-01: Cropped point count is reasonable
TEST_F(RoiTest, CropSphere_PointCountInRange) {
  Eigen::Vector3f center(50.0f, 50.0f, 0.0f);
  double radius = 10.0;
  
  auto cropped = cloudguessr::roi::cropSphere(cloud_, center, radius);
  
  // Expected: approximately pi * r^2 points (for 2D grid at z=0)
  // pi * 10^2 ≈ 314 points, but due to grid discretization it will vary
  EXPECT_GT(cropped->size(), 200u);
  EXPECT_LT(cropped->size(), 400u);
}

// Test larger radius includes more points
TEST_F(RoiTest, CropSphere_LargerRadiusMorePoints) {
  Eigen::Vector3f center(50.0f, 50.0f, 0.0f);
  
  auto small = cloudguessr::roi::cropSphere(cloud_, center, 5.0);
  auto large = cloudguessr::roi::cropSphere(cloud_, center, 15.0);
  
  EXPECT_LT(small->size(), large->size());
}

// Test empty cloud
TEST_F(RoiTest, CropSphere_EmptyCloud) {
  cloudguessr::PointCloudPtr empty(new cloudguessr::PointCloud);
  Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
  
  auto result = cloudguessr::roi::cropSphere(empty, center, 10.0);
  EXPECT_TRUE(result->empty());
}

// Test center outside cloud returns empty or few points
TEST_F(RoiTest, CropSphere_CenterOutside) {
  Eigen::Vector3f center(200.0f, 200.0f, 0.0f);  // Far outside
  double radius = 10.0;
  
  auto cropped = cloudguessr::roi::cropSphere(cloud_, center, radius);
  EXPECT_TRUE(cropped->empty());
}

// Test invalid radius
TEST_F(RoiTest, CropSphere_InvalidRadius) {
  Eigen::Vector3f center(50.0f, 50.0f, 0.0f);
  
  auto result = cloudguessr::roi::cropSphere(cloud_, center, 0.0);
  EXPECT_TRUE(result->empty());
  
  result = cloudguessr::roi::cropSphere(cloud_, center, -5.0);
  EXPECT_TRUE(result->empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
