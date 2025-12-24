/**
 * @file test_round_dataset.cpp
 * @brief Unit tests for cloudguessr::round_dataset module
 */

#include <gtest/gtest.h>
#include "cloudguessr/backend/round_dataset.hpp"
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

class RoundDatasetTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temporary test directory structure
    test_dir_ = "/tmp/cloudguessr_test_rounds";
    fs::create_directories(test_dir_);
    
    // Create round_0001
    std::string round1_dir = test_dir_ + "/round_0001";
    fs::create_directories(round1_dir);
    
    // Write round.yaml
    std::ofstream yaml1(round1_dir + "/round.yaml");
    yaml1 << "round_id: 1\n";
    yaml1 << "gt_pose_in_map:\n";
    yaml1 << "  x: 100.5\n";
    yaml1 << "  y: 200.3\n";
    yaml1 << "  z: 0.0\n";
    yaml1 << "roi_radius: 25.0\n";
    yaml1 << "difficulty: easy\n";
    yaml1 << "notes: Test round 1\n";
    yaml1.close();
    
    // Create empty query.pcd (just for path testing)
    std::ofstream pcd1(round1_dir + "/query.pcd");
    pcd1 << "# PCD placeholder\n";
    pcd1.close();
    
    // Create round_0002
    std::string round2_dir = test_dir_ + "/round_0002";
    fs::create_directories(round2_dir);
    
    std::ofstream yaml2(round2_dir + "/round.yaml");
    yaml2 << "round_id: 2\n";
    yaml2 << "gt_pose_in_map:\n";
    yaml2 << "  x: 50.0\n";
    yaml2 << "  y: 75.0\n";
    yaml2 << "  z: 1.0\n";
    yaml2 << "difficulty: hard\n";
    yaml2.close();
    
    std::ofstream ply2(round2_dir + "/query.ply");
    ply2 << "# PLY placeholder\n";
    ply2.close();
  }
  
  void TearDown() override {
    fs::remove_all(test_dir_);
  }
  
  std::string test_dir_;
};

// Test loading round metadata
TEST_F(RoundDatasetTest, LoadRoundMetadata_AllFields) {
  std::string yaml_path = test_dir_ + "/round_0001/round.yaml";
  auto meta = cloudguessr::round_dataset::loadRoundMetadata(yaml_path);
  
  EXPECT_EQ(meta.round_id, 1);
  EXPECT_NEAR(meta.gt_x, 100.5, 1e-6);
  EXPECT_NEAR(meta.gt_y, 200.3, 1e-6);
  EXPECT_NEAR(meta.gt_z, 0.0, 1e-6);
  EXPECT_NEAR(meta.roi_radius, 25.0, 1e-6);
  EXPECT_EQ(meta.difficulty, "easy");
  EXPECT_EQ(meta.notes, "Test round 1");
}

// Test loading partial metadata (missing optional fields)
TEST_F(RoundDatasetTest, LoadRoundMetadata_PartialFields) {
  std::string yaml_path = test_dir_ + "/round_0002/round.yaml";
  auto meta = cloudguessr::round_dataset::loadRoundMetadata(yaml_path);
  
  EXPECT_EQ(meta.round_id, 2);
  EXPECT_NEAR(meta.gt_x, 50.0, 1e-6);
  EXPECT_EQ(meta.difficulty, "hard");
  // Default roi_radius
  EXPECT_NEAR(meta.roi_radius, 20.0, 1e-6);
}

// Test loading non-existent file
TEST_F(RoundDatasetTest, LoadRoundMetadata_NonExistent) {
  auto meta = cloudguessr::round_dataset::loadRoundMetadata("/nonexistent.yaml");
  
  // Returns default-initialized metadata
  EXPECT_EQ(meta.round_id, 0);
}

// Test scanning rounds directory
TEST_F(RoundDatasetTest, ScanRoundsDirectory_FindsRounds) {
  auto rounds = cloudguessr::round_dataset::scanRoundsDirectory(test_dir_);
  
  EXPECT_EQ(rounds.size(), 2u);
}

// Test scanning non-existent directory
TEST_F(RoundDatasetTest, ScanRoundsDirectory_NonExistent) {
  auto rounds = cloudguessr::round_dataset::scanRoundsDirectory("/nonexistent");
  
  EXPECT_TRUE(rounds.empty());
}

// Test getQueryPath for PCD
TEST_F(RoundDatasetTest, GetQueryPath_PCD) {
  std::string round_dir = test_dir_ + "/round_0001";
  std::string query_path = cloudguessr::round_dataset::getQueryPath(round_dir);
  
  EXPECT_NE(query_path.find("query.pcd"), std::string::npos);
  EXPECT_TRUE(fs::exists(query_path));
}

// Test getQueryPath for PLY
TEST_F(RoundDatasetTest, GetQueryPath_PLY) {
  std::string round_dir = test_dir_ + "/round_0002";
  std::string query_path = cloudguessr::round_dataset::getQueryPath(round_dir);
  
  EXPECT_NE(query_path.find("query.ply"), std::string::npos);
  EXPECT_TRUE(fs::exists(query_path));
}

// Test getRoundYamlPath
TEST_F(RoundDatasetTest, GetRoundYamlPath_Correct) {
  std::string round_dir = test_dir_ + "/round_0001";
  std::string yaml_path = cloudguessr::round_dataset::getRoundYamlPath(round_dir);
  
  EXPECT_NE(yaml_path.find("round.yaml"), std::string::npos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
