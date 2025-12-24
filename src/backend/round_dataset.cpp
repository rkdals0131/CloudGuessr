#include "cloudguessr/backend/round_dataset.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;

namespace cloudguessr {
namespace round_dataset {

RoundMetadata loadRoundMetadata(const std::string& yaml_path) {
  RoundMetadata meta;
  
  try {
    if (!fs::exists(yaml_path)) {
      std::cerr << "[round_dataset] File not found: " << yaml_path << std::endl;
      return meta;
    }

    YAML::Node config = YAML::LoadFile(yaml_path);
    
    if (config["round_id"]) {
      meta.round_id = config["round_id"].as<int>();
    }
    
    if (config["gt_pose_in_map"]) {
      auto gt = config["gt_pose_in_map"];
      if (gt["x"]) meta.gt_x = gt["x"].as<double>();
      if (gt["y"]) meta.gt_y = gt["y"].as<double>();
      if (gt["z"]) meta.gt_z = gt["z"].as<double>();
    }
    
    if (config["roi_radius"]) {
      meta.roi_radius = config["roi_radius"].as<double>();
    }
    
    if (config["difficulty"]) {
      meta.difficulty = config["difficulty"].as<std::string>();
    }
    
    if (config["notes"]) {
      meta.notes = config["notes"].as<std::string>();
    }
    
  } catch (const std::exception& e) {
    std::cerr << "[round_dataset] Failed to parse " << yaml_path << ": " << e.what() << std::endl;
  }
  
  return meta;
}

std::vector<std::string> scanRoundsDirectory(const std::string& rounds_dir) {
  std::vector<std::string> round_dirs;
  
  try {
    if (!fs::exists(rounds_dir) || !fs::is_directory(rounds_dir)) {
      return round_dirs;
    }
    
    for (const auto& entry : fs::directory_iterator(rounds_dir)) {
      if (entry.is_directory()) {
        std::string dir_name = entry.path().filename().string();
        // Check if it looks like a round directory (contains round.yaml)
        if (fs::exists(entry.path() / "round.yaml")) {
          round_dirs.push_back(entry.path().string());
        }
      }
    }
    
    // Sort by name
    std::sort(round_dirs.begin(), round_dirs.end());
    
  } catch (const std::exception& e) {
    std::cerr << "[round_dataset] Error scanning " << rounds_dir << ": " << e.what() << std::endl;
  }
  
  return round_dirs;
}

std::string getQueryPath(const std::string& round_dir) {
  // Try .pcd first, then .ply
  std::string pcd_path = (fs::path(round_dir) / "query.pcd").string();
  std::string ply_path = (fs::path(round_dir) / "query.ply").string();
  
  if (fs::exists(pcd_path)) {
    return pcd_path;
  } else if (fs::exists(ply_path)) {
    return ply_path;
  }
  
  return "";
}

std::string getRoundYamlPath(const std::string& round_dir) {
  return (fs::path(round_dir) / "round.yaml").string();
}

}  // namespace round_dataset
}  // namespace cloudguessr
