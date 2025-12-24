#pragma once

#include "cloudguessr/backend/types.hpp"
#include <string>
#include <vector>

namespace cloudguessr {
namespace round_dataset {

/**
 * @brief Load round metadata from round.yaml
 * @param yaml_path Path to round.yaml
 * @return RoundMetadata (round_id=0 if failed)
 */
RoundMetadata loadRoundMetadata(const std::string& yaml_path);

/**
 * @brief Scan rounds directory and get list of round directories
 * @param rounds_dir Path to rounds directory
 * @return List of round directory paths (sorted)
 */
std::vector<std::string> scanRoundsDirectory(const std::string& rounds_dir);

/**
 * @brief Get query cloud path for a round
 * @param round_dir Round directory path
 * @return Path to query.pcd or query.ply
 */
std::string getQueryPath(const std::string& round_dir);

/**
 * @brief Get round.yaml path for a round
 * @param round_dir Round directory path
 * @return Path to round.yaml
 */
std::string getRoundYamlPath(const std::string& round_dir);

}  // namespace round_dataset
}  // namespace cloudguessr
