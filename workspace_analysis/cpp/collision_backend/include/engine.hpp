#pragma once
#include <nlohmann/json.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <urdf_model/model.h>
#include <string>
#include <set>

namespace cable {
using J = nlohmann::json;
std::string hash(const std::string& bytes);
std::string file_hash(const std::string& path);
Eigen::Isometry3d matrix(const J& value);
J matrix_json(const Eigen::Isometry3d& t);
class Engine {
 public:
  J initialize(const J& request);
  J query(const J& request);
  J set_policy(const J& request);
  const J& identity() const { return identity_; }
 private:
  urdf::ModelInterfaceSharedPtr urdf_;
  planning_scene::PlanningScenePtr scene_;
  J identity_, audit_, objects_, policy_;
  collision_detection::AllowedCollisionMatrix policy_acm_;
  std::vector<collision_detection::FCLGeometryConstPtr> distance_geometry_;
  std::unique_ptr<Engine> refined_;
  J profile_, effective_ready_;
  std::set<std::pair<std::string,std::string>> profile_pairs_, distance_only_pairs_;
  std::string base_, ee_;
};
}
