#pragma once
#include <nlohmann/json.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_model/model.h>
#include <string>

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
  const J& identity() const { return identity_; }
 private:
  urdf::ModelInterfaceSharedPtr urdf_;
  planning_scene::PlanningScenePtr scene_;
  J identity_, audit_, objects_;
  std::string base_, ee_;
};
}
