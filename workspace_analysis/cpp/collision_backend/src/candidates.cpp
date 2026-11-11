#include "engine.hpp"
#include <moveit_setup_srdf_plugins/compute_default_collisions.hpp>
#include <stdexcept>
#include <thread>
#include <algorithm>
namespace cable {
J Engine::candidates(const J& in) {
  if(!scene_ || !in.at("trials").is_number_integer())throw std::runtime_error("candidate trials required");
  int requested=in.at("trials").get<int>();
  const unsigned int threads=std::max(1u,std::thread::hardware_concurrency());
  const unsigned int minimum=100u*threads;
  if(minimum>100000u)throw std::runtime_error("candidate thread count exceeds bounded diagnostic budget");
  int trials=requested==0?int(minimum):requested;
  // Humble 2.5.9 divides by floor(per-thread trials/100) while reporting
  // progress. Reject undersized requests before entering that library code.
  if(trials<int(minimum) || trials>100000)throw std::runtime_error("candidate trials must be 0(auto) or >=100*hardware_threads and <=100000");
  auto isolated=scene_->diff();isolated->decoupleParent();isolated->getAllowedCollisionMatrixNonConst().clear();
  unsigned int progress=0;
  auto candidates=moveit_setup::srdf_setup::computeDefaultCollisions(isolated,&progress,true,trials,.95,false);
  J rows=J::array();for(const auto& [p,d]:candidates)rows.push_back({{"links",{p.first,p.second}},{"candidate_class",moveit_setup::srdf_setup::disabledReasonToString(d.reason)},{"generator_disable_recommendation",d.disable_check},{"applied",false},{"review_status","UNREVIEWED"}});
  return {{"identity",identity_},{"backend_status","CANDIDATES"},{"method","MoveIt Setup Assistant computeDefaultCollisions"},{"trials",trials},{"requested_trials",requested},{"hardware_threads",threads},{"minimum_trials",minimum},{"always_fraction",.95},{"random_seed_controlled",false},{"finite_sampling_is_safety_proof",false},{"candidate_pairs",rows},{"semantic_acm_modified",false}};
}
}
