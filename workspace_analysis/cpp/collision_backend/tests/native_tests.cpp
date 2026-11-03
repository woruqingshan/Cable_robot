#include "engine.hpp"
#include <iostream>
#include <stdexcept>
using cable::J;
static void check(bool b,const char* text) { if(!b)throw std::runtime_error(text); }
static J initialization(bool self_overlap=false,double world_x=10) {
  std::string xml=R"(<robot name="fixture"><link name="base_link"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link><link name="tip"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base_link"/><child link="tip"/><origin xyz=")";
  xml+=self_overlap?"0.05 0 0":"2 0 0";
  xml+=R"("/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>)";
  J p={{"position_m",{world_x,0,0}},{"quaternion_xyzw",{0,0,0,1}}};
  J obj={{"id","obstacle"},{"type","box"},{"parent_frame","base_link"},{"pose",p},{"size_m",{.1,.1,.1}}};
  J s={{"scene",{{"reference_frame","base_link"},{"units",{{"length","m"},{"angle","rad"}}},{"frames",J::array()},{"collision_objects",J::array({obj})}}}};
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation().x()=world_x;
  obj.erase("pose");obj.erase("parent_frame");obj["transform"]=cable::matrix_json(t);
  return {{"model",{{"urdf_xml",xml},{"urdf_sha256",cable::hash(xml)},{"base_frame","base_link"},{"ee_frame","tip"},{"joint_order",{"j"}}}},
          {"scene_canonical_json",s.dump()},{"scene_sha256",cable::hash(s.dump())},{"objects",J::array({obj})}};
}
int main() {
  try {
    const J q={{"joint_positions",{{"j",0}}}};
    cable::Engine separated;auto ready=separated.initialize(initialization());
    check(ready["audit"]["detector"]=="FCL","real FCL initialization");
    check(!separated.query(q)["collision_detected"].get<bool>(),"separated fixture");
    // Consecutive configurations of one revolute link rotate a box around its
    // local origin; use an environment box at the off-axis collision origin.
    auto in=initialization(false,2.2);auto xml=in["model"]["urdf_xml"].get<std::string>();
    const std::string old="<link name=\"tip\"><collision>";
    xml.replace(xml.find(old),old.size(),old+"<origin xyz=\"0.2 0 0\"/>");
    in["model"]["urdf_xml"]=xml;in["model"]["urdf_sha256"]=cable::hash(xml);
    cable::Engine sequence;sequence.initialize(in);
    for(double angle:{1.,0.,1.}) {
      auto result=sequence.query({{"joint_positions",{{"j",angle}}}});
      check(result["environment_collision"].get<bool>()==(angle==0),"separate-overlap-separate stale result");
      check(!result["self_collision"].get<bool>(),"self/environment classification");
    }
    bool rejected=false;try {sequence.query({{"joint_positions",{{"j",2}}}});}catch(...) {rejected=true;}
    check(rejected,"illegal state must fail");
    check(!sequence.query({{"joint_positions",{{"j",1}}}})["collision_detected"].get<bool>(),"recover after bad query");
    cable::Engine overlap;overlap.initialize(initialization(true,10));
    auto result=overlap.query(q);check(result["self_collision"].get<bool>() && !result["environment_collision"].get<bool>(),"self-only fixture");
    cable::Engine both;both.initialize(initialization(true,0));result=both.query(q);
    check(result["self_collision"].get<bool>() && result["environment_collision"].get<bool>(),"both fixture");
    auto bad=initialization();bad["model"]["urdf_sha256"]="wrong";
    rejected=false;try{cable::Engine engine;engine.initialize(bad);}catch(...){rejected=true;}check(rejected,"model hash rejected");
    bad=initialization();xml=bad["model"]["urdf_xml"].get<std::string>();
    auto at=xml.find("<box size=\"0.2 0.2 0.2\"/>");xml.replace(at,std::string("<box size=\"0.2 0.2 0.2\"/>").size(),"<mesh filename=\"file:///not_present_c1.stl\"/>");
    bad["model"]["urdf_xml"]=xml;bad["model"]["urdf_sha256"]=cable::hash(xml);
    rejected=false;try{cable::Engine engine;engine.initialize(bad);}catch(...){rejected=true;}check(rejected,"missing mesh rejected");
    auto policy_input=initialization(true,0);cable::Engine policy_engine;policy_engine.initialize(policy_input);
    J policy={{"version",1},{"model_sha256",policy_input["model"]["urdf_sha256"]},{"review_status","provisional_diagnostic"},{"scope","bare_gripper_model"},
      {"clearance",{{"environment_m",0},{"self_m",0},{"contact_mode","FCL_raw_boolean"}}},{"unresolved_pairs",J::array()},
      {"allowed_pairs",J::array({{{"links",{"base_link","tip"}},{"reason","test assembly"},{"evidence","synthetic fixture"},{"review_status","provisional_model_only"}}})}};
    policy_engine.set_policy({{"policy_canonical_json",policy.dump()},{"policy_sha256",cable::hash(policy.dump())}});
    result=policy_engine.query(q);
    check(result["raw_self_collision"].get<bool>() && !result["effective_self_collision"].get<bool>() && result["environment_collision"].get<bool>(),"ACM self only; environment preserved");
    auto three=policy_input;xml=three["model"]["urdf_xml"].get<std::string>();
    xml.insert(xml.find("</robot>"),R"(<link name="third"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="fixed_c" type="fixed"><parent link="base_link"/><child link="third"/><origin xyz="0.035 0 0"/></joint>)");
    three["model"]["urdf_xml"]=xml;three["model"]["urdf_sha256"]=cable::hash(xml);policy["model_sha256"]=cable::hash(xml);
    cable::Engine third;third.initialize(three);third.set_policy({{"policy_canonical_json",policy.dump()},{"policy_sha256",cable::hash(policy.dump())}});
    result=third.query({{"joint_positions",{{"j",0},{"fixed_c",0}}}});
    check(result["effective_self_collision"].get<bool>(),"A-C still collides after A-B allowed");
    policy["allowed_pairs"][0]["links"]={"base_link","missing"};rejected=false;
    try{third.set_policy({{"policy_canonical_json",policy.dump()},{"policy_sha256",cable::hash(policy.dump())}});}catch(...){rejected=true;}
    check(rejected,"unknown ACM link rejected");
    auto shifted=initialization(false,0);xml=shifted["model"]["urdf_xml"].get<std::string>();
    xml.insert(xml.find("</robot>"),R"(<link name="footprint"/><joint name="root_offset" type="fixed"><parent link="footprint"/><child link="base_link"/><origin xyz="0 0 1"/></joint>)");
    shifted["model"]["urdf_xml"]=xml;shifted["model"]["urdf_sha256"]=cable::hash(xml);
    cable::Engine offset;offset.initialize(shifted);result=offset.query({{"joint_positions",{{"j",0},{"root_offset",0}}}});
    check(!result["contacts_base"].empty(),"actual FCL contact required");
    for(const auto& contact:result["contacts_base"]) {
      check(std::abs(contact["position_base_m"][2].get<double>())<.2,"contact root-to-base conversion");
      check(contact["depth_m"].get<double>()>=0,"contact penetration finite nonnegative");
    }
    std::cout << "PASS: real MoveIt/FCL, separated, environment overlap, sequential clearing, invalid state, self-only, both, bad identity, missing mesh\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
