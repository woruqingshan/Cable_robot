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
    J clearance_cases=J::array();
    auto distance_check=[&](const char* name,double actual,double expected) {
      clearance_cases.push_back({{"case",name},{"actual_m",actual},{"expected_m",expected},{"absolute_error_m",std::abs(actual-expected)},{"tolerance_m",1e-6}});
      check(std::abs(actual-expected)<1e-6,name);
    };
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
    check(result["clearance"]["self"]["separation_m"]==0,"overlap is unsigned zero, not penetration distance");
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
    check(!result["clearance"]["self"]["available"].get<bool>() && result["clearance"]["self"]["separation_m"].is_null(),"all self pairs explicitly ACM-excluded: distance unavailable");
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
    for(double x:{.5,.15,0.}) {
      cable::Engine distance_fixture;distance_fixture.initialize(initialization(false,x));auto d=distance_fixture.query(q);
      std::cerr<<"distance probe x="<<x<<" "<<d["clearance"].dump()<<std::endl;
      distance_check("box environment positive/touch/overlap",d["clearance"]["environment"]["separation_m"].get<double>(),std::max(0.,x-.15));
      distance_check("primitive self distance",d["clearance"]["self"]["separation_m"].get<double>(),1.85);
    }
    auto sphere_input=initialization(false,.5);xml=sphere_input["model"]["urdf_xml"].get<std::string>();
    at=xml.find("<box size=\"0.2 0.2 0.2\"/>");xml.replace(at,std::string("<box size=\"0.2 0.2 0.2\"/>").size(),"<sphere radius=\"0.1\"/>");
    sphere_input["model"]["urdf_xml"]=xml;sphere_input["model"]["urdf_sha256"]=cable::hash(xml);
    cable::Engine sphere;sphere.initialize(sphere_input);auto sd=sphere.query(q);
    distance_check("sphere-box distance",sd["clearance"]["environment"]["separation_m"].get<double>(),.35);
    // Regression for the locally reproduced LIBCCD input-order discrepancy.
    auto a=std::make_shared<fcl::Boxd>(.2,.2,.2),b=std::make_shared<fcl::Boxd>(.1,.1,.1);
    Eigen::Isometry3d bt=Eigen::Isometry3d::Identity();bt.translation().x()=.5;
    fcl::CollisionObjectd first(a),second(b,bt);
    for(bool reverse:{false,true}) {
      fcl::DistanceRequestd request(false);request.gjk_solver_type=fcl::GST_INDEP;fcl::DistanceResultd answer;
      distance_check("order-independent box distance",fcl::distance(reverse?&second:&first,reverse?&first:&second,request,answer),.35);
    }
    auto rotated=initialization(false,.5);auto canon=J::parse(rotated["scene_canonical_json"].get<std::string>());
    canon["scene"]["collision_objects"][0]["pose"]["quaternion_xyzw"]={0,0,std::sin(.7853981633974483/2),std::cos(.7853981633974483/2)};
    Eigen::Isometry3d rt=Eigen::Isometry3d::Identity();rt.linear()=Eigen::AngleAxisd(.7853981633974483,Eigen::Vector3d::UnitZ()).toRotationMatrix();rt.translation().x()=.5;
    rotated["objects"][0]["transform"]=cable::matrix_json(rt);rotated["scene_canonical_json"]=canon.dump();rotated["scene_sha256"]=cable::hash(canon.dump());
    cable::Engine rotated_distance;rotated_distance.initialize(rotated);auto rd=rotated_distance.query(q);
    distance_check("rotated box distance",rd["clearance"]["environment"]["separation_m"].get<double>(),.4-.05*std::sqrt(2.));
    auto multi=initialization(false,.5);canon=J::parse(multi["scene_canonical_json"].get<std::string>());
    auto closer=canon["scene"]["collision_objects"][0];closer["id"]="nearer";closer["pose"]["position_m"]={.3,0,0};canon["scene"]["collision_objects"].push_back(closer);
    auto flat=multi["objects"][0];flat["id"]="nearer";auto tr=Eigen::Isometry3d::Identity();tr.translation().x()=.3;flat["transform"]=cable::matrix_json(tr);multi["objects"].push_back(flat);
    multi["scene_canonical_json"]=canon.dump();multi["scene_sha256"]=cable::hash(canon.dump());
    cable::Engine nearest;nearest.initialize(multi);auto nd=nearest.query(q);
    distance_check("nearest of multiple objects",nd["clearance"]["environment"]["separation_m"].get<double>(),.15);
    check(nd["clearance"]["environment"]["nearest_pair"][0]=="nearer" || nd["clearance"]["environment"]["nearest_pair"][1]=="nearer","nearest object identity");
    std::cout << "CLEARANCE_VALIDATION=" << clearance_cases.dump() << '\n';
    std::cout << "PASS: real MoveIt/FCL, collision regressions, analytic clearance and explicit solver order\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
