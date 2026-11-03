#include "engine.hpp"
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <resource_retriever/retriever.hpp>
#include <openssl/sha.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace cable {
static void need(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
std::string hash(const std::string& bytes) {
  unsigned char result[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), result);
  std::ostringstream out; for (auto x : result) out << std::hex << std::setw(2) << std::setfill('0') << int(x);
  return out.str();
}
std::string file_hash(const std::string& path) {
  std::ifstream f(path, std::ios::binary); need(bool(f), "cannot read identity file: " + path);
  return hash(std::string(std::istreambuf_iterator<char>(f), {}));
}
static double number(const J& x) {
  need(x.is_number() && !x.is_boolean(), "expected real number");
  double v=x.get<double>(); need(std::isfinite(v), "nonfinite number"); return v;
}
Eigen::Isometry3d matrix(const J& v) {
  need(v.is_array() && v.size()==4,"expected 4x4 transform");
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  for(int i=0;i<4;++i) { need(v[i].is_array() && v[i].size()==4,"expected 4x4 transform");
    for(int j=0;j<4;++j) t.matrix()(i,j)=number(v[i][j]); }
  need((t.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()==0,"invalid homogeneous row");
  need((t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm()<1e-9 &&
       std::abs(t.linear().determinant()-1)<1e-9,"invalid SO3"); return t;
}
J matrix_json(const Eigen::Isometry3d& t) {
  J v=J::array(); for(int i=0;i<4;++i) { J row=J::array(); for(int j=0;j<4;++j) row.push_back(t.matrix()(i,j)); v.push_back(row); } return v;
}
static Eigen::Isometry3d pose(const J& p) {
  auto xyz=p.at("position_m"), q=p.at("quaternion_xyzw");
  need(xyz.size()==3 && q.size()==4,"invalid pose dimensions");
  Eigen::Quaterniond quat(number(q[3]),number(q[0]),number(q[1]),number(q[2]));
  need(std::abs(quat.norm()-1)<1e-8,"invalid quaternion norm");
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=quat.normalized().toRotationMatrix();
  for(int i=0;i<3;++i)t.translation()[i]=number(xyz[i]);
  return t;
}
static J urdf_pose(const urdf::Pose& p) {
  return {{"position_m",{p.position.x,p.position.y,p.position.z}},
          {"quaternion_xyzw",{p.rotation.x,p.rotation.y,p.rotation.z,p.rotation.w}}};
}
J Engine::initialize(const J& in) {
  scene_.reset(); identity_=J();
  const auto& m=in.at("model");const std::string xml=m.at("urdf_xml");
  need(hash(xml)==m.at("urdf_sha256"),"URDF identity mismatch");
  urdf_=urdf::parseURDF(xml);need(bool(urdf_),"invalid URDF");
  base_=m.at("base_frame");ee_=m.at("ee_frame");
  need(bool(urdf_->getLink(base_)) && bool(urdf_->getLink(ee_)),"missing base/EE");
  for(auto link=urdf_->getLink(base_);link->parent_joint;link=urdf_->getLink(link->parent_joint->parent_link_name))
    need(link->parent_joint->type==urdf::Joint::FIXED,"C1 requires a fixed root-to-base chain");
  std::vector<std::string> chain_order;
  auto link=urdf_->getLink(ee_);
  while(link->name!=base_) {
    auto joint=link->parent_joint;need(bool(joint),"base is not EE ancestor");
    if(joint->type!=urdf::Joint::FIXED) {need(!joint->mimic,"mimic in selected independent chain");chain_order.push_back(joint->name);}
    link=urdf_->getLink(joint->parent_link_name);
  }
  std::reverse(chain_order.begin(),chain_order.end());
  need(J(chain_order)==m.at("joint_order"),"independent chain order mismatch");
  const auto canonical=in.at("scene_canonical_json").get<std::string>();
  const auto scene_hash=hash(canonical);need(scene_hash==in.at("scene_sha256"),"scene hash mismatch");
  const auto s=J::parse(canonical).at("scene");need(s.at("reference_frame")==base_,"scene frame mismatch");
  need(s.at("units")==J({{"length","m"},{"angle","rad"}}),"scene units mismatch");
  // Independently verify the Python-flattened poses against the canonical graph.
  std::map<std::string,Eigen::Isometry3d> frames;frames[base_]=Eigen::Isometry3d::Identity();
  const auto fs=s.at("frames");std::set<std::string> frame_ids;
  for(const auto& f:fs)need(frame_ids.insert(f.at("id").get<std::string>()).second && f.at("id")!=base_,"duplicate frame");
  for(size_t pass=0;pass<=fs.size();++pass)for(const auto& f:fs) {
    const std::string name=f.at("id"),parent=f.at("parent");
    if(!frames.count(name) && frames.count(parent))frames[name]=frames.at(parent)*pose(f.at("pose"));
  }
  need(frames.size()==fs.size()+1,"unresolved frame graph");
  objects_=in.at("objects");need(objects_.is_array() && objects_.size()==s.at("collision_objects").size(),"object count mismatch");
  std::map<std::string,J> flat;
  for(const auto& obj:objects_)need(flat.emplace(obj.at("id"),obj).second,"duplicate object ID");
  for(const auto& obj:s.at("collision_objects")) {
    const std::string id=obj.at("id");need(flat.count(id),"missing object");const auto& f=flat.at(id);
    need(f.at("type")==obj.at("type"),"object type mismatch");
    const auto expected=frames.at(obj.at("parent_frame"))*pose(obj.at("pose"));
    need((matrix(f.at("transform")).matrix()-expected.matrix()).norm()<1e-9,"object transform mismatch");
    for(const auto& key: {"size_m","radius_m","length_m"})if(obj.contains(key))need(f.at(key)==obj.at(key),"object dimensions mismatch");
  }
  J links=J::array(),joints=J::array(),geometry=J::array(),missing=J::array();size_t shape_count=0;
  resource_retriever::Retriever retriever;
  for(const auto& [name,l]:urdf_->links_) {
    size_t count=l->collision_array.size();
    links.push_back({{"name",name},{"collision_count",count}});
    if(!count && !l->visual_array.empty())missing.push_back(name);
    for(const auto& c:l->collision_array) {
      need(bool(c->geometry),"missing collision geometry");J g={{"link",name},{"origin",urdf_pose(c->origin)}};
      if(c->geometry->type==urdf::Geometry::MESH) {
        const auto mesh=std::dynamic_pointer_cast<urdf::Mesh>(c->geometry);need(bool(mesh),"bad mesh");
        need(mesh->filename.rfind("file://",0)==0 || mesh->filename.rfind("package://",0)==0,"only local mesh resources supported");
        const auto resource=retriever.get(mesh->filename);need(resource.size>0,"empty mesh resource");
        need(mesh->scale.x>0 && mesh->scale.y>0 && mesh->scale.z>0,"nonpositive mesh scale");
        g.update({{"type","mesh"},{"uri",mesh->filename},{"scale",{mesh->scale.x,mesh->scale.y,mesh->scale.z}},
                  {"resource_sha256",hash(std::string(reinterpret_cast<const char*>(resource.data.get()),resource.size))}});
      } else g["type"]="primitive";
      geometry.push_back(g);++shape_count;
    }
  }
  for(const auto& [name,j]:urdf_->joints_) {
    need(j->type==urdf::Joint::FIXED || j->type==urdf::Joint::REVOLUTE,"unsupported joint type");
    if(j->type==urdf::Joint::REVOLUTE)need(bool(j->limits) && std::isfinite(j->limits->lower) &&
      std::isfinite(j->limits->upper) && j->limits->lower<=j->limits->upper,"invalid joint limits");
    J row={{"name",name},{"type",j->type},{"parent",j->parent_link_name},{"child",j->child_link_name},
           {"origin",urdf_pose(j->parent_to_joint_origin_transform)}, {"axis",{j->axis.x,j->axis.y,j->axis.z}}};
    if(j->limits)row["limits"]={j->limits->lower,j->limits->upper};
    if(j->mimic)row["mimic"]={{"joint",j->mimic->joint_name},{"multiplier",j->mimic->multiplier},{"offset",j->mimic->offset}};
    joints.push_back(row);
  }
  auto srdf=std::make_shared<srdf::Model>();
  const auto srdf_xml="<robot name=\""+urdf_->getName()+"\"></robot>";
  need(srdf->initString(*urdf_,srdf_xml),"SRDF initialization failed");
  auto robot=std::make_shared<moveit::core::RobotModel>(urdf_,srdf);
  scene_=std::make_shared<planning_scene::PlanningScene>(robot);
  scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  scene_->getAllowedCollisionMatrixNonConst().clear(); // No adjacent/fixed/random exemptions.
  J moveit_bounds=J::object();
  for(const auto& name:robot->getVariableNames()) {
    const auto& b=robot->getVariableBounds(name);
    moveit_bounds[name]={{"lower",b.min_position_},{"upper",b.max_position_},{"bounded",b.position_bounded_}};
  }
  for(const auto& [name,j]:urdf_->joints_)if(j->type==urdf::Joint::REVOLUTE) {
    const auto& b=robot->getVariableBounds(name);
    need(b.position_bounded_ && std::abs(b.min_position_-j->limits->lower)<1e-12 &&
         std::abs(b.max_position_-j->limits->upper)<1e-12,"MoveIt/URDF limits disagree");
  }
  J loaded_shapes=J::array();
  size_t actual_shapes=0;
  for(const auto* l:robot->getLinkModels()) {
    actual_shapes+=l->getShapes().size();
    for(size_t i=0;i<l->getShapes().size();++i) {
      const auto& shape=l->getShapes()[i];
      J row={{"link",l->getName()},{"origin",matrix_json(l->getCollisionOriginTransforms()[i])}};
      if(shape->type==shapes::MESH) {
        auto mesh=static_cast<const shapes::Mesh*>(shape.get());
        need(mesh->vertex_count>0 && mesh->triangle_count>0,"empty loaded mesh");
        row["vertices"]=mesh->vertex_count;row["triangles"]=mesh->triangle_count;
      }
      loaded_shapes.push_back(row);
    }
  }
  need(actual_shapes==shape_count,"MoveIt lost collision shapes (resource load failure)");
  auto& state=scene_->getCurrentStateNonConst();state.setToDefaultValues();state.update();
  const auto root_to_base=state.getGlobalLinkTransform(base_);
  for(const auto& obj:objects_) {
    const std::string id=obj.at("id");need(!robot->hasLinkModel(id),"object ID shadows robot link");
    moveit_msgs::msg::CollisionObject msg;msg.id=id;msg.header.frame_id=robot->getModelFrame();msg.operation=msg.ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    if(obj.at("type")=="box") {
      auto size=obj.at("size_m");need(size.size()==3,"box dimensions");primitive.type=primitive.BOX;
      for(const auto& x:size) { double v=number(x);need(v>0,"nonpositive size");primitive.dimensions.push_back(v); }
    } else {
      need(obj.at("type")=="cylinder","unsupported primitive");primitive.type=primitive.CYLINDER;
      double radius=number(obj.at("radius_m")),length=number(obj.at("length_m"));need(radius>0 && length>0,"nonpositive cylinder");
      primitive.dimensions={length,radius};
    }
    auto t=root_to_base*matrix(obj.at("transform"));Eigen::Quaterniond q(t.linear());geometry_msgs::msg::Pose p;
    p.position.x=t.translation().x();p.position.y=t.translation().y();p.position.z=t.translation().z();
    p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
    msg.primitives.push_back(primitive);msg.primitive_poses.push_back(p);
    need(scene_->processCollisionObjectMsg(msg),"scene object load failed");
  }
  need(scene_->getWorld()->size()==objects_.size(),"MoveIt world count mismatch");
  J loaded_objects=J::array();
  for(const auto& id:scene_->getWorld()->getObjectIds()) {
    auto object=scene_->getWorld()->getObject(id);
    need(object->shapes_.size()==1,"unexpected shape count");
    J row={{"id",id},{"transform",matrix_json(root_to_base.inverse()*object->global_shape_poses_[0])}};
    if(object->shapes_[0]->type==shapes::BOX) {
      const auto* box=static_cast<const shapes::Box*>(object->shapes_[0].get());
      row["type"]="box";row["size_m"]={box->size[0],box->size[1],box->size[2]};
    } else {
      const auto* cylinder=static_cast<const shapes::Cylinder*>(object->shapes_[0].get());
      row["type"]="cylinder";row["radius_m"]=cylinder->radius;row["length_m"]=cylinder->length;
    }
    loaded_objects.push_back(row);
  }
  const std::string policy="C1:full_robot;empty_ACM;zero_padding;no_clearance;no_tools;coverage_unverified";
  identity_={{"urdf_sha256",hash(xml)},{"scene_sha256",scene_hash},{"robot_geometry_sha256",hash(geometry.dump())},
             {"policy_sha256",hash(policy)},{"srdf_sha256",hash(srdf_xml)},
             {"backend_version","cable_collision_backend/0.1.0"},{"backend_sha256",file_hash("/proc/self/exe")},
             {"base_frame",base_},{"ee_frame",ee_},{"joint_order",m.at("joint_order")},
             {"units",{{"length","m"},{"angle","rad"}}}};
  audit_={{"links",links},{"joints",joints},{"geometry",geometry},{"collision_geometry_count",shape_count},
          {"moveit_shape_count",actual_shapes},{"moveit_variable_bounds",moveit_bounds},{"loaded_shapes",loaded_shapes},{"loaded_objects_base",loaded_objects},{"urdf_link_count",urdf_->links_.size()},{"urdf_joint_count",urdf_->joints_.size()},
          {"moveit_joint_count_including_virtual_root",robot->getJointModels().size()},
          {"root_frame",robot->getModelFrame()},{"root_to_base",matrix_json(root_to_base)},
          {"detector",scene_->getCollisionDetectorName()},{"acm_size",scene_->getAllowedCollisionMatrix().getSize()},
          {"policy",policy},{"missing_visual_collision_links",missing},{"objects_base",objects_},
          {"coverage_verified",false},{"acm_reviewed",false},{"tool_modeled",false},{"clearance_implemented",false}};
  need(audit_["detector"]=="FCL","FCL not active");
  return {{"identity",identity_},{"audit",audit_},{"backend_status","READY"}};
}
J Engine::query(const J& in) {
  need(bool(scene_) && !identity_.is_null(),"backend not initialized");
  const auto start=std::chrono::steady_clock::now();const auto positions=in.at("joint_positions");
  need(positions.is_object() && positions.size()==urdf_->joints_.size(),"full URDF joint state required");
  for(const auto& [name,j]:urdf_->joints_) {
    need(positions.contains(name),"missing joint: "+name);double v=number(positions.at(name));
    if(j->type==urdf::Joint::FIXED)need(v==0,"fixed joint must be zero");
    else {need(j->type==urdf::Joint::REVOLUTE && bool(j->limits),"unsupported joint type");
      need(v>=j->limits->lower && v<=j->limits->upper,"joint outside URDF bounds: "+name);}
    if(j->mimic)need(std::abs(v-(number(positions.at(j->mimic->joint_name))*j->mimic->multiplier+j->mimic->offset))<1e-10,"mimic inconsistency");
  }
  auto& state=scene_->getCurrentStateNonConst();state.setToDefaultValues();
  for(const auto& [name,j]:urdf_->joints_)if(j->type!=urdf::Joint::FIXED && !j->mimic)state.setVariablePosition(name,number(positions.at(name)));
  state.update(true);state.updateCollisionBodyTransforms();need(state.satisfiesBounds(),"MoveIt bounds failure");
  J used=J::object(),transforms=J::object();
  for(const auto& [name,j]:urdf_->joints_) {
    double v=j->type==urdf::Joint::FIXED ? 0 : state.getVariablePosition(name);
    need(std::abs(v-number(positions.at(name)))<1e-10,"MoveIt full-state disagreement");used[name]=v;
  }
  auto root_to_base=state.getGlobalLinkTransform(base_);
  for(const auto* l:scene_->getRobotModel()->getLinkModels())transforms[l->getName()]=matrix_json(root_to_base.inverse()*state.getGlobalLinkTransform(l));
  collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=10000;req.max_contacts_per_pair=1;
  collision_detection::CollisionResult self,env;self.clear();env.clear();
  scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,self,state,scene_->getAllowedCollisionMatrix());
  scene_->getCollisionEnvUnpadded()->checkRobotCollision(req,env,state,scene_->getAllowedCollisionMatrix());
  J pairs=J::array();
  for(auto item : {std::make_pair("self",&self),std::make_pair("environment",&env)})
    for(const auto& contact:item.second->contacts)pairs.push_back({{"kind",item.first},{"first",contact.first.first},{"second",contact.first.second}});
  const bool collision=self.collision || env.collision;
  return {{"identity",identity_},{"backend_status","OK"},{"collision_detected",collision},
          {"self_collision",self.collision},{"environment_collision",env.collision},{"colliding_pairs",pairs},
          {"pairs_may_be_truncated",self.contact_count>=req.max_contacts || env.contact_count>=req.max_contacts},
          {"decision",collision?"COLLIDING":"UNVERIFIED"},
          {"unverified_reasons",{"collision_coverage_unreviewed","ACM_unreviewed_empty","RJ45_tool_TCP_missing","gripper_uncalibrated","clearance_not_implemented"}},
          {"joint_positions_used",used},{"link_transforms_base",transforms},
          {"T_base_to_ee",transforms.at(ee_)},{"object_count",objects_.size()},
          {"query_elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
}
}
