#include "engine.hpp"
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <resource_retriever/retriever.hpp>
#include <openssl/sha.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
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
static std::pair<std::string,std::string> pair_key(std::string a,std::string b) {
  if(b<a)std::swap(a,b);
  return {a,b};
}
static bool selected(const std::set<std::pair<std::string,std::string>>& pairs,const std::string& a,const std::string& b) {
  return pairs.count(pair_key(a,b));
}
J Engine::initialize(const J& in) {
  scene_.reset(); identity_=J(); semantic_enabled_=false; semantic_audit_=J();
  policy_acm_.clear();raw_acm_.clear();policy_=J();refined_.reset();profile_=J();profile_pairs_.clear();distance_only_pairs_.clear();
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
    const Eigen::Isometry3d expected=frames.at(obj.at("parent_frame"))*pose(obj.at("pose"));
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
  std::string srdf_xml="<robot name=\""+urdf_->getName()+"\"></robot>";
  std::set<std::pair<std::string,std::string>> declared_pairs;
  if(in.contains("semantic")) {
    need(!in.contains("geometry_profile_canonical_json"),"semantic mode requires unmodified official geometry");
    const auto& sem=in.at("semantic");srdf_xml=sem.at("srdf_xml");
    need(hash(srdf_xml)==sem.at("srdf_sha256") && sem.at("urdf_sha256")==hash(xml),"semantic SRDF/model hash mismatch");
    const std::string review=sem.at("review_canonical_json");
    need(hash(review)==sem.at("review_sha256"),"semantic review hash mismatch");
    auto reviewed=J::parse(review);need(reviewed.at("urdf_sha256")==hash(xml) && reviewed.at("srdf_sha256")==hash(srdf_xml),"semantic review binding mismatch");
    need(reviewed.at("status")=="model_only_partial","unsupported semantic review claim");
    need(srdf_xml.find("<!DOCTYPE")==std::string::npos && srdf_xml.find("<!ENTITY")==std::string::npos,"SRDF DTD/entities forbidden");
    tinyxml2::XMLDocument doc;need(doc.Parse(srdf_xml.c_str())==tinyxml2::XML_SUCCESS,"invalid SRDF XML");
    const auto* root=doc.RootElement();
    need(root && std::string(root->Name())=="robot" && root->Attribute("name") && urdf_->getName()==root->Attribute("name"),"SRDF robot name mismatch");
    auto attr=[](const tinyxml2::XMLElement* e,const char* k)->std::string {const char* v=e->Attribute(k);need(v && *v,std::string("missing SRDF attribute ")+k);return v;};
    std::set<std::string> groups;std::set<std::pair<std::string,std::string>> states;
    for(auto e=root->FirstChildElement();e;e=e->NextSiblingElement())if(std::string(e->Name())=="group")need(groups.insert(attr(e,"name")).second,"duplicate SRDF group");
    for(auto e=root->FirstChildElement();e;e=e->NextSiblingElement()) {
      const std::string tag=e->Name();
      if(tag=="disable_collisions") {
        auto a=attr(e,"link1"),b=attr(e,"link2");attr(e,"reason");
        need(a!=b && urdf_->getLink(a) && urdf_->getLink(b) && declared_pairs.insert(pair_key(a,b)).second,"invalid/duplicate SRDF disabled pair");
        need(!e->FirstChildElement(),"invalid disabled pair child");
      } else if(tag=="group") {
        need(e->FirstChildElement(),"empty SRDF group");
        for(auto c=e->FirstChildElement();c;c=c->NextSiblingElement()) {
          const std::string kind=c->Name();
          if(kind=="joint")need(bool(urdf_->getJoint(attr(c,"name"))),"unknown SRDF joint");
          else if(kind=="link")need(bool(urdf_->getLink(attr(c,"name"))),"unknown SRDF link");
          else if(kind=="chain") {
            auto b=attr(c,"base_link"),v=attr(c,"tip_link");auto link=urdf_->getLink(v);need(bool(link) && bool(urdf_->getLink(b)),"unknown SRDF chain link");
            while(link->name!=b && link->parent_joint)link=urdf_->getLink(link->parent_joint->parent_link_name);
            need(link->name==b && b!=v,"invalid SRDF chain");
          } else if(kind=="group")need(groups.count(attr(c,"name")),"unknown SRDF subgroup");
          else need(false,"unsupported SRDF group element");
        }
      } else if(tag=="group_state") {
        const auto g=attr(e,"group");need(groups.count(g) && states.insert({g,attr(e,"name")}).second,"invalid SRDF group state");
        std::set<std::string> names;
        for(auto c=e->FirstChildElement();c;c=c->NextSiblingElement()) {
          need(std::string(c->Name())=="joint","invalid state element");auto j=urdf_->getJoint(attr(c,"name"));
          need(bool(j) && j->type==urdf::Joint::REVOLUTE && !j->mimic && names.insert(j->name).second,"invalid state joint");
          std::istringstream v(attr(c,"value"));double value;std::string extra;need(bool(v>>value) && !(v>>extra) && std::isfinite(value) && value>=j->limits->lower && value<=j->limits->upper,"invalid named state value");
        }
      } else need(false,"unsupported SRDF element (explicit support required)");
    }
    std::map<std::string,std::set<std::string>> edges;
    for(auto e=root->FirstChildElement("group");e;e=e->NextSiblingElement("group"))
      for(auto c=e->FirstChildElement("group");c;c=c->NextSiblingElement("group"))edges[attr(e,"name")].insert(attr(c,"name"));
    std::set<std::string> visiting,done;
    std::function<void(const std::string&)> visit=[&](const std::string& n){
      need(!visiting.count(n),"cyclic SRDF subgroup");if(done.count(n))return;
      visiting.insert(n);for(const auto& child:edges[n])visit(child);visiting.erase(n);done.insert(n);
    };for(const auto& n:groups)visit(n);
    J expected=J::array();for(const auto& p:declared_pairs)expected.push_back({p.first,p.second});
    need(expected==sem.at("disabled_pairs") && expected==reviewed.at("disabled_pairs"),"reviewed SRDF disabled pairs mismatch");
    std::set<std::pair<std::string,std::string>> reviewed_pairs;
    for(const auto& row:reviewed.at("pair_reviews")) {
      auto names=row.at("links");need(names.is_array() && names.size()==2 && row.at("decision")=="disabled_model_only" && row.at("evidence").is_string() && !row.at("evidence").get<std::string>().empty(),"missing semantic pair review evidence");
      need(reviewed_pairs.insert(pair_key(names[0],names[1])).second,"duplicate semantic review pair");
    }
    need(reviewed_pairs==declared_pairs,"unreviewed disabled SRDF pair");
    semantic_enabled_=true;
    semantic_audit_={{"disabled_pairs",expected},{"review_sha256",hash(review)},{"review_status",reviewed.at("status")},{"raw_acm_size",0},{"overlay_applied",false}};
  }
  need(srdf->initString(*urdf_,srdf_xml),"SRDF initialization failed");
  auto robot=std::make_shared<moveit::core::RobotModel>(urdf_,srdf);
  if(semantic_enabled_) {
    need(robot->getJointModelGroupNames().size()==srdf->getGroups().size(),"MoveIt lost invalid SRDF group");
    for(const auto& st:srdf->getGroupStates()) {
      const auto* group=robot->getJointModelGroup(st.group_);need(group,"unknown named state group");
      for(const auto& [n,v]:st.joint_values_)need(group->hasJointModel(n),"named state joint outside group");
    }
  }
  scene_=std::make_shared<planning_scene::PlanningScene>(robot);
  scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  if(!semantic_enabled_)scene_->getAllowedCollisionMatrixNonConst().clear();
  if(semantic_enabled_) {
    const auto& acm=scene_->getAllowedCollisionMatrix();J actual=J::array();
    for(const auto& [a,la]:urdf_->links_)for(const auto& [b,lb]:urdf_->links_)if(a<b) {
      collision_detection::AllowedCollision::Type kind;
      if(acm.getAllowedCollision(a,b,kind) && kind==collision_detection::AllowedCollision::ALWAYS)actual.push_back({a,b});
    }
    need(actual==semantic_audit_.at("disabled_pairs"),"MoveIt initialized ACM differs from reviewed SRDF");
    semantic_audit_["actual_disabled_pairs"]=actual;semantic_audit_["acm_sha256"]=hash(actual.dump());
    semantic_audit_["initialization_verified"]=true;
  }
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
    Eigen::Isometry3d t=root_to_base*matrix(obj.at("transform"));Eigen::Quaterniond q(t.linear());geometry_msgs::msg::Pose p;
    p.position.x=t.translation().x();p.position.y=t.translation().y();p.position.z=t.translation().z();
    p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
    msg.primitives.push_back(primitive);msg.primitive_poses.push_back(p);
    need(scene_->processCollisionObjectMsg(msg),"scene object load failed");
  }
  need(scene_->getWorld()->size()==objects_.size(),"MoveIt world count mismatch");
  // Pin MoveIt's shape cache for persistent distance queries; reuse the exact
  // loaded/scaled geometry and native RobotState, never a second FK/model.
  distance_geometry_.clear();
  for(const auto* link:robot->getLinkModels())
    for(size_t i=0;i<link->getShapes().size();++i)
      distance_geometry_.push_back(collision_detection::createCollisionGeometry(link->getShapes()[i],link,i));
  for(const auto& id:scene_->getWorld()->getObjectIds()) {
    const auto object=scene_->getWorld()->getObject(id);
    for(const auto& shape:object->shapes_)
      distance_geometry_.push_back(collision_detection::createCollisionGeometry(shape,object.get()));
  }
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
          {"coverage_verified",false},{"acm_reviewed",false},{"tool_modeled",false},{"clearance_implemented",true}};
  need(audit_["detector"]=="FCL","FCL not active");
  if(semantic_enabled_) {
    audit_["semantic"]=semantic_audit_;
    identity_["semantic_acm_sha256"]=semantic_audit_.at("acm_sha256");
    identity_["semantic_review_sha256"]=semantic_audit_.at("review_sha256");
  }
  J reply={{"audit",audit_},{"backend_status","READY"}};
  if(in.contains("geometry_profile_canonical_json")) {
    const auto text=in.at("geometry_profile_canonical_json").get<std::string>();
    need(hash(text)==in.at("geometry_profile_sha256"),"geometry profile hash mismatch");profile_=J::parse(text);
    need(profile_.at("version")==1 && profile_.at("source_urdf_sha256")==hash(xml),"geometry profile source mismatch");
    need(profile_.at("method")=="official_full" || profile_.at("method")=="pair_local_triangle_exclusion","unsupported geometry profile");
    identity_["source_urdf_sha256"]=hash(xml);identity_["source_geometry_sha256"]=identity_["robot_geometry_sha256"];
    identity_["effective_urdf_sha256"]=hash(xml);identity_["effective_geometry_sha256"]=identity_["robot_geometry_sha256"];
    identity_["geometry_profile_sha256"]=hash(text);identity_["geometry_profile"]=profile_.at("id");
    identity_["geometry_review_status"]=profile_.at("review_status");
    if(profile_.at("method")=="pair_local_triangle_exclusion") {
      need(profile_.at("review_status")=="diagnostic_only","refined geometry cannot claim physical validation");
      const auto& em=in.at("effective_model");const std::string derived=em.at("urdf_xml");
      need(hash(derived)==em.at("urdf_sha256"),"effective URDF hash mismatch");auto eu=urdf::parseURDF(derived);need(bool(eu),"invalid effective URDF");
      need(eu->links_.size()==urdf_->links_.size() && eu->joints_.size()==urdf_->joints_.size() && eu->getRoot()->name==urdf_->getRoot()->name,"effective tree changed");
      for(const auto& [name,j]:urdf_->joints_) {
        auto k=eu->getJoint(name);need(bool(k),"effective joint missing");
        need(k->type==j->type && k->parent_link_name==j->parent_link_name && k->child_link_name==j->child_link_name && urdf_pose(k->parent_to_joint_origin_transform)==urdf_pose(j->parent_to_joint_origin_transform),"effective kinematics changed");
        need(k->axis.x==j->axis.x && k->axis.y==j->axis.y && k->axis.z==j->axis.z,"effective axis changed");
        need(bool(k->limits)==bool(j->limits) && bool(k->mimic)==bool(j->mimic),"effective joint semantics changed");
        if(j->limits)need(k->limits->lower==j->limits->lower && k->limits->upper==j->limits->upper && k->limits->effort==j->limits->effort && k->limits->velocity==j->limits->velocity,"effective limits changed");
        if(j->mimic)need(k->mimic->joint_name==j->mimic->joint_name && k->mimic->multiplier==j->mimic->multiplier && k->mimic->offset==j->mimic->offset,"effective mimic changed");
      }
      for(const auto& pair:profile_.at("target_pairs")) {
        const std::string a=pair.at(0),b=pair.at(1);need(a!=b && bool(urdf_->getLink(a)) && bool(urdf_->getLink(b)),"invalid profile pair");
        need(profile_pairs_.insert(pair_key(a,b)).second,"duplicate profile pair");
      }
      need(!profile_pairs_.empty(),"empty refinement scope");
      J nested=in;for(const auto* key:{"geometry_profile_canonical_json","geometry_profile_sha256","effective_model"})nested.erase(key);
      nested["model"]["urdf_xml"]=derived;nested["model"]["urdf_sha256"]=hash(derived);
      refined_=std::make_unique<Engine>();effective_ready_=refined_->initialize(nested);refined_->distance_only_pairs_=profile_pairs_;
      std::map<std::string,J> edits;for(const auto& edit:profile_.at("edits"))need(edits.emplace(edit.at("link"),edit).second,"duplicate geometry edit");
      const auto& dg=effective_ready_.at("audit").at("geometry");need(dg.size()==geometry.size(),"profile changed collision count");
      for(size_t i=0;i<geometry.size();++i) {
        const auto& a=geometry[i];const auto& b=dg[i];const std::string name=a.at("link");
        if(!edits.count(name))need(a==b,"undeclared geometry modification");
        else {
          const auto& edit=edits.at(name);need(a.at("link")==b.at("link") && a.at("origin")==b.at("origin") && a.at("scale")==b.at("scale"),"geometry origin/scale changed");
          need(a.at("resource_sha256")==edit.at("source_sha256") && b.at("resource_sha256")==edit.at("effective_sha256"),"profile mesh hash mismatch");
          need(a.at("uri")==edit.at("source_uri") && b.at("uri")==edit.at("effective_uri"),"profile mesh URI mismatch");
        }
      }
      identity_["effective_urdf_sha256"]=hash(derived);identity_["effective_geometry_sha256"]=effective_ready_.at("identity").at("robot_geometry_sha256");
      identity_["robot_geometry_sha256"]=hash(identity_["source_geometry_sha256"].get<std::string>()+":"+identity_["effective_geometry_sha256"].get<std::string>()+":"+hash(text));
      reply["effective_ready"]=effective_ready_;
    } else need(!in.contains("effective_model") && profile_.at("edits").empty(),"official profile must not replace geometry");
    reply["geometry_profile"]=profile_;
  }
  reply["identity"]=identity_;return reply;
}
J Engine::set_policy(const J& in) {
  need(!semantic_enabled_,"semantic mode forbids implicit historical policy overlay");
  need(bool(scene_),"not initialized");
  const auto text=in.at("policy_canonical_json").get<std::string>();
  need(hash(text)==in.at("policy_sha256"),"policy hash mismatch");
  const auto p=J::parse(text);
  need(p.at("version")==1 && p.at("model_sha256")==identity_.at("urdf_sha256"),"policy model/version mismatch");
  need(p.at("review_status")=="baseline" || p.at("review_status")=="provisional_diagnostic","unvalidated review claim");
  need(p.at("scope")=="bare_gripper_model","unsupported tool scope");
  const auto c=p.at("clearance");
  need(number(c.at("environment_m"))==0 && number(c.at("self_m"))==0 && c.at("contact_mode")=="FCL_raw_boolean","unsupported clearance");
  collision_detection::AllowedCollisionMatrix acm;std::set<std::pair<std::string,std::string>> pairs;
  for(const auto& item:p.at("allowed_pairs")) {
    auto pair=item.at("links").get<std::vector<std::string>>();need(pair.size()==2,"invalid pair");std::sort(pair.begin(),pair.end());
    need(pair[0]!=pair[1] && scene_->getRobotModel()->hasLinkModel(pair[0]) && scene_->getRobotModel()->hasLinkModel(pair[1]),"unknown self link");
    need(item.at("review_status")=="provisional_model_only" && !item.at("reason").get<std::string>().empty() && !item.at("evidence").get<std::string>().empty(),"unreviewed pair");
    need(pairs.insert({pair[0],pair[1]}).second,"duplicate policy pair");acm.setEntry(pair[0],pair[1],true);
  }
  for(const auto& pair:p.at("unresolved_pairs")) {
    auto names=pair.get<std::vector<std::string>>();need(names.size()==2,"invalid unresolved pair");std::sort(names.begin(),names.end());
    need(!pairs.count({names[0],names[1]}),"unresolved pair allowed");
  }
  policy_acm_=acm;policy_=p;identity_["policy_sha256"]=hash(text);
  if(refined_) {
    J nested=p;nested["model_sha256"]=refined_->identity().at("urdf_sha256");const auto text=nested.dump();
    refined_->set_policy({{"policy_canonical_json",text},{"policy_sha256",hash(text)}});
  }
  return {{"identity",identity_},{"backend_status","POLICY_SET"},{"policy",policy_}};
}
J Engine::query(const J& in) {
  need(bool(scene_) && !identity_.is_null(),"backend not initialized");
  if(semantic_enabled_)need(!in.contains("task_scope_canonical_json") && !in.contains("task_scope_sha256") && !in.contains("task_scope_context"),"semantic mode forbids task scope overlay");
  if(refined_ && (in.contains("task_scope_canonical_json") || in.contains("task_scope_sha256") || in.contains("task_scope_context")))throw std::runtime_error("task scope v0 requires official full geometry; diagnostic refinement unsupported");
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
  if(in.contains("geometry_audit")) {
    const auto& a=in.at("geometry_audit");
    need(a.is_object() && (a.size()==2 || (a.size()==3 && a.contains("distance_pairs"))) && a.at("max_contacts_per_pair").is_number_integer() && a.at("max_contacts").is_number_integer(),"invalid geometry audit limits");
    req.max_contacts_per_pair=a.at("max_contacts_per_pair").get<size_t>();req.max_contacts=a.at("max_contacts").get<size_t>();
    need(req.max_contacts_per_pair>=1 && req.max_contacts_per_pair<=512 && req.max_contacts>=req.max_contacts_per_pair && req.max_contacts<=50000,"geometry audit limits outside bounds");
  }
  collision_detection::CollisionResult self,env;self.clear();env.clear();
  scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,self,state,raw_acm_);
  scene_->getCollisionEnvUnpadded()->checkRobotCollision(req,env,state,raw_acm_);
  J pairs=J::array(), contacts=J::array();
  for(auto item : {std::make_pair("self",&self),std::make_pair("environment",&env)})
    for(const auto& contact:item.second->contacts)pairs.push_back({{"kind",item.first},{"first",contact.first.first},{"second",contact.first.second}});
  collision_detection::CollisionResult effective;effective.clear();
  scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,effective,state,policy_acm_);
  J filtered=J::array(), effective_pairs=J::array();
  for(const auto& contact:self.contacts) {
    collision_detection::AllowedCollision::Type type;
    if(policy_acm_.getEntry(contact.first.first,contact.first.second,type) && type==collision_detection::AllowedCollision::ALWAYS)
      filtered.push_back({contact.first.first,contact.first.second});
  }
  for(const auto& contact:effective.contacts)effective_pairs.push_back({contact.first.first,contact.first.second});
  for(auto item : {std::make_pair("self",&self),std::make_pair("environment",&env)})
    for(const auto& pair:item.second->contacts) for(const auto& contact:pair.second) {
      const Eigen::Vector3d base_position=root_to_base.inverse()*contact.pos;
      const Eigen::Vector3d base_normal=root_to_base.linear().transpose()*contact.normal;
      need(base_position.allFinite() && base_normal.allFinite() && std::isfinite(contact.depth),"invalid FCL contact");
      J record={{"kind",item.first},{"first",pair.first.first},{"second",pair.first.second},
        {"position_base_m",{base_position.x(),base_position.y(),base_position.z()}},
        {"normal_base",{base_normal.x(),base_normal.y(),base_normal.z()}},{"depth_m",contact.depth},
        {"source","MoveIt/FCL CollisionResult"}};
      if(in.contains("geometry_audit"))for(const auto* side:{"first","second"}) {
        const std::string name=record.at(side);if(transforms.contains(name)) {
          Eigen::Vector3d local=matrix(transforms.at(name)).inverse()*base_position;
          record[std::string("position_")+side+"_link_m"]={local.x(),local.y(),local.z()};
        }
      }
      contacts.push_back(record);
    }
  // Installed MoveIt distanceCallback uses the default LIBCCD solver, which
  // fails the reversed-box analytic fixture on this FCL 0.7 installation.
  // Select FCL GST_INDEP explicitly on MoveIt's own geometry/transforms.
  using NamedObject=std::pair<std::string,std::shared_ptr<fcl::CollisionObjectd>>;
  std::vector<NamedObject> robot_objects,world_objects;
  for(const auto* link:scene_->getRobotModel()->getLinkModels())
    for(size_t i=0;i<link->getShapes().size();++i) {
      auto g=collision_detection::createCollisionGeometry(link->getShapes()[i],link,i);
      robot_objects.emplace_back(link->getName(),std::make_shared<fcl::CollisionObjectd>(g->collision_geometry_,state.getCollisionBodyTransform(link,i)));
    }
  for(const auto& id:scene_->getWorld()->getObjectIds()) {
    const auto object=scene_->getWorld()->getObject(id);
    for(size_t i=0;i<object->shapes_.size();++i) {
      auto g=collision_detection::createCollisionGeometry(object->shapes_[i],object.get());
      world_objects.emplace_back(id,std::make_shared<fcl::CollisionObjectd>(g->collision_geometry_,object->global_shape_poses_[i]));
    }
  }
  auto distance_json=[&](bool self_query,bool colliding,const collision_detection::CollisionResult& collisions,int exclude_targets=0,const collision_detection::AllowedCollisionMatrix* override_acm=nullptr) {
    double best=std::numeric_limits<double>::infinity();J nearest=J::array();
    if(self_query && (exclude_targets || !distance_only_pairs_.empty())) {
      colliding=false;for(const auto& pair:collisions.contacts) {
        const bool target=selected(exclude_targets?profile_pairs_:distance_only_pairs_,pair.first.first,pair.first.second);
        if(target!=bool(exclude_targets))colliding=true;
      }
    }
    if(colliding) {
      best=0;
      for(const auto& p:collisions.contacts) {
        if(self_query && ((exclude_targets && selected(profile_pairs_,p.first.first,p.first.second)) || (!distance_only_pairs_.empty() && !selected(distance_only_pairs_,p.first.first,p.first.second))))continue;
        nearest={p.first.first,p.first.second};break;
      }
    } else for(size_t i=0;i<robot_objects.size();++i) {
      const auto& first=robot_objects[i];const auto& others=self_query?robot_objects:world_objects;
      for(size_t j=self_query?i+1:0;j<others.size();++j) {
        const auto& second=others[j];
        if(self_query && ((exclude_targets && selected(profile_pairs_,first.first,second.first)) || (!distance_only_pairs_.empty() && !selected(distance_only_pairs_,first.first,second.first))))continue;
        collision_detection::AllowedCollision::Type type;
        const auto& used_acm=override_acm?*override_acm:policy_acm_;
        if(self_query && (first.first==second.first || (used_acm.getAllowedCollision(first.first,second.first,type) && type==collision_detection::AllowedCollision::ALWAYS)))continue;
        if(first.second->getAABB().distance(second.second->getAABB())>best)continue;
        fcl::DistanceRequestd request(false);request.gjk_solver_type=fcl::GST_INDEP;
        fcl::DistanceResultd result;
        const double value=fcl::distance(first.second.get(),second.second.get(),request,result);
        need(std::isfinite(value),"FCL distance returned nonfinite value");
        if(std::max(0.0,value)<best) {best=std::max(0.0,value);nearest={first.first,second.first};}
      }
    }
    return J{{"separation_m",std::isfinite(best)?J(best):J(nullptr)},{"nearest_pair",nearest},{"available",std::isfinite(best)},
       {"solver","FCL/GST_INDEP"},{"meaning","unsigned set separation; overlap maps to zero; not penetration depth"}};
  };
  const bool collision=self.collision || env.collision;
  J response={{"identity",identity_},{"backend_status","OK"},{"collision_detected",collision},
          {"clearance",{{"self",distance_json(true,effective.collision,effective)},{"environment",distance_json(false,env.collision,env)},
            {"self_acm_applied",true},{"signed_penetration_distance",false}}},
          {"raw_self_collision",self.collision},{"effective_self_collision",effective.collision},
          {"effective_decision",effective.collision || env.collision ? "COLLIDING":"UNVERIFIED"},
          {"filtered_pairs",filtered},{"effective_self_pairs",effective_pairs},{"applied_policy",policy_},
          {"self_collision",self.collision},{"environment_collision",env.collision},{"colliding_pairs",pairs},{"contacts_base",contacts},
          {"pairs_may_be_truncated",self.contact_count>=req.max_contacts || env.contact_count>=req.max_contacts},
          {"decision",collision?"COLLIDING":"UNVERIFIED"},
          {"unverified_reasons",{"collision_coverage_unreviewed","ACM_model_scope_only","RJ45_tool_TCP_missing","gripper_uncalibrated","clearance_hardware_tolerance_unvalidated"}},
          {"joint_positions_used",used},{"link_transforms_base",transforms},
          {"T_base_to_ee",transforms.at(ee_)},{"object_count",objects_.size()},
          {"query_elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()}};
  if(semantic_enabled_) {
    collision_detection::CollisionResult sem,sem_bool;collision_detection::CollisionRequest bool_req;bool_req.contacts=false;
    const auto& acm=scene_->getAllowedCollisionMatrix();
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,sem,state,acm);
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(bool_req,sem_bool,state,acm);
    J blocking=J::array();for(const auto& c:sem.contacts)blocking.push_back({c.first.first,c.first.second});
    need(sem.collision==sem_bool.collision,"semantic contact/boolean disagreement");
    response["semantic_self_collision"]=sem_bool.collision;
    response["semantic_blocking_pairs"]=blocking;
    response["disabled_collision_pairs"]=semantic_audit_.at("actual_disabled_pairs");
    response["semantic_clearance"]={{"self",distance_json(true,sem_bool.collision,sem,0,&acm)},{"environment",response.at("clearance").at("environment")}};
    response["semantic_pair_list_may_be_truncated"]=sem.contact_count>=req.max_contacts;
    response["semantic_query_method"]="independent_MoveIt_FCL_contacts_and_boolean";
    response["hardware_validated"]=false;
  }
  if(in.contains("geometry_audit")) {
    J truncated=J::array();for(auto item:{std::make_pair("self",&self),std::make_pair("environment",&env),std::make_pair("effective_self",&effective)})
      for(const auto& p:item.second->contacts)if(p.second.size()>=req.max_contacts_per_pair)truncated.push_back({{"kind",item.first},{"links",{p.first.first,p.first.second}}});
    response["contact_collection"]={{"max_contacts_per_pair",req.max_contacts_per_pair},{"max_contacts",req.max_contacts},{"global_limit_reached",self.contact_count>=req.max_contacts || env.contact_count>=req.max_contacts || effective.contact_count>=req.max_contacts},{"pairs_at_limit",truncated},{"complete_contact_surface",false}};
    if(in.at("geometry_audit").contains("distance_pairs")) {
      const auto& requested=in.at("geometry_audit").at("distance_pairs");
      need(requested.is_array() && requested.size()<=32,"distance pair audit limit");J values=J::array();std::set<std::pair<std::string,std::string>> unique;
      for(const auto& pair:requested) {
        need(pair.is_array() && pair.size()==2,"invalid audit distance pair");const std::string a=pair.at(0),b=pair.at(1);
        need(a!=b && scene_->getRobotModel()->hasLinkModel(a) && scene_->getRobotModel()->hasLinkModel(b) && unique.insert(pair_key(a,b)).second,"invalid/duplicate audit distance pair");
        double best=std::numeric_limits<double>::infinity();bool overlap=false;
        for(const auto& first:robot_objects)if(first.first==a)for(const auto& second:robot_objects)if(second.first==b) {
          fcl::CollisionRequestd check;fcl::CollisionResultd checked;fcl::collide(first.second.get(),second.second.get(),check,checked);
          if(checked.isCollision()) {overlap=true;best=0;}
          else if(!overlap) {
            fcl::DistanceRequestd request(false);request.gjk_solver_type=fcl::GST_INDEP;fcl::DistanceResultd answer;
            double distance=fcl::distance(first.second.get(),second.second.get(),request,answer);need(std::isfinite(distance),"nonfinite pair distance");best=std::min(best,std::max(0.,distance));
          }
        }
        values.push_back({{"links",{a,b}},{"collision",overlap},{"available",std::isfinite(best)},{"separation_m",std::isfinite(best)?J(best):J(nullptr)},{"acm_applied",false},{"geometry","official_full_for_this_engine"},{"solver","FCL/GST_INDEP unsigned; collision uses FCL boolean"}});
      }
      response["pair_geometry_distances"]=values;
    }
  }
  if(refined_) {
    need(self.contact_count<req.max_contacts && effective.contact_count<req.max_contacts,"profile composition needs nontruncated pair coverage");
    auto derived=refined_->query(in);
    need(!derived.at("pairs_may_be_truncated").get<bool>(),"derived global contact limit reached");
    for(auto it=transforms.begin();it!=transforms.end();++it)need((matrix(it.value()).matrix()-matrix(derived.at("link_transforms_base").at(it.key())).matrix()).norm()<1e-11,"profile FK disagreement");
    response["official_raw_self_collision"]=self.collision;response["official_contacts_base"]=contacts;
    response["official_colliding_pairs"]=pairs;response["official_clearance"]=response["clearance"];
    auto merge=[&](const J& a,const J& b,bool named) {
      J result=J::array();for(const auto* data:{&a,&b})for(const auto& p:*data) {
        const bool environment=named && p.at("kind")=="environment";
        const std::string first=named?p.at("first").get<std::string>():p.at(0).get<std::string>(),second=named?p.at("second").get<std::string>():p.at(1).get<std::string>();
        const bool target=selected(profile_pairs_,first,second);
        if((data==&a && (environment || !target)) || (data==&b && !environment && target))result.push_back(p);
      }return result;
    };
    response["colliding_pairs"]=merge(pairs,derived.at("colliding_pairs"),true);
    response["contacts_base"]=merge(contacts,derived.at("contacts_base"),true);
    response["filtered_pairs"]=merge(filtered,derived.at("filtered_pairs"),false);
    response["effective_self_pairs"]=merge(effective_pairs,derived.at("effective_self_pairs"),false);
    bool raw=false;for(const auto& p:response["colliding_pairs"])if(p.at("kind")=="self")raw=true;
    const bool eff=!response["effective_self_pairs"].empty();response["raw_self_collision"]=raw;response["self_collision"]=raw;
    response["effective_self_collision"]=eff;response["collision_detected"]=raw || env.collision;
    response["decision"]=raw || env.collision?"COLLIDING":"UNVERIFIED";response["effective_decision"]=eff || env.collision?"COLLIDING":"UNVERIFIED";
    auto unchanged=distance_json(true,effective.collision,effective,1);auto changed=derived.at("clearance").at("self");
    J best=unchanged;if(!best.at("available").get<bool>() || (changed.at("available").get<bool>() && number(changed.at("separation_m"))<number(best.at("separation_m"))))best=changed;
    response["clearance"]["self"]=best;response["clearance"]["self"]["scope"]="pair-local refined target meshes; official meshes for all other pairs";
    response["geometry_profile"]=profile_;response["profile_target_pairs"]=profile_.at("target_pairs");
    if(response.contains("pair_geometry_distances")) {
      response["official_pair_geometry_distances"]=response["pair_geometry_distances"];
      auto& distances=response["pair_geometry_distances"];
      for(size_t i=0;i<distances.size();++i)if(selected(profile_pairs_,distances[i].at("links").at(0),distances[i].at("links").at(1))) {
        distances[i]=derived.at("pair_geometry_distances").at(i);distances[i]["geometry"]="derived_target_pair";
      }
    }
    if(in.contains("geometry_audit"))response["derived_contact_collection"]=derived.at("contact_collection");
  }
  if(in.contains("task_scope_canonical_json") || in.contains("task_scope_sha256") || in.contains("task_scope_context")) {
    need(!refined_,"task scope v0 requires official full geometry; diagnostic refinement unsupported");
    const std::string text=in.at("task_scope_canonical_json");const auto digest=hash(text);
    need(digest==in.at("task_scope_sha256") && in.at("task_scope_context")==identity_,"task scope/context identity mismatch");
    const auto scope=J::parse(text);
    need(scope.dump()==text,"task scope must be canonical JSON without ambiguous keys");
    const std::set<std::string> scope_keys={"version","id","source_model_sha256","existing_acm_policy_sha256","assembly_root","assembly_members","internal_diagnostic_pairs","blocking_rule","environment_rule","geometry_rule","missing_coverage","verification_status","evidence_references"};
    need(scope.is_object() && scope.size()==scope_keys.size(),"task scope fields");for(const auto& item:scope.items())need(scope_keys.count(item.key()),"unknown task scope field");
    need(scope.at("version").is_number_integer() && scope.at("version")==1 && !scope.at("id").get<std::string>().empty(),"task scope version/id");
    need(scope.at("source_model_sha256")==identity_.at("urdf_sha256") && scope.at("existing_acm_policy_sha256")==identity_.at("policy_sha256"),"task scope model/ACM mismatch");
    need(scope.at("assembly_root")=="gripper_link" && bool(urdf_->getLink("gripper_link")),"task scope gripper root");
    need(scope.at("blocking_rule")=="all_other_self_pairs_with_existing_acm" && scope.at("environment_rule")=="official_full_all_scene_objects" && scope.at("geometry_rule")=="official_full_only" && scope.at("verification_status")=="task_scoped_unverified","unsupported task scope rule");
    need(!scope.at("missing_coverage").empty() && !scope.at("evidence_references").empty(),"scope must retain missing coverage/evidence");
    std::set<std::string> members={"gripper_link"};bool progress=true;
    while(progress) {progress=false;for(const auto& [name,j]:urdf_->joints_)if(members.count(j->parent_link_name) && members.insert(j->child_link_name).second)progress=true;}
    const auto declared=scope.at("assembly_members").get<std::vector<std::string>>();
    need(std::set<std::string>(declared.begin(),declared.end())==members && declared.size()==members.size(),"scope assembly membership mismatch");
    std::vector<std::string> all_names;for(const auto& [name,l]:urdf_->links_)all_names.push_back(name);
    collision_detection::AllowedCollisionMatrix scoped_acm=policy_acm_,internal_acm(all_names,true);
    std::set<std::pair<std::string,std::string>> internal_pairs;
    for(const auto& item:scope.at("internal_diagnostic_pairs")) {
      need(item.is_object() && item.size()==5 && item.contains("links") && item.contains("mechanism") && item.contains("urdf_joint_path") && item.contains("evidence") && item.contains("unverified_factors"),"internal diagnostic fields");
      const auto names=item.at("links").get<std::vector<std::string>>();need(names.size()==2 && names[0]!=names[1] && members.count(names[0]) && members.count(names[1]),"illegal internal pair");
      need(internal_pairs.insert(pair_key(names[0],names[1])).second,"duplicate internal pair");
      collision_detection::AllowedCollision::Type kind;need(!policy_acm_.getAllowedCollision(names[0],names[1],kind) || kind!=collision_detection::AllowedCollision::ALWAYS,"internal pair already ACM-allowed");
      const auto mechanism=item.at("mechanism").get<std::string>();need(mechanism=="gear_meshing_candidate" || mechanism=="joint_bearing_candidate" || mechanism=="linkage_assembly_candidate" || mechanism=="unresolved_internal_contact","invalid mechanism claim");
      need(!item.at("evidence").get<std::string>().empty() && !item.at("unverified_factors").empty(),"unverified internal evidence required");
      auto ancestry=[&](std::string name) {std::vector<std::string> result={name};auto l=urdf_->getLink(name);while(l->parent_joint){l=urdf_->getLink(l->parent_joint->parent_link_name);result.push_back(l->name);}return result;};
      auto a=ancestry(names[0]),b=ancestry(names[1]);std::string common;
      for(const auto& n:b)if(std::find(a.begin(),a.end(),n)!=a.end()){common=n;break;}
      std::vector<std::string> path;for(const auto& name:names){auto l=urdf_->getLink(name);while(l->name!=common){path.push_back(l->parent_joint->name);l=urdf_->getLink(l->parent_joint->parent_link_name);}}
      auto expected=item.at("urdf_joint_path").get<std::vector<std::string>>();std::sort(path.begin(),path.end());std::sort(expected.begin(),expected.end());need(path==expected,"internal pair topology mismatch");
      scoped_acm.setEntry(names[0],names[1],true);internal_acm.setEntry(names[0],names[1],false);
    }
    need(!internal_pairs.empty(),"explicit internal diagnostics required");
    collision_detection::CollisionResult blocking,internal,blocking_bool,internal_bool,environment_bool;
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,blocking,state,scoped_acm);
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(req,internal,state,internal_acm);
    collision_detection::CollisionRequest boolean_req;boolean_req.contacts=false;
    // Authoritative complete searches: never infer candidate by editing a capped contact list.
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(boolean_req,blocking_bool,state,scoped_acm);
    scene_->getCollisionEnvUnpadded()->checkSelfCollision(boolean_req,internal_bool,state,internal_acm);
    scene_->getCollisionEnvUnpadded()->checkRobotCollision(boolean_req,environment_bool,state,raw_acm_);
    need(environment_bool.collision==env.collision,"task/official environment disagreement");
    auto list_pairs=[](const collision_detection::CollisionResult& r){J result=J::array();for(const auto& p:r.contacts)result.push_back({p.first.first,p.first.second});return result;};
    J internal_contacts=J::array();for(const auto& c:contacts)if(c.at("kind")=="self" && selected(internal_pairs,c.at("first"),c.at("second")))internal_contacts.push_back(c);
    // Query internal contacts independently too: raw global contacts may have been capped before reaching these pairs.
    internal_contacts=J::array();for(const auto& pair:internal.contacts)for(const auto& contact:pair.second){Eigen::Vector3d p=root_to_base.inverse()*contact.pos;internal_contacts.push_back({{"kind","internal_diagnostic"},{"first",pair.first.first},{"second",pair.first.second},{"position_base_m",{p.x(),p.y(),p.z()}},{"depth_m",contact.depth},{"source","independent Native FCL internal query"}});}
    response["task_scope"]={{"scope_sha256",digest},{"scope_id",scope.at("id")},{"binding",identity_},{"official_raw_self_collision",self.collision},{"official_effective_self_collision",effective.collision},{"task_scope_blocking_self_collision",blocking_bool.collision},{"gripper_internal_collision",internal_bool.collision},{"environment_collision",environment_bool.collision},{"zero_clearance_candidate",!blocking_bool.collision && !environment_bool.collision},{"task_scope_blocking_pairs",list_pairs(blocking)},{"gripper_internal_pairs",list_pairs(internal)},{"gripper_internal_contacts",internal_contacts},{"declared_internal_pairs",scope.at("internal_diagnostic_pairs")},{"gripper_internal_verification","UNVERIFIED_MECHANISM"},{"missing_coverage",scope.at("missing_coverage")},{"clearance",{{"self",distance_json(true,blocking_bool.collision,blocking,0,&scoped_acm)},{"environment",response.at("clearance").at("environment")}}},{"blocking_pair_list_may_be_truncated",blocking.contact_count>=req.max_contacts},{"internal_pair_list_may_be_truncated",internal.contact_count>=req.max_contacts},{"contact_surface_complete",false},{"candidate_computed_from_fresh_boolean_queries",true}};
  }
  response["query_elapsed_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return response;
}
}
