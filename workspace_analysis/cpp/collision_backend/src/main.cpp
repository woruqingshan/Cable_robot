#include "engine.hpp"
#include <iostream>
#include <memory>
#include <set>
#include <cstdlib>

int main() {
  setenv("RCUTILS_LOGGING_USE_STDOUT","0",1);
  std::unique_ptr<cable::Engine> engine;std::set<std::string> seen;std::string line;
  while(std::getline(std::cin,line)) {
    cable::J response={{"protocol_version",1},{"id",nullptr},{"status","ERROR"},{"identity",nullptr}};
    bool quit=false;
    try {
      if(line.size()>16*1024*1024)throw std::runtime_error("request exceeds 16 MiB");
      std::vector<std::set<std::string>> keys;
      auto callback=[&](int, cable::J::parse_event_t event, cable::J& value) {
        if(event==cable::J::parse_event_t::object_start)keys.emplace_back();
        if(event==cable::J::parse_event_t::key && !keys.back().insert(value.get<std::string>()).second)
          throw std::runtime_error("duplicate JSON key");
        if(event==cable::J::parse_event_t::object_end)keys.pop_back();
        return true;
      };
      auto in=cable::J::parse(line,callback);
      if(!in.at("id").is_string())throw std::runtime_error("id must be string");
      response["id"]=in.at("id");const std::string id=in.at("id");
      if(id.empty() || !seen.insert(id).second)throw std::runtime_error("duplicate/empty request id");
      if(in.at("protocol_version")!=1)throw std::runtime_error("unsupported protocol");
      const auto op=in.at("op").get<std::string>();
      std::set<std::string> allowed={"protocol_version","id","op"};
      if(op=="INIT")for(const auto* key:{"model","scene_canonical_json","scene_sha256","objects"})allowed.insert(key);
      if(op=="QUERY")allowed.insert("joint_positions");
      if(op=="POLICY") {allowed.insert("policy_canonical_json");allowed.insert("policy_sha256");}
      for(const auto& field:in.items())if(!allowed.count(field.key()))throw std::runtime_error("unknown request option");
      if(op=="INIT") {
        if(engine)throw std::runtime_error("already initialized");
        auto candidate=std::make_unique<cable::Engine>();response.update(candidate->initialize(in));engine=std::move(candidate);
      } else if(op=="POLICY") {
        if(!engine)throw std::runtime_error("not initialized");response.update(engine->set_policy(in));
      } else if(op=="QUERY") {
        if(!engine)throw std::runtime_error("not initialized");response.update(engine->query(in));
      } else if(op=="SHUTDOWN") { if(engine)response["identity"]=engine->identity();response["backend_status"]="STOPPED";quit=true; }
      else throw std::runtime_error("unsupported operation");
      response["status"]="OK";
    } catch(const std::exception& e) {
      response["error"]=e.what();response["backend_status"]="ERROR";response["decision"]="ERROR";
      if(engine)response["identity"]=engine->identity();
      std::cerr << "collision_backend: " << e.what() << '\n';
    }
    std::cout << response.dump() << std::endl;if(quit)return 0;
  }
  return 0;
}
