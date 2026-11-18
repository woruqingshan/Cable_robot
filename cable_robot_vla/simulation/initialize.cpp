// Simulator state initialization only. No ROS or hardware communication.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/plugin/Register.hh>
#include <vector>
#include <string>
class Stage23Initialize : public ignition::gazebo::System,
                         public ignition::gazebo::ISystemConfigure,
                         public ignition::gazebo::ISystemPreUpdate {
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  std::vector<std::pair<std::string,double>> positions;
  bool done=false;
 public:
  void Configure(const ignition::gazebo::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &, ignition::gazebo::EventManager &) override {
    model=ignition::gazebo::Model(entity);
    auto copy=sdf->Clone();
    for(auto j=copy->GetElement("joint"); j; j=j->GetNextElement("joint"))
      positions.emplace_back(j->Get<std::string>("name"),j->Get<double>("position"));
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if(done) return;
    for(const auto &entry:positions)
      if(model.JointByName(ecm,entry.first)==ignition::gazebo::kNullEntity) return;
    for(const auto &entry:positions) {
      const auto entity=model.JointByName(ecm,entry.first);
      ecm.CreateComponent(entity,ignition::gazebo::components::JointPositionReset({entry.second}));
    }
    done=true;
  }
};
IGNITION_ADD_PLUGIN(Stage23Initialize,ignition::gazebo::System,
                   Stage23Initialize::ISystemConfigure,Stage23Initialize::ISystemPreUpdate)
