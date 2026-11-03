// GraspWeld: plugin de sistema de Gazebo Fortress para el gripper de rover_arm_sim.
//
// Qué hace (y nada más):
//   * Lee los contactos de las colisiones de las dos mordazas (crea el componente ContactSensorData
//     en ellas, así el sistema de física los llena; no hace falta un sensor de contacto).
//   * Si el comando es "hold" y las DOS mordazas tocan el mismo modelo (no estático, no el robot)
//     durante <min_contact_time>, lo une a <parent_link> con una unión fija (componente
//     DetachableJoint, el mismo mecanismo que usa el sistema DetachableJoint de Gazebo).
//   * Con cualquier comando distinto de "hold" suelta el objeto.
//   * Publica su estado (qué toca cada mordaza y qué sostiene) como JSON a 10 Hz.
// A diferencia del sistema DetachableJoint de serie, arranca sin nada sujeto y sirve para cualquier
// objeto del mundo (no hay que declarar cada objeto). La decisión de cuándo sujetar la toma el
// emulador de drivers (rover_arm_sim/emulator_node.py), que sabe si el operador está cerrando.

#include <chrono>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace rover_arm_sim
{
using ignition::gazebo::Entity;
using ignition::gazebo::EntityComponentManager;
using ignition::gazebo::kNullEntity;
namespace components = ignition::gazebo::components;

class GraspWeld
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(const Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
                 EntityComponentManager & _ecm, ignition::gazebo::EventManager &) override
  {
    this->model_ = ignition::gazebo::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      ignerr << "GraspWeld debe ir dentro de un <model>." << std::endl;
      return;
    }
    auto sdf = _sdf->Clone();
    this->parentLinkName_ = sdf->Get<std::string>("parent_link");
    if (sdf->HasElement("finger_link")) {
      for (auto e = sdf->GetElement("finger_link"); e; e = e->GetNextElement("finger_link")) {
        this->fingerNames_.push_back(e->Get<std::string>());
      }
    }
    const auto commandTopic = sdf->Get<std::string>("command_topic", "/rover_arm_sim/grasp/command").first;
    const auto stateTopic = sdf->Get<std::string>("state_topic", "/rover_arm_sim/grasp/state").first;
    const double minContact = sdf->Get<double>("min_contact_time", 0.05).first;
    this->minContact_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(minContact));
    if (this->parentLinkName_.empty() || this->fingerNames_.size() < 2) {
      ignerr << "GraspWeld necesita <parent_link> y dos <finger_link>." << std::endl;
      return;
    }
    this->node_.Subscribe(commandTopic, &GraspWeld::OnCommand, this);
    this->pub_ = this->node_.Advertise<ignition::msgs::StringMsg>(stateTopic);
    this->configured_ = true;
    ignmsg << "GraspWeld: comando en [" << commandTopic << "], estado en [" << stateTopic << "]"
           << std::endl;
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo & _info, EntityComponentManager & _ecm) override
  {
    if (!this->configured_) {
      return;
    }
    if (!this->initialized_ && !this->Init(_ecm)) {
      return;
    }
    if (_info.paused) {
      return;
    }
    std::string command;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      command = this->command_;
    }

    // Modelos que toca cada mordaza
    std::vector<std::set<Entity>> touching(this->fingerCollisions_.size());
    for (size_t i = 0; i < this->fingerCollisions_.size(); ++i) {
      for (Entity c : this->fingerCollisions_[i]) {
        auto data = _ecm.Component<components::ContactSensorData>(c);
        if (!data) {
          continue;
        }
        for (const auto & contact : data->Data().contact()) {
          const Entity a = contact.collision1().id();
          const Entity b = contact.collision2().id();
          const Entity other = (a == c) ? b : a;
          const Entity m = ignition::gazebo::topLevelModel(other, _ecm);
          if (m == kNullEntity || m == this->model_.Entity()) {
            continue;
          }
          auto isStatic = _ecm.Component<components::Static>(m);
          if (isStatic && isStatic->Data()) {
            continue;
          }
          touching[i].insert(m);
        }
      }
    }
    std::set<Entity> common = touching[0];
    for (size_t i = 1; i < touching.size(); ++i) {
      std::set<Entity> both;
      for (Entity e : common) {
        if (touching[i].count(e)) {
          both.insert(e);
        }
      }
      common = both;
    }
    const auto now = _info.simTime;
    if (common.empty()) {
      this->candidate_ = kNullEntity;
    } else if (!common.count(this->candidate_)) {
      this->candidate_ = *common.begin();
      this->candidateSince_ = now;
    }

    if (this->joint_ != kNullEntity && command != "hold") {
      _ecm.RequestRemoveEntity(this->joint_);
      ignmsg << "GraspWeld: suelta " << this->Name(this->attached_, _ecm) << std::endl;
      this->joint_ = kNullEntity;
      this->attached_ = kNullEntity;
    }
    if (this->joint_ == kNullEntity && command == "hold" && this->candidate_ != kNullEntity &&
        now - this->candidateSince_ >= this->minContact_)
    {
      Entity childLink = _ecm.EntityByComponents(
        components::Link(), components::ParentEntity(this->candidate_), components::CanonicalLink());
      if (childLink == kNullEntity) {
        auto links = _ecm.ChildrenByComponents(this->candidate_, components::Link());
        if (!links.empty()) {
          childLink = links.front();
        }
      }
      if (childLink != kNullEntity) {
        this->joint_ = _ecm.CreateEntity();
        _ecm.CreateComponent(this->joint_, components::DetachableJoint(
          {this->parentLink_, childLink, "fixed"}));
        this->attached_ = this->candidate_;
        ignmsg << "GraspWeld: sujeta " << this->Name(this->attached_, _ecm) << std::endl;
      }
    }

    if (now - this->lastPublish_ >= std::chrono::milliseconds(100)) {
      this->lastPublish_ = now;
      std::ostringstream json;
      json << "{\"attached\": \"" << this->Name(this->attached_, _ecm) << "\", \"touching\": [";
      for (size_t i = 0; i < touching.size(); ++i) {
        json << (i ? ", " : "") << "[";
        size_t k = 0;
        for (Entity e : touching[i]) {
          json << (k++ ? ", " : "") << "\"" << this->Name(e, _ecm) << "\"";
        }
        json << "]";
      }
      json << "]}";
      ignition::msgs::StringMsg msg;
      msg.set_data(json.str());
      this->pub_.Publish(msg);
    }
  }

private:
  bool Init(EntityComponentManager & _ecm)
  {
    this->parentLink_ = this->model_.LinkByName(_ecm, this->parentLinkName_);
    if (this->parentLink_ == kNullEntity) {
      return false;
    }
    this->fingerCollisions_.clear();
    for (const auto & name : this->fingerNames_) {
      const Entity link = this->model_.LinkByName(_ecm, name);
      if (link == kNullEntity) {
        return false;
      }
      auto collisions = _ecm.ChildrenByComponents(link, components::Collision());
      for (Entity c : collisions) {
        if (!_ecm.Component<components::ContactSensorData>(c)) {
          _ecm.CreateComponent(c, components::ContactSensorData());
        }
      }
      this->fingerCollisions_.push_back(collisions);
    }
    this->initialized_ = true;
    ignmsg << "GraspWeld listo: " << this->fingerNames_.size() << " mordazas en "
           << this->parentLinkName_ << std::endl;
    return true;
  }

  std::string Name(Entity _e, const EntityComponentManager & _ecm) const
  {
    if (_e == kNullEntity) {
      return "";
    }
    auto n = _ecm.Component<components::Name>(_e);
    return n ? n->Data() : std::to_string(_e);
  }

  void OnCommand(const ignition::msgs::StringMsg & _msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->command_ = _msg.data();
  }

  ignition::gazebo::Model model_{kNullEntity};
  std::string parentLinkName_;
  std::vector<std::string> fingerNames_;
  std::vector<std::vector<Entity>> fingerCollisions_;
  Entity parentLink_{kNullEntity};
  Entity candidate_{kNullEntity};
  Entity joint_{kNullEntity};
  Entity attached_{kNullEntity};
  std::chrono::steady_clock::duration candidateSince_{0};
  std::chrono::steady_clock::duration lastPublish_{0};
  std::chrono::steady_clock::duration minContact_{0};
  bool configured_{false};
  bool initialized_{false};
  std::mutex mutex_;
  std::string command_{"release"};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;
};
}  // namespace rover_arm_sim

IGNITION_ADD_PLUGIN(
  rover_arm_sim::GraspWeld, ignition::gazebo::System,
  rover_arm_sim::GraspWeld::ISystemConfigure, rover_arm_sim::GraspWeld::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(rover_arm_sim::GraspWeld, "rover_arm_sim::GraspWeld")
