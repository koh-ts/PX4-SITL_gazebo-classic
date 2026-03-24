#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class GazeboDynamicWindPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();

    std::string link_name = "base_link";
    if (sdf->HasElement("linkName")) link_name = sdf->Get<std::string>("linkName");
    link_ = model_->GetLink(link_name);
    if (!link_) {
      gzerr << "[dynamic_wind] link '" << link_name << "' not found\n";
      return;
    }

    // 初期風（フォールバック）
    if (sdf->HasElement("windVelocity")) {
      auto v = sdf->Get<ignition::math::Vector3d>("windVelocity");
      wind_vel_ = v;
    }

    // ~/wind を Vector3d で購読
    node_ = transport::NodePtr(new transport::Node());
    node_->Init(world_->Name());
    wind_sub_ = node_->Subscribe("~/wind", &GazeboDynamicWindPlugin::OnWindMsg, this);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboDynamicWindPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[dynamic_wind] loaded, listening ~/wind (Vector3d)\n";
  }

private:
  void OnWindMsg(ConstVector3dPtr &msg) {
    wind_vel_.Set(msg->x(), msg->y(), msg->z());
  }

  void OnUpdate(const common::UpdateInfo &) {
    if (!link_) return;
    // 簡易モデル: 風速ベクトルをそのまま力として適用
    ignition::math::Vector3d force = wind_vel_;
    link_->AddForce(force);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_conn_;
  transport::NodePtr node_;
  transport::SubscriberPtr wind_sub_;
  ignition::math::Vector3d wind_vel_{0, 0, 0};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboDynamicWindPlugin)

}  // namespace gazebo