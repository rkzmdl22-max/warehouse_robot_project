#include <cmath>
#include <functional>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo {
class ClaudeGait : public ModelPlugin {
 public:
  void Load(physics::ModelPtr m, sdf::ElementPtr) override {
    model = m;
    const char* n[6] = {"abad_L_Joint","hip_L_Joint","knee_L_Joint","abad_R_Joint","hip_R_Joint","knee_R_Joint"};
    for (int i = 0; i < 6; i++) joints.push_back(model->GetJoint(n[i]));
    pose0 = model->WorldPose();
    conn = event::Events::ConnectWorldUpdateBegin(std::bind(&ClaudeGait::OnUpdate, this));
    gzmsg << "claude_gait loaded\n";
  }
  void OnUpdate() {
    double now = model->GetWorld()->SimTime().Double();
    if (t0 < 0) t0 = now;
    double t = now - t0;
    bool w = t < T;
    double s = w ? std::sin(2*M_PI*F*t) : 0.0;
    double q[6] = {0, HIP*s, KNEE*std::max(0.0, s), 0, HIP*s, KNEE*std::max(0.0, -s)};
    for (int i = 0; i < 6; i++) if (joints[i]) joints[i]->SetPosition(0, q[i]);
    double x = pose0.Pos().X() + V*(w ? t : T)*std::cos(YAW);
    model->SetWorldPose(ignition::math::Pose3d(x, pose0.Pos().Y() + V*(w ? t : T)*std::sin(YAW), pose0.Pos().Z(), 0, 0, YAW));
    model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
  }
 private:
  physics::ModelPtr model;
  std::vector<physics::JointPtr> joints;
  event::ConnectionPtr conn;
  ignition::math::Pose3d pose0;
  double t0 = -1;
  const double HIP = 0.24, KNEE = 0.32, F = 1.2, T = 14.0, V = 3.77/14.0, YAW = 2.583;
};
GZ_REGISTER_MODEL_PLUGIN(ClaudeGait)
}
